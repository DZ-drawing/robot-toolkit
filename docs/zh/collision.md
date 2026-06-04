---
> 本文档是 [English Original](../collision.md) 的中文翻译。
---

# 碰撞检测

## 概述

robot-toolkit 为机器人机械臂提供三层碰撞检测系统：

| 层级 | 方法 | 适用场景 | 性能 |
|------|------|---------|------|
| **Tier 1** | 解析公式 | 基本体之间的碰撞（球体、胶囊体、长方体） | O(1) 每对 |
| **Tier 2** | GJK / EPA | 任何涉及 `TriangleMesh` 的碰撞对 | 迭代法，通常 8–32 次 |
| **Tier 3** | `CollisionChecker` | 完整机器人碰撞调度（自动选择 Tier 1 或 2） | 取决于几何体类型 |

- **Tier 1** 使用封闭形式的距离公式处理常见基本体形状。
- **Tier 2** 使用 GJK 算法进行交叉/距离查询，当形状重叠时使用 EPA
  计算穿透深度和接触信息。
- **Tier 3** 是统一的 `CollisionChecker`，根据输入对的几何体类型自动
  调度到合适的层级。

---

## TriangleMesh（三角面片凸包碰撞体）

`TriangleMesh` 是一个凸三角面片网格（convex triangle mesh）数据类，作为
GJK/EPA 管线的主要几何表示。它包含顶点位置、三角面索引、4x4 位姿变换和
可选的名称。

```python
from robot_ik.collision import TriangleMesh
import numpy as np
```

### 数据字段

| 字段 | 类型 | 描述 |
|------|------|------|
| `vertices` | `np.ndarray (N, 3)` | 局部坐标系下的顶点位置 |
| `faces` | `np.ndarray (M, 3, int)` | 三角面索引（指向 `vertices`） |
| `pose` | `np.ndarray (4, 4)` | 齐次变换矩阵（局部 → 世界） |
| `name` | `str` | 网格标识符 |

### 支撑函数（Support Function）

**支撑函数**返回沿给定方向最远的顶点。这是 GJK 算法的核心操作。它通过将
查询方向变换到局部坐标系后再进行顶点搜索，从而正确处理旋转变换。

```python
mesh = TriangleMesh.from_box([1.0, 1.0, 1.0])
direction = np.array([1.0, 0.0, 0.0])
extreme = mesh.support(direction)  # 沿 +X 方向最远的顶点
```

### 更新位姿

```python
T = np.eye(4)
T[:3, 3] = [0.5, 0.0, 0.0]  # 沿 X 轴平移 0.5 m
mesh.update_pose(T)
```

### 工厂方法（Factory Methods）

#### `from_box(size, pose=None, name="box")`

创建包含 8 个顶点和 12 个三角面的长方体网格。

```python
mesh = TriangleMesh.from_box([0.6, 0.4, 0.2])
```

#### `from_sphere(radius=1.0, subdivisions=1, pose=None, name="sphere")`

创建二十面体球近似网格。`subdivisions=0` 产生 20 个面；
每增加一级细分数，面数增加 4 倍。

```python
mesh = TriangleMesh.from_sphere(radius=0.05, subdivisions=2)
```

#### `from_capsule(p1, p2, radius=0.05, subdivisions=1, pose=None, name="capsule")`

通过生成表面点集的凸包创建胶囊体网格。

```python
mesh = TriangleMesh.from_capsule(
    p1=np.array([0, 0, 0]),
    p2=np.array([0, 0, 0.3]),
    radius=0.04,
)
```

#### `from_convex_hull(points, pose=None, name="convex_hull")`

使用 `scipy.spatial.ConvexHull` 从任意点集创建凸包网格。

```python
points = np.random.randn(50, 3)  # 随机点云
mesh = TriangleMesh.from_convex_hull(points)
```

---

## GJK 算法

**Gilbert–Johnson–Keerthi (GJK)** 算法在两个凸形状的闵可夫斯基差
（Minkowski difference）上操作。两个形状相交当且仅当原点位于其闵可夫斯基
差的内部或边界上。

```python
from robot_ik.collision import gjk_intersect, gjk_distance
```

### `gjk_intersect(shape_a, shape_b, max_iterations=64) -> bool`

判断两个凸形状是否相交，返回 `True` 或 `False`。算法在闵可夫斯基差中
构建单纯形（simplex）并迭代地向原点方向优化。

```python
box = TriangleMesh.from_box([1.0, 1.0, 1.0])
sphere = TriangleMesh.from_sphere(radius=0.5)

overlapping = gjk_intersect(box, sphere)
print(f"是否重叠: {overlapping}")
```

### `gjk_distance(shape_a, shape_b, max_iterations=64) -> tuple`

返回 `(distance, closest_on_a, closest_on_b)`：
- `distance`：欧几里得分离距离（重叠时为 0.0）。
- `closest_on_a`、`closest_on_b`：世界坐标系中的最近点（3D 向量），
  重叠时为 `None`。

```python
box = TriangleMesh.from_box([1.0, 1.0, 1.0])
box.update_pose(np.eye(4))  # 位于原点

sphere = TriangleMesh.from_sphere(radius=0.3)
T = np.eye(4)
T[:3, 3] = [1.5, 0.0, 0.0]  # 沿 X 轴偏移
sphere.update_pose(T)

dist, pt_a, pt_b = gjk_distance(box, sphere)
print(f"距离: {dist:.4f}")
print(f"长方体上最近点: {pt_a}")
print(f"球体上最近点:   {pt_b}")
```

---

## EPA 算法

**扩展多面体算法（Expanding Polytope Algorithm, EPA）** 在两个凸形状重叠时
计算穿透深度、碰撞法线和接触点。它基于 GJK 结果，将单纯形扩展为逼近
闵可夫斯基差边界的多面体。

```python
from robot_ik.collision import epa_penetration
```

### `epa_penetration(shape_a, shape_b, max_iterations=64, tolerance=1e-6) -> tuple`

返回 `(depth, normal, contact_point)`：
- `depth`（≥ 0）：穿透深度。
- `normal`：单位碰撞法线（从 B 指向 A）。
- `contact_point`：世界坐标系下的接触点（最近见证点对的中点）。

```python
box = TriangleMesh.from_box([1.0, 1.0, 1.0])
sphere = TriangleMesh.from_sphere(radius=0.6)

depth, normal, contact = epa_penetration(box, sphere)
print(f"穿透深度: {depth:.4f}")
print(f"碰撞法线: {normal}")
print(f"接触点:   {contact}")
```

> **注意：** EPA 内部会先调用 `gjk_intersect`。如果形状未重叠，将返回
> `(0.0, [1,0,0], [0,0,0])`。

---

## CollisionChecker 集成

`CollisionChecker` 是统一入口，管理机器人连杆几何体和环境障碍物，自动
调度到合适的碰撞方法。

```python
from robot_ik.collision import (
    CollisionChecker, Sphere, Capsule, Box, TriangleMesh,
)
```

### 三层调度逻辑

```
_check_geometry_collision(g1, g2)
├── 任一为 TriangleMesh → Tier 2: GJK/EPA (_check_pair_mesh)
└── 否则                → Tier 1: 解析公式 (_check_pair_primitive)
```

当选择 Tier 2 时，基本体几何体会通过各自的 `.to_mesh()` 方法自动转换为
`TriangleMesh`，然后运行 GJK/EPA。

### 设置机器人

```python
checker = CollisionChecker()

# 添加连杆几何体（基本体）
checker.add_link_geometry("link1", Sphere(radius=0.05))
checker.add_link_geometry("link2", Capsule(
    p1=np.array([0, 0, 0]),
    p2=np.array([0, 0, 0.3]),
    radius=0.04,
))

# 添加环境障碍物（网格）
obstacle = TriangleMesh.from_box([2.0, 2.0, 0.1])
obstacle.update_pose(np.eye(4))  # 地面
checker.add_obstacle(obstacle)
```

### 自碰撞检测（Self-Collision）

```python
link_transforms = {
    "link1": np.eye(4),
    "link2": np.eye(4),
}

result = checker.check_self_collision(link_transforms, ignore_adjacent=True)
if result is not None:
    print(f"自碰撞: {result.pair}, 穿透深度={-result.distance:.4f}")
else:
    print("无自碰撞")
```

### 环境碰撞检测（Environment Collision）

```python
result = checker.check_environment_collision(link_transforms)
if result is not None:
    print(f"环境碰撞: {result.pair}")
```

### `to_mesh()` 便捷方法

每个基本体（`Sphere`、`Capsule`、`Box`）都提供 `.to_mesh()` 方法，用于
手动执行 Tier 2 查询：

```python
sphere = Sphere(radius=0.05)
sphere.pose = np.eye(4)
mesh = sphere.to_mesh()  # TriangleMesh（二十面体球）

box = Box(size=np.array([0.1, 0.1, 0.1]))
box_mesh = box.to_mesh()  # TriangleMesh（12 个三角面）
```

### CollisionResult（碰撞结果）

| 字段 | 类型 | 描述 |
|------|------|------|
| `is_colliding` | `bool` | 是否发生碰撞 |
| `distance` | `float` | 有符号距离（穿透时为负值） |
| `contact_point` | `np.ndarray or None` | 世界坐标系下的接触点 |
| `pair` | `tuple[str, str]` | 碰撞对的名称 |

---

## API 参考

### 核心函数

| 函数 | 签名 | 返回值 |
|------|------|--------|
| `gjk_intersect` | `(TriangleMesh, TriangleMesh, max_iter=64)` | `bool` |
| `gjk_distance` | `(TriangleMesh, TriangleMesh, max_iter=64)` | `(float, ndarray, ndarray)` |
| `epa_penetration` | `(TriangleMesh, TriangleMesh, max_iter=64, tol=1e-6)` | `(float, ndarray, ndarray)` |

### 基本体距离函数

| 函数 | 配对 |
|------|------|
| `distance_sphere_to_sphere` | 球体 ↔ 球体 |
| `distance_sphere_to_capsule` | 球体 ↔ 胶囊体 |
| `distance_capsule_to_capsule` | 胶囊体 ↔ 胶囊体 |
| `distance_sphere_to_box` | 球体 ↔ 长方体 |
| `distance_box_to_box` | 长方体 ↔ 长方体 |
| `distance_point_to_sphere` | 点 ↔ 球体 |
| `distance_point_to_box` | 点 ↔ 长方体 |

### 类

| 类 | 描述 |
|------|------|
| `TriangleMesh` | 具有 GJK 支撑函数的凸三角面片网格 |
| `Sphere` | 球体基本体，支持 `.to_mesh()` |
| `Capsule` | 胶囊体基本体，支持 `.to_mesh()` |
| `Box` | 长方体基本体，支持 `.to_mesh()` |
| `CollisionChecker` | 统一碰撞调度器 |
| `CollisionResult` | 碰撞结果数据类 |

---

## 性能特征

| 操作 | 复杂度 | 典型迭代次数 |
|------|--------|------------|
| 球体–球体（Tier 1） | O(1) | 1 |
| 胶囊体–胶囊体（Tier 1） | O(1) | 1 |
| GJK 相交检测（Tier 2） | O(k·n) | 4–16 |
| GJK 距离查询（Tier 2） | O(k·n) | 8–32 |
| EPA 穿透计算（Tier 2） | O(k·n²) | 8–32 |

其中 `k` = 迭代次数（由 `max_iterations` 限制），`n` = 网格顶点数。
支撑函数每次调用的朴素实现为 O(n)（遍历所有顶点），因此 GJK 的整体
开销为 O(k·n)。EPA 由于需要 BFS 地平线计算，最坏情况下额外开销约为
O(k·n²)。

**性能优化建议：**
- 尽可能使用 Tier 1 基本体——它们比 GJK 快得多。
- 保持网格顶点数较低。长方体（8 个顶点）比高度细分的球体便宜得多。
- 球体/胶囊体网格使用 `subdivisions=0` 或 `1`，除非需要高精度。
- 缓存网格转换：调用 `.to_mesh()` 一次并重复使用结果，而非每帧重新转换。
