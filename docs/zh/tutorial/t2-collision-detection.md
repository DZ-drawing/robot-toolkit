[English](../../tutorial/t2-collision-detection.md) | **中文**

# 教程 2：自碰撞检测

**目标**：使用基本几何形状检测机器人自身连杆之间以及机器人和环境障碍物之间的碰撞。

**演示功能**：
- 带有 `Sphere`、`Capsule` 和 `Box` 基本几何体的 `CollisionChecker`
- `add_link_geometry` 和 `add_obstacle` 用于构建碰撞世界
- 带相邻连杆过滤的 `check_self_collision`
- `check_environment_collision` 用于障碍物回避

---

## 工作原理

`CollisionChecker` 存储以连杆名称为键的碰撞几何体（球体、胶囊体、长方体）。在每个配置下，我们提供每个连杆的 4x4 齐次变换矩阵；碰撞检测器将每个基本几何体变换到世界坐标系并测试距离。间隙小于或等于零的配对被报告为碰撞。

```python
"""教程 2：自碰撞检测。

为 6 自由度机器人设置简单的碰撞世界，扫描各种配置，
并报告任何自碰撞或环境碰撞。

运行方式：python -m docs.tutorial.t2_collision_detection
"""

import numpy as np
from robot_ik.collision import (
    CollisionChecker,
    CollisionResult,
    Sphere,
    Capsule,
    Box,
)


def build_collision_model(base_offset: np.ndarray = np.zeros(3)):
    """构建一个带有胶囊体连杆几何体和长方体障碍物的 CollisionChecker。

    参数：
        base_offset: 应用于所有连杆几何体的平移（用于 arm2）。

    返回：
        (checker, link_names) 元组。
    """
    checker = CollisionChecker()
    # 6 自由度铰接机器人的近似连杆长度（米）
    link_lengths = [0.0, 0.3, 0.5, 0.1, 0.4, 0.1]

    link_names = []
    for i in range(6):
        name = f"link{i}"
        link_names.append(name)
        # 每个连杆建模为沿局部 Z 轴的胶囊体
        checker.add_link_geometry(
            name,
            Capsule(
                p1=np.array([0.0, 0.0, 0.0]) + base_offset,
                p2=np.array([0.0, 0.0, link_lengths[i]]) + base_offset,
                radius=0.05,
            ),
        )

    # 在 z = 0 处添加一个类似桌子的长方体障碍物
    obstacle_pose = np.eye(4)
    obstacle_pose[:3, 3] = np.array([0.4, 0.0, -0.05])
    checker.add_obstacle(
        Box(size=np.array([0.8, 0.8, 0.1]), pose=obstacle_pose)
    )

    return checker, link_names


def make_link_transforms(
    q: np.ndarray,
    link_names: list[str],
    base_offset: np.ndarray = np.zeros(3),
) -> dict[str, np.ndarray]:
    """从关节角度构建 4x4 连杆变换的字典。

    这是一个简化的占位实现；真正的实现会遍历 DH 运动链。
    每个连杆坐标系沿 Z 偏移近似连杆长度，旋转角度等于关节角度。

    参数：
        q: 关节角度 (6,)。
        link_names: 连杆名称列表。
        base_offset: 基座的世界坐标系偏移。

    返回：
        连杆名称到 4x4 变换的映射字典。
    """
    link_lengths = [0.0, 0.3, 0.5, 0.1, 0.4, 0.1]
    T = np.eye(4)
    T[:3, 3] = base_offset

    transforms: dict[str, np.ndarray] = {}
    for i, name in enumerate(link_names):
        # 绕 Z 轴旋转关节角度，然后沿 Z 轴平移连杆长度
        Rz = np.eye(4)
        c, s = np.cos(q[i]), np.sin(q[i])
        Rz[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
        T = T @ Rz
        transforms[name] = T.copy()
        T[:3, 3] += T[:2, 2].reshape(3)  # 沿局部 Z 步进
        # 近似：仅偏移 Z
        tz = np.eye(4)
        tz[2, 3] = link_lengths[i]
        T = T @ tz

    return transforms


def main() -> None:
    # 构建碰撞模型
    checker, link_names = build_collision_model()

    # 扫描配置网格并检查碰撞
    n_steps = 20
    collision_count = 0

    for i in range(n_steps):
        for j in range(n_steps):
            q = np.zeros(6)
            q[0] = np.pi * (2 * i / n_steps - 1)  # 底座扫描
            q[1] = np.pi / 2 * (2 * j / n_steps - 1)  # 肩部扫描

            transforms = make_link_transforms(q, link_names)

            # 自碰撞检测（默认忽略相邻连杆）
            sc: CollisionResult | None = checker.check_self_collision(
                transforms, ignore_adjacent=True
            )
            if sc is not None:
                collision_count += 1
                print(f"  自碰撞 q=[{q[0]:+.2f}, {q[1]:+.2f}, ...]  "
                      f"配对={sc.pair}  距离={sc.distance:.4f}")

            # 环境碰撞检测
            ec: CollisionResult | None = checker.check_environment_collision(
                transforms
            )
            if ec is not None:
                print(f"  环境碰撞 q=[{q[0]:+.2f}, {q[1]:+.2f}, ...]  "
                      f"连杆={ec.pair[0]}  距离={ec.distance:.4f}")

    total = n_steps * n_steps
    print(f"\n检查了 {total} 个配置，"
          f"发现 {collision_count} 次自碰撞。")


if __name__ == "__main__":
    main()
```

## 学习要点

- 使用碰撞基本几何体（胶囊体、球体、长方体）对机器人连杆建模
- 使用 `CollisionChecker` 构建碰撞世界
- 带相邻连杆过滤的自碰撞检测
- 环境（障碍物）碰撞检测
- 解释 `CollisionResult` 字段（`is_colliding`、`distance`、`contact_point`）

## 主要 API 说明

| 概念 | API |
|------|-----|
| 创建检测器 | `checker = CollisionChecker()` |
| 添加连杆几何体 | `checker.add_link_geometry(name, geometry)` |
| 添加障碍物 | `checker.add_obstacle(geometry)` |
| 自碰撞检测 | `checker.check_self_collision(link_transforms)` |
| 环境碰撞检测 | `checker.check_environment_collision(link_transforms)` |
| 结果字段 | `.is_colliding`、`.distance`、`.contact_point`、`.pair` |
