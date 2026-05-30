[English](../../tutorial/t7-cartesian-path-collision.md) | **中文**

# T7：笛卡尔直线轨迹与碰撞检测

**难度：** 高级 | **时间：** 30 分钟 | **模块：** `robot_ik.trajectory`、`robot_ik.collision`

本教程将笛卡尔直线轨迹规划与实时碰撞检测相结合。你将生成一个任务空间直线轨迹，使用 SLERP 姿态插值，构建带有障碍物的碰撞环境，并验证每个路点的自碰撞和环境碰撞。

## 你将学到

1. 使用 `cartesian_straight_line` 生成笛卡尔直线轨迹
2. 使用 `CollisionChecker`、`Sphere`、`Capsule` 和 `Box` 构建碰撞场景
3. 检查每个轨迹路点的碰撞
4. 将碰撞分类为自碰撞和环境碰撞
5. 绘制轨迹和碰撞结果

## 前置要求

```bash
pip install robot-ik numpy matplotlib
```

```python
import numpy as np
import matplotlib.pyplot as plt
```

---

## 步骤 1：生成笛卡尔直线轨迹

`cartesian_straight_line` 在笛卡尔空间中规划从起始关节配置到目标位姿（4x4 齐次变换）的直线运动。位置使用线性插值；姿态使用 SLERP（球面线性插值）。在每个步骤中，通过求解逆运动学获取对应的关节角度。

```python
from robot_ik import six_dof_articulated, cartesian_straight_line

robot = six_dof_articulated()

# 起始配置
q_start = np.array([0.0, np.pi/6, 0.0, 0.0, np.pi/4, 0.0])

# 验证起始末端执行器位姿
T_start = robot.forward_kinematics(q_start)
print("起始位姿（平移）：")
print(f"  x={T_start[0,3]:.3f}, y={T_start[1,3]:.3f}, z={T_start[2,3]:.3f}")

# 定义目标位姿（x 方向偏移 0.15m，z 方向偏移 0.1m）
T_target = T_start.copy()
T_target[0, 3] += 0.15  # 在 x 方向向前移动
T_target[2, 3] += 0.10  # 在 z 方向向上移动

print("目标位姿（平移）：")
print(f"  x={T_target[0,3]:.3f}, y={T_target[1,3]:.3f}, z={T_target[2,3]:.3f}")

# 生成轨迹（3 秒，50 Hz）
traj = cartesian_straight_line(
    robot=robot,
    q_start=q_start,
    target_pose=T_target,
    duration=3.0,
    dt=0.02,
)

print(f"\n轨迹已生成：")
print(f"  路点数：  {len(traj.time_points)}")
print(f"  持续时间：   {traj.duration:.1f} s")
print(f"  dt：         {traj.time_points[1] - traj.time_points[0]:.3f} s")
```

`TrajectoryResult` 包含：

| 字段 | 形状 | 描述 |
|------|------|------|
| `time_points` | `(N,)` | 时间戳（秒） |
| `joint_positions` | `(N, 6)` | 关节角度（弧度） |
| `joint_velocities` | `(N, 6)` | 关节速度 (rad/s) |
| `joint_accelerations` | `(N, 6)` | 关节加速度 (rad/s^2) |
| `duration` | 标量 | 总持续时间（秒） |

### 绘制关节轨迹

```python
fig, axes = plt.subplots(6, 1, figsize=(10, 8), sharex=True)
labels = [f"关节 {i}" for i in range(6)]

for i in range(6):
    axes[i].plot(traj.time_points, np.degrees(traj.joint_positions[:, i]))
    axes[i].set_ylabel(labels[i])
    axes[i].grid(True, alpha=0.3)

axes[-1].set_xlabel("时间 (s)")
fig.suptitle("笛卡尔直线：关节轨迹")
plt.tight_layout()
plt.savefig("t7_joint_trajectory.png", dpi=150)
plt.close()
```

### 绘制笛卡尔路径

```python
# 沿轨迹计算末端执行器位置
ee_positions = []
for i in range(len(traj.time_points)):
    T = robot.forward_kinematics(traj.joint_positions[i])
    ee_positions.append(T[:3, 3])

ee_positions = np.array(ee_positions)

fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")
ax.plot(ee_positions[:, 0], ee_positions[:, 1], ee_positions[:, 2],
        "b.-", linewidth=1.5, label="末端执行器路径")
ax.scatter(*ee_positions[0],  color="green", s=100, label="起始")
ax.scatter(*ee_positions[-1], color="red",   s=100, label="终止")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("末端执行器笛卡尔路径")
ax.legend()
plt.tight_layout()
plt.savefig("t7_cartesian_path.png", dpi=150)
plt.close()
```

## 步骤 2：设置碰撞检测器

创建 `CollisionChecker` 并为机器人连杆和环境障碍物添加碰撞几何体。

```python
from robot_ik import CollisionChecker, Sphere, Capsule, Box

checker = CollisionChecker()
```

### 添加连杆几何体

对于每个连杆，添加位于连杆局部坐标系中的碰撞基本几何体。
`add_link_geometry` 每个连杆接受多个几何体：

```python
# 在每个关节位置添加关节球体
for i in range(6):
    T_joint = robot.forward_kinematics(q_start, return_all=True)[1][i + 1]
    checker.add_link_geometry(
        f"joint{i}",
        Sphere(radius=0.06, pose=T_joint.copy()),
    )

# 在连续关节原点之间添加连杆胶囊体
transforms = robot.forward_kinematics(q_start, return_all=True)[1]
for i in range(5):
    p1 = transforms[i + 1][:3, 3]
    p2 = transforms[i + 2][:3, 3]
    checker.add_link_geometry(
        f"link{i}",
        Capsule(p1=p1, p2=p2, radius=0.04),
    )
```

### 添加环境障碍物

使用 `add_obstacle` 在世界坐标系中添加静态障碍物：

```python
# 桌面（机器人下方的一个长方体）
table = Box(
    size=np.array([0.8, 0.8, 0.05]),
    pose=np.array([
        [1, 0, 0, 0.0],
        [0, 1, 0, 0.0],
        [0, 0, 1, -0.35],
        [0, 0, 0, 1],
    ]),
)
checker.add_obstacle(table)

# 竖直柱体（工作空间中的障碍物）
pillar = Box(
    size=np.array([0.08, 0.08, 0.6]),
    pose=np.array([
        [1, 0, 0, 0.45],
        [0, 1, 0, 0.0],
        [0, 0, 1, -0.1],
        [0, 0, 0, 1],
    ]),
)
checker.add_obstacle(pillar)

print(f"带几何体的连杆：{list(checker.link_geometries.keys())}")
print(f"障碍物：           {len(checker.obstacles)}")
```

## 步骤 3：在每个路点检查碰撞

构建一个函数，对单个关节配置验证自碰撞和环境碰撞：

```python
def check_waypoint(q, checker, robot, ignore_adjacent=True):
    """检查单个配置的碰撞。

    返回：
        包含 'self_collision'、'env_collision'、'safe' 键的字典。
    """
    # 计算所有连杆变换
    _, transforms = robot.forward_kinematics(q, return_all=True)

    link_transforms = {}
    for i in range(6):
        link_transforms[f"joint{i}"] = transforms[i + 1]
    for i in range(5):
        link_transforms[f"link{i}"] = transforms[i + 1]

    # 自碰撞检测
    self_result = checker.check_self_collision(
        link_transforms, ignore_adjacent=ignore_adjacent
    )

    # 环境碰撞检测
    env_result = checker.check_environment_collision(link_transforms)

    return {
        "safe": self_result is None and env_result is None,
        "self_collision": self_result,
        "env_collision": env_result,
    }
```

现在扫描整个轨迹：

```python
safe_count = 0
self_col_count = 0
env_col_count = 0
collision_points = []

print("正在检查轨迹路点的碰撞...")
print(f"{'路点':>8}  {'时间':>6}  {'状态':>12}  {'详情'}")
print("-" * 60)

for i in range(len(traj.time_points)):
    q = traj.joint_positions[i]
    result = check_waypoint(q, checker, robot)

    if result["safe"]:
        safe_count += 1
        status = "安全"
        detail = ""
    elif result["self_collision"]:
        self_col_count += 1
        status = "自碰撞"
        pair = result["self_collision"].pair
        detail = f"{pair[0]} <-> {pair[1]}"
        collision_points.append(i)
    elif result["env_collision"]:
        env_col_count += 1
        status = "环境碰撞"
        detail = f"{result['env_collision'].pair[0]} <-> 障碍物"
        collision_points.append(i)
    else:
        safe_count += 1
        status = "安全"
        detail = ""

    # 每 25 个路点或碰撞时打印
    if i % 25 == 0 or not result["safe"]:
        t = traj.time_points[i]
        print(f"{i:>8d}  {t:>6.2f}s  {status:>12}  {detail}")

print()
print(f"摘要：{safe_count} 个安全，"
      f"{self_col_count} 次自碰撞，"
      f"{env_col_count} 次环境碰撞")
print(f"总路点数：{len(traj.time_points)}")
```

## 步骤 4：可视化碰撞结果

绘制突出碰撞点的笛卡尔路径：

```python
fig = plt.figure(figsize=(10, 6))

# 位置子图
ax1 = fig.add_subplot(121, projection="3d")
safe_mask = np.ones(len(ee_positions), dtype=bool)
for idx in collision_points:
    safe_mask[idx] = False

# 安全部分用蓝色
ax1.plot(ee_positions[safe_mask, 0],
         ee_positions[safe_mask, 1],
         ee_positions[safe_mask, 2],
         "b-", linewidth=1.5, label="安全")

# 碰撞点用红色
if len(collision_points) > 0:
    col_pos = ee_positions[collision_points]
    ax1.scatter(col_pos[:, 0], col_pos[:, 1], col_pos[:, 2],
                color="red", s=50, label="碰撞", zorder=5)

# 障碍物
pillar_pos = pillar.pose[:3, 3]
ax1.scatter(*pillar_pos, color="orange", s=200, marker="s",
            label="柱体")

ax1.set_xlabel("X (m)")
ax1.set_ylabel("Y (m)")
ax1.set_zlabel("Z (m)")
ax1.set_title("带碰撞检测的路径")
ax1.legend()

# 关节空间子图
ax2 = fig.add_subplot(122)
for i in range(6):
    color = "blue" if all(idx not in collision_points for idx in range(len(traj.time_points)))
    ax2.plot(traj.time_points,
             np.degrees(traj.joint_positions[:, i]),
             label=f"J{i}")

for idx in collision_points:
    ax2.axvline(traj.time_points[idx], color="red", alpha=0.3, linewidth=0.5)

ax2.set_xlabel("时间 (s)")
ax2.set_ylabel("关节角度 (度)")
ax2.set_title("关节轨迹（红色 = 碰撞）")
ax2.legend(fontsize=8)
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("t7_collision_results.png", dpi=150)
plt.close()
```

## 步骤 5：碰撞裕度和安全阈值

`_check_geometry_collision` 方法使用 `collision_threshold` 参数（默认 0.0）。你可以使用正阈值在实际接触之前触发警告 —— 用于安全裕度：

```python
# 使用 2cm 安全裕度检查
#（这需要直接调用 _check_geometry_collision 或
# 在子类中修改 collision_threshold）

# 简单方法：通过检查 CollisionResult 上的 distance 字段
# 在不使用阈值的情况下检查距离
def min_clearance(q, checker, robot):
    """计算到任何碰撞的最小间隙。"""
    _, transforms = robot.forward_kinematics(q, return_all=True)
    link_transforms = {}
    for i in range(6):
        link_transforms[f"joint{i}"] = transforms[i + 1]
    for i in range(5):
        link_transforms[f"link{i}"] = transforms[i + 1]

    min_dist = float("inf")
    self_result = checker.check_self_collision(link_transforms)
    env_result = checker.check_environment_collision(link_transforms)

    if self_result is not None:
        min_dist = min(min_dist, self_result.distance)
    if env_result is not None:
        min_dist = min(min_dist, env_result.distance)

    return min_dist

# 检查几个路点的间隙
print("间隙分析：")
safety_margin = 0.02  # 2 cm
for idx in [0, len(traj.time_points)//4, len(traj.time_points)//2,
            3*len(traj.time_points)//4, -1]:
    q = traj.joint_positions[idx]
    d = min_clearance(q, checker, robot)
    t = traj.time_points[idx]
    status = "正常" if d > safety_margin else "警告"
    print(f"  t={t:.2f}s: 间隙={d:.4f}m  [{status}]")
```

## 步骤 6：综合应用 —— 安全路径规划

将轨迹生成、碰撞检测和重新规划组合在一个完整的工作流中：

```python
from robot_ik import cartesian_straight_line

def plan_safe_cartesian_path(robot, checker, q_start, T_target,
                              duration=3.0, dt=0.02):
    """生成笛卡尔路径并验证碰撞。

    返回：
        (轨迹, 碰撞索引) 元组，如果不安全则返回 (None, [])。
    """
    traj = cartesian_straight_line(robot, q_start, T_target, duration, dt)

    collision_indices = []
    for i in range(len(traj.time_points)):
        result = check_waypoint(traj.joint_positions[i], checker, robot)
        if not result["safe"]:
            collision_indices.append(i)

    if len(collision_indices) > 0:
        print(f"警告：{len(collision_indices)}/{len(traj.time_points)} "
              f"个路点存在碰撞")
    else:
        print(f"路径已验证：全部 {len(traj.time_points)} 个路点安全")

    return traj, collision_indices


# 规划路径
traj, collisions = plan_safe_cartesian_path(
    robot, checker, q_start, T_target, duration=3.0
)

# 如果发现碰撞，你可以：
# 1. 使用不同的目标位姿重新规划
# 2. 使用 RRT* 路径规划（T4）找到无碰撞路径
# 3. 添加中间路点以避开障碍物
```

## 几何体参考

碰撞模块提供三种基本类型：

| 类型 | 构造函数 | 参数 |
|------|----------|------|
| `Sphere` | `Sphere(radius, pose)` | `radius`（米），`pose` 为 4x4 |
| `Capsule` | `Capsule(p1, p2, radius, pose)` | 线段端点，`radius`（米） |
| `Box` | `Box(size, pose)` | `size` 为 (x,y,z)，`pose` 为 4x4 |

支持的成对距离函数：

- `distance_sphere_to_sphere`
- `distance_sphere_to_capsule`
- `distance_capsule_to_capsule`
- `distance_sphere_to_box`
- `distance_box_to_box`
- `distance_point_to_sphere`
- `distance_point_to_box`

## 关键要点

| 函数 | 用途 |
|------|------|
| `cartesian_straight_line(robot, q_start, target_pose, duration)` | 规划笛卡尔直线（使用 SLERP） |
| `CollisionChecker()` | 创建碰撞检测环境 |
| `checker.add_link_geometry(name, geom)` | 将碰撞形状附加到连杆 |
| `checker.add_obstacle(geom)` | 添加环境障碍物 |
| `checker.check_self_collision(transforms)` | 检查连杆间碰撞 |
| `checker.check_environment_collision(transforms)` | 检查连杆-障碍物碰撞 |

## 下一步

- **T4：** 使用 RRT* 在障碍物周围进行无碰撞路径规划
- **T5：** 添加动力学分析以验证沿轨迹的力矩限制
- **T6：** 使用 Meshcat 在 3D 中可视化无碰撞路径
