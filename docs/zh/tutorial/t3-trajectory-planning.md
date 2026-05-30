[English](../../tutorial/t3-trajectory-planning.md) | **中文**

# 教程 3：协调双臂轨迹规划

**目标**：为两个协同移动物体的机械臂生成时间同步轨迹，使用基于路点的插值和 S 曲线速度曲线。

**演示功能**：
- `joint_cubic_interpolation` 用于平滑的单段运动
- `s_curve_profile` 用于加加速度受限的轨迹
- `waypoint_trajectory` 用于多路点路径
- 双臂时间协调

---

## 工作原理

每个机械臂遵循自己的关节空间路点序列。我们生成具有相同时间向量的轨迹，使两个臂同时启动和停止 —— 这是它们共同搬运物体时的要求。

```python
"""教程 3：协调双臂轨迹规划。

为两个机械臂对共享物体执行抓取-放置操作生成同步的关节空间轨迹。

运行方式：python -m docs.tutorial.t3_trajectory_planning
"""

import numpy as np
from robot_ik.trajectory import (
    TrajectoryResult,
    joint_cubic_interpolation,
    joint_quintic_interpolation,
    s_curve_profile,
    waypoint_trajectory,
    trapezoidal_velocity_profile,
)


def print_summary(label: str, traj: TrajectoryResult) -> None:
    """打印轨迹统计信息。"""
    q = traj.joint_positions
    v = traj.joint_velocities
    print(f"\n{label}：")
    print(f"  持续时间       : {traj.duration:.2f} s  "
          f"({len(traj.time_points)} 个采样点)")
    print(f"  关节范围    : [{q.min():.3f}, {q.max():.3f}] rad")
    print(f"  最大 |速度| : {np.abs(v).max():.3f} rad/s")
    print(f"  起始位置 : {q[0]}")
    print(f"  终止位置   : {q[-1]}")


def main() -> None:
    dof = 6
    dt = 0.01  # 100 Hz

    # ------------------------------------------------------------------
    # 1. 三次插值（平滑，边界处速度为零）
    # ------------------------------------------------------------------
    q_start = np.zeros(dof)
    q_end = np.array([0.5, -0.3, 0.4, 0.0, 0.2, 0.0])

    traj_cubic = joint_cubic_interpolation(q_start, q_end, duration=2.0, dt=dt)
    print_summary("三次插值", traj_cubic)

    # ------------------------------------------------------------------
    # 2. S 曲线速度曲线（加加速度受限，C2 连续）
    # ------------------------------------------------------------------
    v_max = np.full(dof, 2.0)   # rad/s
    a_max = np.full(dof, 4.0)   # rad/s^2
    j_max = np.full(dof, 20.0)  # rad/s^3

    traj_scurve = s_curve_profile(
        q_start, q_end, duration=2.0, v_max=v_max, a_max=a_max, j_max=j_max, dt=dt
    )
    print_summary("S 曲线速度曲线", traj_scurve)

    # ------------------------------------------------------------------
    # 3. 多路点轨迹（每对之间使用五次插值）
    # ------------------------------------------------------------------
    waypoints = [
        np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),   # 归零位
        np.array([0.3, 0.2, -0.5, 0.0, 0.5, 0.0]),   # 接近
        np.array([0.3, 0.2, -0.3, 0.0, 0.5, 0.0]),   # 抓取
        np.array([0.3, -0.2, -0.3, 0.0, 0.5, 0.0]),  # 提升
        np.array([0.5, -0.3, 0.0, 0.0, 0.2, 0.0]),   # 放置
    ]
    times = [0.0, 1.0, 2.0, 3.5, 5.0]  # 每个路点的到达时间

    traj_wp = waypoint_trajectory(waypoints, times, method="quintic", dt=dt)
    print_summary("多路点（五次）", traj_wp)

    # ------------------------------------------------------------------
    # 4. 协调双臂轨迹（相同时间向量）
    # ------------------------------------------------------------------
    # 臂 1 在位置 A 抓取，臂 2 抓取另一侧（Y 方向偏移 0.6 m）
    arm1_waypoints = [
        np.array([0.5, 0.0, 0.3, 0, 0, 0]),
        np.array([0.5, 0.3, 0.3, 0, 0, 0]),
        np.array([0.5, -0.3, 0.3, 0, 0, 0]),
    ]
    arm2_waypoints = [
        wp + np.array([0.0, 0.6, 0.0, 0.0, 0.0, 0.0])
        for wp in arm1_waypoints
    ]
    times_dual = [0.0, 2.0, 4.0]

    traj_arm1 = waypoint_trajectory(arm1_waypoints, times_dual, method="cubic", dt=dt)
    traj_arm2 = waypoint_trajectory(arm2_waypoints, times_dual, method="cubic", dt=dt)

    # 验证时间同步
    assert len(traj_arm1.time_points) == len(traj_arm2.time_points), \
        "轨迹必须具有相同数量的采样点"
    max_time_error = np.max(np.abs(traj_arm1.time_points - traj_arm2.time_points))
    assert max_time_error < 1e-9, f"时间不匹配：{max_time_error}"

    print_summary("双臂 臂1", traj_arm1)
    print_summary("双臂 臂2", traj_arm2)
    print(f"\n时间同步正常  (最大 dt 误差 = {max_time_error:.1e} s)")


if __name__ == "__main__":
    main()
```

## 学习要点

- 两个关节配置之间的三次/五次多项式插值
- 带加加速度限制的 S 曲线速度曲线
- 带到达时间指定的多路点轨迹生成
- 通过共享时间向量实现双臂时间协调
- 解释 `TrajectoryResult` 字段

## 主要 API 说明

| 函数 | 签名 | 说明 |
|------|------|------|
| `joint_cubic_interpolation` | `(q_start, q_end, duration, dt)` | 端点处速度为零 |
| `joint_quintic_interpolation` | `(q_start, q_end, duration, v_start, v_end, ...)` | 完整边界控制 |
| `s_curve_profile` | `(q_start, q_end, duration, v_max, a_max, j_max, dt)` | 加加速度受限 (C2) |
| `trapezoidal_velocity_profile` | `(q_start, q_end, duration, v_max, a_max, dt)` | 三段梯形 |
| `waypoint_trajectory` | `(waypoints, times, method, blend_radius, dt)` | `method`："linear"/"cubic"/"quintic" |

`TrajectoryResult` 字段：`.time_points`、`.joint_positions`、`.joint_velocities`、
`.joint_accelerations`、`.duration`。
