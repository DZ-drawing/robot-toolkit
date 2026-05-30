[English](../../tutorial/t4-path-planning-rrt.md) | **中文**

# 教程 4：使用 RRT* 进行无碰撞路径规划

**目标**：使用 RRT*（快速探索随机树星形）算法为 6 自由度机械臂规划无碰撞的关节空间路径。

**演示功能**：
- `RRTStar` 规划器构建
- `CollisionChecker` 集成用于自由空间验证
- 从起始到目标配置的路径规划
- 顺序多臂规划（臂1 就位后再规划臂2）

---

## 工作原理

RRT* 从起始配置构建一棵无碰撞配置树。在每次迭代中，它采样一个随机关节配置，找到最近的树节点，以有界步长向采样点推进，如果边是无碰撞的则插入新节点。近节点重连逐步改善路径代价。迭代完成后，提取路径并进行捷径平滑。

```python
"""教程 4：使用 RRT* 进行无碰撞路径规划。

在共享工作空间中为两个机械臂规划路径。臂 1 先规划；
然后臂 2 在规划时将臂 1 的最终配置视为障碍物。

运行方式：python -m docs.tutorial.t4_path_planning_rrt
"""

import numpy as np
from robot_ik import six_dof_articulated
from robot_ik.collision import CollisionChecker, Capsule, Box
from robot_ik.path_planning import RRTStar


def build_checker_with_obstacles() -> CollisionChecker:
    """构建一个带有静态长方体障碍物的 CollisionChecker。"""
    checker = CollisionChecker()

    # 将每个连杆建模为沿局部 Z 轴的胶囊体
    link_lengths = [0.0, 0.3, 0.5, 0.1, 0.4, 0.1]
    for i in range(6):
        checker.add_link_geometry(
            f"link{i}",
            Capsule(
                p1=np.zeros(3),
                p2=np.array([0.0, 0.0, link_lengths[i]]),
                radius=0.05,
            ),
        )

    # 在两个臂之间添加静态长方体障碍物
    obs_pose = np.eye(4)
    obs_pose[:3, 3] = np.array([0.75, 0.0, 0.3])  # 位于两个臂之间
    checker.add_obstacle(
        Box(size=np.array([0.2, 0.4, 0.2]), pose=obs_pose)
    )

    return checker


def main() -> None:
    # -- 设置 ------------------------------------------------------------
    robot = six_dof_articulated()
    checker = build_checker_with_obstacles()

    # 关节限位作为 (6, 2) 数组供 RRT* 规划器使用
    joint_limits = np.array([
        [-np.pi, np.pi],
        [-np.pi / 2, np.pi / 2],
        [-3 * np.pi / 4, 3 * np.pi / 4],
        [-np.pi, np.pi],
        [-np.pi / 2, np.pi / 2],
        [-np.pi, np.pi],
    ])

    # -- 规划臂 1 -------------------------------------------------------
    planner1 = RRTStar(
        robot=robot,
        collision_checker=checker,
        joint_limits=joint_limits,
        step_size=0.15,
        max_iterations=500,
        goal_threshold=0.2,
        goal_sample_rate=0.15,
    )

    start1 = np.zeros(6)
    goal1 = np.array([np.pi / 2, 0.0, np.pi / 4, 0.0, np.pi / 2, 0.0])

    result1 = planner1.plan(start1, goal1)
    if result1.success:
        print(f"臂 1 路径已找到：{result1.path.shape[0]} 个路点，"
              f"代价={result1.cost:.2f}，时间={result1.planning_time:.3f}s，"
              f"节点数={result1.nodes_explored}")
    else:
        print(f"臂 1 规划失败：{result1.message}")
        return

    # -- 规划臂 2（将臂1的目标视为障碍物）--------------------
    # 将臂1的最终连杆位置表示为静态胶囊体障碍物
    # 以便臂2避免与臂1的停放配置碰撞
    T, all_transforms = robot.forward_kinematics(goal1, return_all=True)
    for i in range(1, 7):  # 跳过基座（索引 0）
        T_link = all_transforms[i].copy()
        checker.add_obstacle(
            Capsule(
                p1=np.zeros(3),
                p2=np.array([0.0, 0.0, 0.15]),
                radius=0.06,
                pose=T_link,
            )
        )

    # 将臂2的基座沿 X 轴偏移 1.5 m（起始/目标镜像）
    arm2_offset = np.array([1.5, 0.0, 0.0])

    planner2 = RRTStar(
        robot=robot,
        collision_checker=checker,
        joint_limits=joint_limits,
        step_size=0.15,
        max_iterations=500,
        goal_threshold=0.2,
        goal_sample_rate=0.15,
    )

    start2 = np.zeros(6)
    goal2 = np.array([-np.pi / 2, 0.0, -np.pi / 4, 0.0, np.pi / 2, 0.0])

    result2 = planner2.plan(start2, goal2)
    if result2.success:
        print(f"臂 2 路径已找到：{result2.path.shape[0]} 个路点，"
              f"代价={result2.cost:.2f}，时间={result2.planning_time:.3f}s，"
              f"节点数={result2.nodes_explored}")
    else:
        print(f"臂 2 规划失败：{result2.message}")

    # -- 摘要 ----------------------------------------------------------
    print(f"\n摘要：")
    print(f"  臂 1：起始 {start1} -> 目标 {goal1}")
    print(f"  臂 2：起始 {start2} -> 目标 {goal2}")
    print(f"  路径路点总数："
          f"{result1.path.shape[0]} + {result2.path.shape[0]} = "
          f"{result1.path.shape[0] + result2.path.shape[0]}")


if __name__ == "__main__":
    main()
```

## 学习要点

- 基于采样的路径规划基础（随机采样、树生长）
- 将 `CollisionChecker` 集成到规划循环中
- 调整 RRT* 参数（`step_size`、`max_iterations`、`goal_threshold`）
- 通过障碍物注入实现顺序多机器人规划
- 解释 `PathPlanningResult` 字段

## 主要 API 说明

| 概念 | API |
|------|-----|
| 创建规划器 | `RRTStar(robot, collision_checker, joint_limits, step_size=, max_iterations=, goal_threshold=)` |
| 规划 | `planner.plan(start, goal) -> PathPlanningResult` |
| 结果字段 | `.success`、`.path`（Nx6 ndarray）、`.cost`、`.planning_time`、`.nodes_explored`、`.message` |
| 便捷函数 | `plan_path_rrt_star(robot, checker, start, goal, joint_limits, **kwargs)` |

## 提示

- 对于更复杂的环境，增加 `max_iterations`（例如 2000+）。
- 较小的 `step_size` 提供更精细的分辨率，但规划速度更慢。
- `goal_sample_rate`（默认 0.1）使采样偏向目标；对于简单问题可增加，困难问题可减小。
- 返回的 `.path` 已经由规划器进行了捷径平滑。
