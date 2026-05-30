[English](t4-path-planning-rrt.md) | [中文](../zh/tutorial/t4-path-planning-rrt.md)
# Tutorial 4: Collision-Free Path Planning with RRT*

**Objective**: Plan collision-free joint-space paths for a 6-DOF manipulator
using the RRT* (Rapidly-exploring Random Tree Star) algorithm.

**Features demonstrated**:
- `RRTStar` planner construction
- `CollisionChecker` integration for free-space validation
- Path planning from start to goal configurations
- Sequential multi-arm planning (plan arm2 after arm1 is in place)

---

## How it works

RRT* builds a tree of collision-free configurations rooted at the start
configuration. At each iteration it samples a random joint configuration,
finds the nearest tree node, steers toward the sample by a bounded step,
and (if the edge is collision-free) inserts the new node. Near-node rewiring
progressively improves path cost. After the iterations complete, the path is
extracted and shortcut-smoothed.

```python
"""Tutorial 4: Collision-Free Path Planning with RRT*.

Plan paths for two arms in a shared workspace. Arm 1 plans first; then
arm 2 plans while treating arm 1's final configuration as an obstacle.

Run with:  python -m docs.tutorial.t4_path_planning_rrt
"""

import numpy as np
from robot_ik import six_dof_articulated
from robot_ik.collision import CollisionChecker, Capsule, Box
from robot_ik.path_planning import RRTStar


def build_checker_with_obstacles() -> CollisionChecker:
    """Build a CollisionChecker with a static box obstacle in the workspace."""
    checker = CollisionChecker()

    # Model each link as a capsule along local Z
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

    # Add a static box obstacle between the arms
    obs_pose = np.eye(4)
    obs_pose[:3, 3] = np.array([0.75, 0.0, 0.3])  # centred between arms
    checker.add_obstacle(
        Box(size=np.array([0.2, 0.4, 0.2]), pose=obs_pose)
    )

    return checker


def main() -> None:
    # -- Setup ------------------------------------------------------------
    robot = six_dof_articulated()
    checker = build_checker_with_obstacles()

    # Joint limits as (6, 2) array for the RRT* planner
    joint_limits = np.array([
        [-np.pi, np.pi],
        [-np.pi / 2, np.pi / 2],
        [-3 * np.pi / 4, 3 * np.pi / 4],
        [-np.pi, np.pi],
        [-np.pi / 2, np.pi / 2],
        [-np.pi, np.pi],
    ])

    # -- Plan arm 1 -------------------------------------------------------
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
        print(f"Arm 1 path found: {result1.path.shape[0]} waypoints, "
              f"cost={result1.cost:.2f}, time={result1.planning_time:.3f}s, "
              f"nodes={result1.nodes_explored}")
    else:
        print(f"Arm 1 planning failed: {result1.message}")
        return

    # -- Plan arm 2 (treating arm1's goal as obstacle) --------------------
    # Represent arm1's final link positions as static capsule obstacles
    # so arm2 avoids colliding with arm1's parked configuration.
    T, all_transforms = robot.forward_kinematics(goal1, return_all=True)
    for i in range(1, 7):  # skip base (index 0)
        T_link = all_transforms[i].copy()
        checker.add_obstacle(
            Capsule(
                p1=np.zeros(3),
                p2=np.array([0.0, 0.0, 0.15]),
                radius=0.06,
                pose=T_link,
            )
        )

    # Offset arm2's base by 1.5 m along X (mirrored start/goal)
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
        print(f"Arm 2 path found: {result2.path.shape[0]} waypoints, "
              f"cost={result2.cost:.2f}, time={result2.planning_time:.3f}s, "
              f"nodes={result2.nodes_explored}")
    else:
        print(f"Arm 2 planning failed: {result2.message}")

    # -- Summary ----------------------------------------------------------
    print(f"\nSummary:")
    print(f"  Arm 1: start {start1} -> goal {goal1}")
    print(f"  Arm 2: start {start2} -> goal {goal2}")
    print(f"  Total path waypoints: "
          f"{result1.path.shape[0]} + {result2.path.shape[0]} = "
          f"{result1.path.shape[0] + result2.path.shape[0]}")


if __name__ == "__main__":
    main()
```

## Learning outcomes

- Sampling-based path planning fundamentals (random sampling, tree growth)
- Integrating `CollisionChecker` into the planning loop
- Tuning RRT* parameters (`step_size`, `max_iterations`, `goal_threshold`)
- Sequential multi-robot planning with obstacle injection
- Interpreting `PathPlanningResult` fields

## Key API notes

| Concept | API |
|---|---|
| Create planner | `RRTStar(robot, collision_checker, joint_limits, step_size=, max_iterations=, goal_threshold=)` |
| Plan | `planner.plan(start, goal) -> PathPlanningResult` |
| Result fields | `.success`, `.path` (Nx6 ndarray), `.cost`, `.planning_time`, `.nodes_explored`, `.message` |
| Convenience func | `plan_path_rrt_star(robot, checker, start, goal, joint_limits, **kwargs)` |

## Tips

- Increase `max_iterations` (e.g., 2000+) for more complex environments.
- A smaller `step_size` gives finer resolution but slower planning.
- `goal_sample_rate` (default 0.1) biases sampling toward the goal; increase
  for easier problems, decrease for harder ones.
- The returned `.path` is already shortcut-smoothed by the planner.
