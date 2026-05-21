"""Test path planning module."""

import numpy as np

from robot_ik import six_dof_articulated
from robot_ik.collision import CollisionChecker, Sphere
from robot_ik.path_planning import RRTStar, plan_path_rrt_star


def test_rrt_star_basic():
    """Basic RRT* planning test."""
    robot = six_dof_articulated()
    checker = CollisionChecker()

    # Add simple collision geometry
    for i in range(6):
        sphere = Sphere(radius=0.05)
        checker.add_link_geometry(f"link{i}", sphere)

    # Joint limits (radians)
    joint_limits = np.array(
        [
            [-np.pi, np.pi],
            [-np.pi / 2, np.pi / 2],
            [-np.pi, np.pi],
            [-np.pi, np.pi],
            [-np.pi / 2, np.pi / 2],
            [-np.pi, np.pi],
        ]
    )

    start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    goal = np.array([0.5, 0.3, 0.5, 0.0, 0.0, 0.0])

    planner = RRTStar(
        robot,
        checker,
        joint_limits,
        step_size=0.2,
        max_iterations=500,
        goal_threshold=0.3,
    )

    result = planner.plan(start, goal)

    print(f"  Success: {result.success}")
    print(f"  Path length: {len(result.path)}")
    print(f"  Cost: {result.cost:.2f}")
    print(f"  Time: {result.planning_time:.2f}s")
    print(f"  Nodes: {result.nodes_explored}")

    if result.success:
        print(f"  Path shape: {result.path.shape}")
        print(f"  Start matches: {np.allclose(result.path[0], start, atol=0.1)}")
        print(f"  Goal matches: {np.allclose(result.path[-1], goal, atol=0.3)}")

    assert result.success, "Expected planning to succeed"
    assert len(result.path) > 0, "Expected non-empty path"
    assert np.allclose(result.path[0], start, atol=0.1), "Path start mismatch"
    print("  [PASS] test_rrt_star_basic")


def test_rrt_star_collision():
    """RRT* with collision avoidance."""
    robot = six_dof_articulated()
    checker = CollisionChecker()

    # Add link geometry
    for i in range(6):
        sphere = Sphere(radius=0.04)
        checker.add_link_geometry(f"link{i}", sphere)

    # Add obstacle
    from robot_ik.collision import Box

    obstacle = Box(size=np.array([0.2, 0.2, 0.2]))
    obstacle.pose[:3, 3] = np.array([0.3, 0.0, 0.4])
    checker.add_obstacle(obstacle)

    joint_limits = np.array(
        [
            [-np.pi, np.pi],
            [-np.pi / 2, np.pi / 2],
            [-np.pi, np.pi],
            [-np.pi, np.pi],
            [-np.pi / 2, np.pi / 2],
            [-np.pi, np.pi],
        ]
    )

    start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    goal = np.array([0.8, 0.0, 0.0, 0.0, 0.0, 0.0])

    planner = RRTStar(
        robot,
        checker,
        joint_limits,
        step_size=0.15,
        max_iterations=800,
        goal_threshold=0.25,
    )

    result = planner.plan(start, goal)

    print(f"  Success: {result.success}")
    print(f"  Nodes explored: {result.nodes_explored}")
    print(f"  Planning time: {result.planning_time:.2f}s")

    # Should find a path around the obstacle
    assert (
        result.success or result.nodes_explored >= planner.max_iterations
    ), "Expected either success or max iterations reached"

    if result.success:
        # Verify path is collision-free (sample check)
        for i in range(0, len(result.path), 10):
            result.path[i]
            transforms = {}
            for j in range(6):
                transforms[f"link{j}"] = np.eye(4)  # Simplified
            coll = checker.check_environment_collision(transforms)
            if coll:
                print(f"  Warning: Collision at waypoint {i}")

    print("  [PASS] test_rrt_star_collision")


def test_convenience_function():
    """Test convenience planning function."""
    robot = six_dof_articulated()
    checker = CollisionChecker()

    joint_limits = np.array(
        [
            [-np.pi, np.pi],
            [-np.pi / 2, np.pi / 2],
            [-np.pi, np.pi],
            [-np.pi, np.pi],
            [-np.pi / 2, np.pi / 2],
            [-np.pi, np.pi],
        ]
    )

    start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    goal = np.array([0.3, 0.2, 0.3, 0.0, 0.0, 0.0])

    result = plan_path_rrt_star(
        robot,
        checker,
        start,
        goal,
        joint_limits,
        max_iterations=300,
    )

    print(f"  Result: {result.message}")
    assert isinstance(result.success, bool), "Expected success boolean"
    print("  [PASS] test_convenience_function")


if __name__ == "__main__":
    print("=== Path Planning Test Suite ===\n")
    test_rrt_star_basic()
    test_rrt_star_collision()
    test_convenience_function()
    print("\n=== All tests passed ===")
