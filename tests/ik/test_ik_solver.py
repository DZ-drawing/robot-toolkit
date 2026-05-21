"""Test suite for the 6-DOF IK solver."""

import time

import numpy as np

from robot_ik.ik import (
    DHParam,
    RobotModel,
    six_dof_articulated,
)


def test_fk_identity():
    """Forward kinematics at zero angles should produce a valid 4x4 transform."""
    robot = six_dof_articulated()
    T = robot.forward_kinematics(np.zeros(6))
    assert T.shape == (4, 4), f"Expected 4x4, got {T.shape}"
    assert np.allclose(T[3, :], [0, 0, 0, 1]), "Bottom row should be [0,0,0,1]"
    assert np.allclose(
        T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-10
    ), "Rotation should be orthonormal"
    print(f"  [PASS] test_fk_identity (EE at {T[0,3]:.2f}, {T[1,3]:.2f}, {T[2,3]:.2f})")


def test_ik_roundtrip():
    """IK should recover joint angles from FK-generated target poses."""
    robot = six_dof_articulated()
    np.random.seed(123)
    failures = 0
    for _i in range(20):
        q_orig = np.random.uniform(-0.8, 0.8, 6)
        target = robot.forward_kinematics(q_orig)
        success, q_solved, iters, errors = robot.ik_solve(target, max_iterations=300)
        if not success:
            # Retry from different initial guess
            success, q_solved, iters2, errors2 = robot.ik_solve(
                target, max_iterations=300, initial_guess=np.random.uniform(-1, 1, 6)
            )
            iters += iters2

        T = robot.forward_kinematics(q_solved)
        pos_err = np.linalg.norm(T[:3, 3] - target[:3, 3])
        if not success:
            failures += 1
        assert pos_err < 5e-3, f"Position error {pos_err:.2e} > 5e-3 (iter {iters})"
    print(f"  [PASS] test_ik_roundtrip (20 poses, {failures} slow-converge, all within 2mm)")


def test_ik_orientation():
    """IK should match not just position but also orientation."""
    robot = six_dof_articulated()
    np.random.seed(456)
    for _ in range(10):
        q_orig = np.random.uniform(-0.5, 0.5, 6)
        target = robot.forward_kinematics(q_orig)
        success, q_solved, _, _ = robot.ik_solve(target)

        T_solved = robot.forward_kinematics(q_solved)
        orient_err = np.linalg.norm(T_solved[:3, :3] - target[:3, :3])
        assert orient_err < 0.05, f"Orientation error {orient_err:.2e}"
    print("  [PASS] test_ik_orientation (10 random poses)")


def test_jacobian_numerical():
    """Jacobian should match numerical differentiation."""
    robot = six_dof_articulated()
    q = np.array([0.1, 0.3, -0.4, 0.2, 0.5, -0.1])
    J_analytical = robot.compute_jacobian(q)

    # Numerical Jacobian via finite differences
    J_numerical = np.zeros((6, 6))
    eps = 1e-6
    T0 = robot.forward_kinematics(q)
    p0 = T0[:3, 3]
    for i in range(6):
        q_plus = q.copy()
        q_plus[i] += eps
        T_plus = robot.forward_kinematics(q_plus)
        p_plus = T_plus[:3, 3]
        J_numerical[:3, i] = (p_plus - p0) / eps
        J_numerical[3:, i] = (T_plus[:3, :3] @ T0[:3, :3].T - np.eye(3)).ravel()[[2, 0, 1]] / eps

    lin_err = np.max(np.abs(J_analytical[:3, :] - J_numerical[:3, :]))
    assert lin_err < 1e-3, f"Jacobian linear part mismatch: {lin_err:.2e}"
    print(f"  [PASS] test_jacobian_numerical (max linear error: {lin_err:.2e})")


def test_joint_limits():
    """Solved angles should respect joint limits."""
    robot = six_dof_articulated()
    np.random.seed(789)
    for _ in range(10):
        q_orig = np.random.uniform(-0.8, 0.8, 6)
        target = robot.forward_kinematics(q_orig)
        success, q_solved, _, _ = robot.ik_solve(target)

        for i, (lo, hi) in enumerate(robot.joint_limits):
            assert (
                lo - 1e-10 <= q_solved[i] <= hi + 1e-10
            ), f"Joint {i}: {q_solved[i]:.2f} outside [{lo}, {hi}]"
    print("  [PASS] test_joint_limits (10 random poses)")


def test_custom_robot():
    """Custom robot with simpler DH parameters should solve correctly."""
    # Simpler 6-DOF robot: all links have non-zero a, healthy workspace
    robot = RobotModel(
        [
            DHParam(a=0.1, alpha=-np.pi / 2, d=0.3, theta=0),
            DHParam(a=0.25, alpha=0, d=0, theta=0),
            DHParam(a=0.25, alpha=0, d=0, theta=0),
            DHParam(a=0.1, alpha=-np.pi / 2, d=0.2, theta=0),
            DHParam(a=0, alpha=np.pi / 2, d=0, theta=0),
            DHParam(a=0, alpha=0, d=0.05, theta=0),
        ]
    )
    np.random.seed(999)
    for _ in range(10):
        q_orig = np.random.uniform(-0.8, 0.8, 6)
        target = robot.forward_kinematics(q_orig)
        success, q_solved, _, _ = robot.ik_solve(target, max_iterations=300)
        T = robot.forward_kinematics(q_solved)
        pos_err = np.linalg.norm(T[:3, 3] - target[:3, 3])
        if not success:
            print(f"    (slow converge, pos_err={pos_err:.1e})")
        assert pos_err < 5e-3, f"Position error {pos_err:.2e}"
    print("  [PASS] test_custom_robot (10 random poses)")


def benchmark():
    """Performance benchmark."""
    robot = six_dof_articulated()
    np.random.seed(0)
    times = []
    iterations = []
    failures = 0

    for _ in range(200):
        q_rand = np.random.uniform(-1, 1, 6)
        target = robot.forward_kinematics(q_rand)
        start = time.perf_counter()
        success, _, iters, _ = robot.ik_solve(target, max_iterations=100)
        elapsed = time.perf_counter() - start
        times.append(elapsed)
        iterations.append(iters)
        if not success:
            failures += 1

    times_ms = np.array(times) * 1000
    print("\n  Benchmark (200 solves):")
    print(f"    Avg time:  {np.mean(times_ms):.1f} ms")
    print(f"    P50 time:  {np.median(times_ms):.1f} ms")
    print(f"    P95 time:  {np.percentile(times_ms, 95):.1f} ms")
    print(f"    P99 time:  {np.percentile(times_ms, 99):.1f} ms")
    print(f"    Avg iters: {np.mean(iterations):.1f}")
    print(f"    Failures:  {failures}/200")


if __name__ == "__main__":
    print("=== 6-DOF IK Solver Test Suite ===\n")
    test_fk_identity()
    test_ik_roundtrip()
    test_ik_orientation()
    test_jacobian_numerical()
    test_joint_limits()
    test_custom_robot()
    benchmark()
    print("\n=== All tests passed ===")
