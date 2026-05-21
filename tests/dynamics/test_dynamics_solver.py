"""Test suite for robot-dyn rigid body dynamics solver."""

import time

import numpy as np

from robot_ik import DynamicsSolver, LinkInertia, RobotDynamicsModel, six_dof_articulated_dyn


def test_pendulum_gravity():
    """1-DOF pendulum: gravity torque should match analytical mgL sin(theta)."""
    model = RobotDynamicsModel(
        dh_a=np.array([1.0]),
        dh_alpha=np.array([0.0]),
        dh_d=np.array([0.0]),
        links=[
            LinkInertia(
                mass=1.0,
                com=np.array([0.5, 0.0, 0.0]),
                inertia=np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]]),
            )
        ],
        gravity=np.array([-9.81, 0.0, 0.0]),
        joint_damping=np.zeros(1),
    )
    solver = DynamicsSolver(model)
    for deg in [0, 30, 60, 90]:
        theta = np.deg2rad(deg)
        tau = solver.gravity_torque(np.array([theta]))
        expected = 1.0 * 9.81 * 0.5 * np.sin(theta)
        assert abs(abs(tau[0]) - expected) < 0.02, f"Expected {expected:.4f}, got {tau[0]:.4f}"
        print(f"  [PASS] test_pendulum_gravity at {deg}°: tau={tau[0]:.4f}")


def test_pendulum_zero_velocity():
    """At zero velocity and acceleration, only gravity torque should be non-zero."""
    model = RobotDynamicsModel(
        dh_a=np.array([1.0]),
        dh_alpha=np.array([0.0]),
        dh_d=np.array([0.0]),
        links=[
            LinkInertia(
                mass=1.0,
                com=np.array([0.5, 0.0, 0.0]),
                inertia=np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]]),
            )
        ],
        gravity=np.array([-9.81, 0.0, 0.0]),
        joint_damping=np.zeros(1),
    )
    solver = DynamicsSolver(model)
    tau_full = solver.inverse_dynamics(np.array([np.pi / 6]), np.zeros(1), np.zeros(1))
    tau_grav = solver.gravity_torque(np.array([np.pi / 6]))
    assert abs(tau_full[0] - tau_grav[0]) < 1e-10
    print("  [PASS] test_pendulum_zero_velocity")


def test_coriolis_no_gravity():
    """Coriolis torque should exclude gravity contribution."""
    model = RobotDynamicsModel(
        dh_a=np.array([1.0]),
        dh_alpha=np.array([0.0]),
        dh_d=np.array([0.0]),
        links=[
            LinkInertia(
                mass=1.0,
                com=np.array([0.5, 0.0, 0.0]),
                inertia=np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]]),
            )
        ],
        gravity=np.array([-9.81, 0.0, 0.0]),
        joint_damping=np.zeros(1),
    )
    solver = DynamicsSolver(model)
    tau_full = solver.inverse_dynamics(np.array([0.5]), np.array([2.0]), np.zeros(1))
    tau_grav = solver.gravity_torque(np.array([0.5]))
    tau_cor = solver.coriolis_torque(np.array([0.5]), np.array([2.0]))
    assert abs(tau_full[0] - tau_grav[0] - tau_cor[0]) < 1e-10
    print("  [PASS] test_coriolis_no_gravity")


def test_6dof_gravity_nonzero():
    """6-DOF robot at zero config should have gravity torques on shoulder/elbow."""
    solver = DynamicsSolver(six_dof_articulated_dyn())
    tau = solver.gravity_torque(np.zeros(6))
    assert np.sum(np.abs(tau) > 0.01) >= 1, "Expected gravity torques on some joints"
    print(f"  [PASS] test_6dof_gravity_nonzero (tau={np.round(tau, 2)})")


def test_6dof_roundtrip():
    """Inverse dynamics with zero accel should match gravity+coriolis."""
    solver = DynamicsSolver(six_dof_articulated_dyn())
    for _ in range(10):
        q = np.random.uniform(-1, 1, 6)
        qd = np.random.uniform(-2, 2, 6)
        tau_full = solver.inverse_dynamics(q, qd, np.zeros(6))
        tau_grav = solver.gravity_torque(q)
        tau_cor = solver.coriolis_torque(q, qd)
        assert np.allclose(tau_full, tau_grav + tau_cor, atol=1e-8)
    print("  [PASS] test_6dof_roundtrip (10 random configs)")


def benchmark():
    solver = DynamicsSolver(six_dof_articulated_dyn())
    times = []
    for _ in range(500):
        q = np.random.uniform(-1, 1, 6)
        qd = np.random.uniform(-2, 2, 6)
        qdd = np.random.uniform(-5, 5, 6)
        start = time.perf_counter()
        solver.inverse_dynamics(q, qd, qdd)
        times.append(time.perf_counter() - start)
    t = np.array(times) * 1000
    print("\n  Benchmark (500 runs):")
    print(f"    Avg:  {np.mean(t):.2f} ms")
    print(f"    P50:  {np.median(t):.2f} ms")
    print(f"    P95:  {np.percentile(t, 95):.2f} ms")


if __name__ == "__main__":
    print("=== robot-dyn Test Suite ===\n")
    test_pendulum_gravity()
    test_pendulum_zero_velocity()
    test_coriolis_no_gravity()
    test_6dof_gravity_nonzero()
    test_6dof_roundtrip()
    benchmark()
    print("\n=== All tests passed ===")
