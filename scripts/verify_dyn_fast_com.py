#!/usr/bin/env python3
"""Verify the C++ dynamics extension (robot_dyn_fast) matches the Python
reference implementation and analytical solutions.

Checks the historical COM bug (com_b = R @ com + origin, gravity torque 3x
error) is NOT present in the C++ RNEA:
  1. 1-DOF pendulum gravity torque vs analytical mgL*sin(theta)
  2. C++ vs Python full inverse dynamics parity on a random 6-DOF arm
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from robot_ik.dynamics.solver import (  # noqa: E402
    DynamicsSolver,
    LinkInertia,
    RobotDynamicsModel,
)

try:
    import robot_ik.robot_dyn_fast as dyn_fast
except ImportError as e:
    print(f"FAIL: C++ extension not importable: {e}")
    sys.exit(1)


def make_pendulum() -> tuple[RobotDynamicsModel, np.ndarray]:
    model = RobotDynamicsModel(
        dh_a=np.array([1.0]),
        dh_alpha=np.array([0.0]),
        dh_d=np.array([0.0]),
        links=[
            LinkInertia(
                mass=1.0,
                com=np.array([0.5, 0.0, 0.0]),
                inertia=np.eye(3) * 0.1,
            )
        ],
        gravity=np.array([-9.81, 0.0, 0.0]),
        joint_damping=np.zeros(1),
    )
    dh = np.column_stack([model.dh_a, model.dh_alpha, model.dh_d, np.zeros(1)])
    return model, dh


def make_6dof(seed: int = 42) -> tuple[RobotDynamicsModel, np.ndarray]:
    rng = np.random.default_rng(seed)
    n = 6
    masses = rng.uniform(1.0, 5.0, n)
    coms = rng.uniform(-0.3, 0.3, (n, 3))
    inertias = np.array([np.eye(3) * rng.uniform(0.05, 0.5) for _ in range(n)])
    # articulated 6-DOF DH (same family as six_dof_articulated_dyn)
    dh_a = np.array([0.1, 0.9, 0.1, 0.0, 0.1, 0.05])
    dh_alpha = np.array([np.pi / 2, 0.0, np.pi / 2, -np.pi / 2, np.pi / 2, 0.0])
    dh_d = np.array([0.4, 0.0, 0.2, 0.3, 0.0, 0.15])
    links = [
        LinkInertia(mass=masses[i], com=coms[i], inertia=inertias[i])
        for i in range(n)
    ]
    model = RobotDynamicsModel(
        dh_a=dh_a,
        dh_alpha=dh_alpha,
        dh_d=dh_d,
        links=links,
        gravity=np.array([0.0, 0.0, -9.81]),
        joint_damping=np.full(n, 0.05),
    )
    dh = np.column_stack([dh_a, dh_alpha, dh_d, np.zeros(n)])
    return model, dh


def main() -> int:
    failures = 0

    # --- Check 1: pendulum gravity torque (the historical COM bug test) ---
    model, dh = make_pendulum()
    print("=== Check 1: pendulum gravity torque (C++ vs analytical) ===")
    for deg in [0, 30, 60, 90]:
        theta = np.deg2rad(deg)
        q = np.zeros(1)
        q[0] = theta
        tau_cpp = dyn_fast.inverse_dynamics(
            dh, q, np.zeros(1), np.zeros(1),
            np.array([1.0]), np.array([[0.5, 0.0, 0.0]]),
            np.array([np.eye(3) * 0.1]).reshape(1, 9),
            np.zeros(1), np.array([-9.81, 0.0, 0.0]),
        )
        expected = 1.0 * 9.81 * 0.5 * np.sin(theta)
        if abs(abs(tau_cpp[0]) - expected) > 0.02:
            print(f"  FAIL {deg:2d}°: tau_cpp={tau_cpp[0]:+.4f}, expected |{expected:.4f}|")
            failures += 1
        else:
            print(f"  PASS {deg:2d}°: tau_cpp={tau_cpp[0]:+.4f} (analytical ±{expected:.4f})")

    # --- Check 2: C++ vs Python parity on random 6-DOF arm ---
    model, dh = make_6dof()
    solver = DynamicsSolver(model)
    rng = np.random.default_rng(7)
    print("=== Check 2: C++ vs Python inverse dynamics parity (6-DOF, 50 configs) ===")
    max_err = 0.0
    for _ in range(50):
        q = rng.uniform(-np.pi, np.pi, 6)
        qd = rng.uniform(-2.0, 2.0, 6)
        qdd = rng.uniform(-5.0, 5.0, 6)
        tau_py = solver.inverse_dynamics(q, qd, qdd)
        tau_cpp = dyn_fast.inverse_dynamics(
            dh, q, qd, qdd,
            np.array([l.mass for l in model.links]),
            np.array([l.com for l in model.links]),
            np.array([l.inertia for l in model.links]).reshape(6, 9),
            model.joint_damping,
            model.gravity,
        )
        err = np.max(np.abs(tau_py - tau_cpp))
        max_err = max(max_err, float(err))
    print(f"  max |tau_py - tau_cpp| over 50 random states = {max_err:.3e}")
    if max_err > 1e-8:
        print("  FAIL: C++ and Python dynamics disagree")
        failures += 1
    else:
        print("  PASS: C++ matches Python reference")

    print(f"=== RESULT: {failures} failure(s) ===")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
