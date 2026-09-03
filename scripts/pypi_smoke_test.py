#!/usr/bin/env python3
"""Smoke test for the PyPI-released robot-ik wheel (fresh venv install).

Verifies the published 0.4.0 wheel end-to-end: metadata, C++ extensions,
forward/inverse kinematics on a 6-DOF articulated arm, and collision
(principal new feature of 0.4.0).
"""

import numpy as np

import robot_ik
from robot_ik import Box, CollisionChecker, DHParam, RobotModel, Sphere

print("version:", robot_ik.__version__)
print("HAS_IK_FAST:", robot_ik.HAS_IK_FAST, "HAS_DYN_FAST:", robot_ik.HAS_DYN_FAST)

# 6-DOF articulated arm (same DH as tutorials)
model = RobotModel(
    [
        DHParam(0.4, np.pi / 2, 0.0, 0.0),
        DHParam(0.9, 0.0, 0.0, 0.0),
        DHParam(0.1, np.pi / 2, 0.0, 0.0),
        DHParam(0.0, -np.pi / 2, 0.3, 0.0),
        DHParam(0.0, np.pi / 2, 0.0, 0.0),
        DHParam(0.0, 0.0, 0.15, 0.0),
    ]
)

# --- FK ---
q = np.array([0.1, -0.2, 0.3, 0.5, -0.4, 0.2])
T = model.forward_kinematics(q)
print("FK endpoint:", np.round(T[:3, 3], 4))

# --- IK via FastIKSolver (C++ damped-LS with Python fallback) ---
from robot_ik.ik.fast_wrapper import FastIKSolver

dh = np.array([[p.a, p.alpha, p.d, p.theta] for p in model.dh_params])
limits = np.array([[-2.9, 2.9]] * 6)
solver = FastIKSolver(dh, limits)
converged, q_sol, iters, ik_err = solver.ik_solve(T)
q_sol = np.asarray(q_sol)
T_sol = model.forward_kinematics(q_sol)
err = float(np.linalg.norm(T_sol[:3, 3] - T[:3, 3]))
print(f"IK converged: {converged} iters={iters} position err: {err:.2e}")

# --- Collision (new in 0.4.0): overlapping sphere vs box at origin ---
from robot_ik.collision import distance_sphere_to_box  # noqa: E402

sphere = Sphere(radius=0.1, pose=np.eye(4))
box = Box(size=np.array([0.2, 0.2, 0.2]), pose=np.eye(4))
d = distance_sphere_to_box(sphere, box)
print("sphere-box distance (overlapping):", d)

checker = CollisionChecker()
checker.add_link_geometry("tcp", sphere)
checker.add_obstacle(box)
hit = checker.check_environment_collision({"tcp": np.eye(4)})
print("env collision result:", hit)

assert robot_ik.__version__ == "0.4.0"
assert robot_ik.HAS_IK_FAST and robot_ik.HAS_DYN_FAST
assert converged and err < 1e-3
assert d <= 0.0 and hit is not None
print("SMOKE TEST OK")
