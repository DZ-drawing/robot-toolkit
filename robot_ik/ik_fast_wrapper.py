"""Fast IK solver — C++ extension with Python fallback."""

import numpy as np

# Try loading C++ extension
_has_fast = False
try:
    from robot_ik.ik_fast import compute_jacobian as _jac_cpp
    from robot_ik.ik_fast import forward_kinematics as _fk_cpp
    from robot_ik.ik_fast import ik_solve as _ik_cpp

    _has_fast = True
except ImportError:
    pass


class FastIKSolver:
    """High-performance IK solver using C++ extension.

    Automatically falls back to pure Python if extension not available.
    """

    def __init__(self, dh_params: np.ndarray, joint_limits: np.ndarray):
        self.dh = np.asarray(dh_params, dtype=np.float64)
        self.limits = np.asarray(joint_limits, dtype=np.float64)

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        if _has_fast:
            return _fk_cpp(self.dh, np.asarray(q, dtype=np.float64))
        from robot_ik.ik_solver import DHParam, RobotModel

        robot = RobotModel([DHParam(*row) for row in self.dh])
        return robot.forward_kinematics(q)

    def compute_jacobian(self, q: np.ndarray) -> np.ndarray:
        if _has_fast:
            return _jac_cpp(self.dh, np.asarray(q, dtype=np.float64))
        from robot_ik.ik_solver import DHParam, RobotModel

        robot = RobotModel([DHParam(*row) for row in self.dh])
        return robot.compute_jacobian(q)

    def ik_solve(
        self,
        target: np.ndarray,
        initial_guess: np.ndarray | None = None,
        max_iterations=200,
        position_tolerance=1e-4,
        orientation_tolerance=1e-3,
        damping=0.1,
    ) -> tuple[bool, np.ndarray, int, np.ndarray]:
        if _has_fast:
            q0 = np.asarray(
                initial_guess if initial_guess is not None else np.zeros(6), dtype=np.float64
            )
            return _ik_cpp(
                self.dh,
                target,
                q0,
                self.limits,
                max_iterations,
                position_tolerance,
                orientation_tolerance,
                damping,
            )
        from robot_ik.ik_solver import DHParam, RobotModel

        robot = RobotModel(
            [DHParam(*row) for row in self.dh],
            [(self.limits[i, 0], self.limits[i, 1]) for i in range(6)],
        )
        return robot.ik_solve(
            target,
            initial_guess,
            max_iterations,
            position_tolerance,
            orientation_tolerance,
            damping,
        )

    @property
    def has_fast(self) -> bool:
        return _has_fast
