"""6-DOF Inverse Kinematics Solver — Robot IP Core

Jacobian-based iterative IK with DH parameter forward kinematics.
Solves for joint angles given desired end-effector pose (position + orientation).
Supports arbitrary 6-DOF serial manipulators via Denavit-Hartenberg parameters.

Author: Danny Zeng
License: MIT
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class DHParam:
    """Single Denavit-Hartenberg link parameter."""

    a: float  # link length (meters)
    alpha: float  # link twist (radians)
    d: float  # link offset (meters)
    theta: float  # joint angle (radians) — variable for revolute joints


def dh_transform(dh: DHParam) -> np.ndarray:
    """Compute 4x4 homogeneous transform from a single DH parameter set.

    Standard DH convention: T = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
    """
    ct, st = np.cos(dh.theta), np.sin(dh.theta)
    ca, sa = np.cos(dh.alpha), np.sin(dh.alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, dh.a * ct],
            [st, ct * ca, -ct * sa, dh.a * st],
            [0, sa, ca, dh.d],
            [0, 0, 0, 1],
        ]
    )


class RobotModel:
    """Serial manipulator defined by Denavit-Hartenberg parameters."""

    def __init__(
        self, dh_params: list[DHParam], joint_limits: list[tuple[float, float]] | None = None
    ):
        """
        Args:
            dh_params: List of DH parameters, one per joint (6 for 6-DOF).
            joint_limits: Optional [(min, max)] in radians per joint. Used to clamp IK solutions.
        """
        if len(dh_params) != 6:
            raise ValueError(f"Expected 6 DH parameters for 6-DOF robot, got {len(dh_params)}")
        self.dh_params = dh_params
        self.joint_limits = joint_limits or [(-np.pi, np.pi)] * 6

    def forward_kinematics(self, joint_angles: np.ndarray, return_all: bool = False):
        """Compute end-effector pose given joint angles.

        Args:
            joint_angles: 6-element array of joint angles in radians.
            return_all: If True, return all link transforms.

        Returns:
            T: 4x4 homogeneous transform matrix from base to end-effector.
            If return_all, also returns list of intermediate transforms.
        """
        T = np.eye(4)
        transforms = [T.copy()]
        for i, dh in enumerate(self.dh_params):
            dh_i = DHParam(dh.a, dh.alpha, dh.d, joint_angles[i])
            T = T @ dh_transform(dh_i)
            transforms.append(T.copy())
        if return_all:
            return T, transforms
        return T

    def end_effector_pose(self, joint_angles: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return (position, rotation_matrix) of end-effector."""
        T = self.forward_kinematics(joint_angles)
        return T[:3, 3], T[:3, :3]

    def compute_jacobian(self, joint_angles: np.ndarray) -> np.ndarray:
        """Compute 6x6 geometric Jacobian at given joint angles.

        Returns:
            J: 6x6 matrix [v_x, v_y, v_z, omega_x, omega_y, omega_z]^T
        """
        _, transforms = self.forward_kinematics(joint_angles, return_all=True)
        T_ee = transforms[-1]
        p_ee = T_ee[:3, 3]

        J = np.zeros((6, len(joint_angles)))
        for i in range(len(joint_angles)):
            T_i = transforms[i]  # transform to frame i
            z_i = T_i[:3, 2]  # z-axis of frame i (rotation axis)
            p_i = T_i[:3, 3]  # origin of frame i

            # Linear velocity contribution
            J[:3, i] = np.cross(z_i, p_ee - p_i)
            # Angular velocity contribution
            J[3:, i] = z_i

        return J

    def ik_solve(
        self,
        target_pose: np.ndarray,  # 4x4 homogeneous transform
        initial_guess: np.ndarray | None = None,
        max_iterations: int = 200,
        position_tolerance: float = 1e-4,  # meters
        orientation_tolerance: float = 1e-3,  # radians
        damping: float = 0.1,
    ) -> tuple[bool, np.ndarray, int, list[float]]:
        """Inverse kinematics using damped least-squares (Levenberg-Marquardt).

        Solves for joint angles that achieve the target end-effector pose.

        Args:
            target_pose: 4x4 desired end-effector homogeneous transform.
            initial_guess: Starting joint angles (default: all zeros).
            max_iterations: Maximum solver iterations.
            position_tolerance: Convergence threshold for position (meters).
            orientation_tolerance: Convergence threshold for orientation (radians).
            damping: Damping factor for numerical stability.

        Returns:
            (success, joint_angles, iterations, error_history)
        """
        q = initial_guess.copy() if initial_guess is not None else np.zeros(6)
        target_pos = target_pose[:3, 3]
        target_rot = target_pose[:3, :3]

        errors = []
        for iteration in range(max_iterations):
            # Current pose
            T_current = self.forward_kinematics(q)
            current_pos = T_current[:3, 3]
            current_rot = T_current[:3, :3]

            # Position error
            pos_error = target_pos - current_pos
            # Orientation error (axis-angle from rotation error matrix)
            R_error = target_rot @ current_rot.T
            theta = np.arccos(np.clip((np.trace(R_error) - 1) / 2, -1, 1))
            if abs(theta) > 1e-10:
                axis = np.array(
                    [
                        R_error[2, 1] - R_error[1, 2],
                        R_error[0, 2] - R_error[2, 0],
                        R_error[1, 0] - R_error[0, 1],
                    ]
                ) / (2 * np.sin(theta))
                orient_error = theta * axis
            else:
                orient_error = np.zeros(3)

            error = np.concatenate([pos_error, orient_error])
            total_err = np.linalg.norm(error)
            errors.append(total_err)

            # Check convergence
            if (
                np.linalg.norm(pos_error) < position_tolerance
                and np.linalg.norm(orient_error) < orientation_tolerance
            ):
                # Clamp to joint limits
                q = np.clip(
                    q, [lim[0] for lim in self.joint_limits], [lim[1] for lim in self.joint_limits]
                )
                return True, q, iteration + 1, errors

            # Compute Jacobian and update
            J = self.compute_jacobian(q)
            # Adaptive damping: increase when near singularity
            try:
                cond = np.linalg.cond(J @ J.T)
                lam = damping * (1.0 + np.log10(max(cond, 1.0)) * 0.1)
            except (np.linalg.LinAlgError, RuntimeWarning):
                lam = damping * 10.0  # fallback: heavy damping for ill-conditioned
            # Damped pseudo-inverse: J^T (J J^T + lambda^2 I)^-1
            JJT = J @ J.T
            damped = JJT + lam**2 * np.eye(6)
            try:
                delta_q = J.T @ np.linalg.solve(damped, error)
            except np.linalg.LinAlgError:
                # Fallback to standard pseudo-inverse
                delta_q = np.linalg.pinv(J) @ error

            # Guard against NaN from ill-conditioned solves
            delta_q = np.nan_to_num(delta_q, nan=0.0, posinf=1.0, neginf=-1.0)
            q = q + delta_q
            # Clamp to joint limits
            q = np.clip(
                q, [lim[0] for lim in self.joint_limits], [lim[1] for lim in self.joint_limits]
            )

        return False, q, max_iterations, errors


# ============================================================
# Pre-built robot models
# ============================================================


def six_dof_articulated():
    """Standard 6-DOF articulated robot (like a typical industrial arm).

    DH parameters for a 6R anthropomorphic arm with spherical wrist.
    """
    return RobotModel(
        [
            DHParam(a=0, alpha=-np.pi / 2, d=0.3, theta=0),  # Base rotation
            DHParam(a=0.5, alpha=0, d=0, theta=0),  # Shoulder
            DHParam(a=0.1, alpha=-np.pi / 2, d=0, theta=0),  # Elbow
            DHParam(a=0, alpha=np.pi / 2, d=0.4, theta=0),  # Wrist 1
            DHParam(a=0, alpha=-np.pi / 2, d=0, theta=0),  # Wrist 2
            DHParam(a=0, alpha=0, d=0.1, theta=0),  # Wrist 3 (tool)
        ],
        joint_limits=[
            (-np.pi, np.pi),  # Base: full rotation
            (-np.pi / 2, np.pi / 2),  # Shoulder: -90 to +90 degrees
            (-np.pi * 3 / 4, np.pi * 3 / 4),  # Elbow
            (-np.pi, np.pi),  # Wrist 1
            (-np.pi / 2, np.pi / 2),  # Wrist 2
            (-np.pi, np.pi),  # Wrist 3
        ],
    )


def spherical_wrist_6dof():
    """6-DOF robot with spherical wrist — analytically solvable geometry.

    Used to verify numerical IK against closed-form solutions.
    """
    return RobotModel(
        [
            DHParam(a=0, alpha=0, d=0.3, theta=0),
            DHParam(a=0.3, alpha=-np.pi / 2, d=0, theta=0),
            DHParam(a=0.3, alpha=0, d=0, theta=0),
            DHParam(a=0, alpha=-np.pi / 2, d=0.3, theta=0),
            DHParam(a=0, alpha=np.pi / 2, d=0, theta=0),
            DHParam(a=0, alpha=-np.pi / 2, d=0.1, theta=0),
        ]
    )
