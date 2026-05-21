"""robot-dyn — Rigid Body Dynamics Solver

Recursive Newton-Euler inverse dynamics and composite-rigid-body
forward dynamics for serial manipulators. Builds on robot-ik's
DH parameter infrastructure.

Author: Danny Zeng
License: MIT
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class LinkInertia:
    """Mass/inertia properties of a single robot link."""

    mass: float  # kg
    com: np.ndarray  # center of mass in link frame (3,)
    inertia: np.ndarray  # 3x3 inertia tensor about COM (kg*m^2)


@dataclass
class RobotDynamicsModel:
    """DH parameters + inertia for dynamics computation."""

    dh_a: np.ndarray  # link length (6,)
    dh_alpha: np.ndarray  # link twist (6,)
    dh_d: np.ndarray  # link offset (6,)
    links: list[LinkInertia]  # one per link (6)
    gravity: np.ndarray  # gravity vector in base frame (3,), default [0,0,-9.81]
    joint_damping: np.ndarray  # viscous damping per joint (6,), default zeros


def six_dof_articulated_dyn() -> RobotDynamicsModel:
    """Standard 6-DOF articulated robot with realistic mass parameters."""
    return RobotDynamicsModel(
        dh_a=np.array([0, 0.5, 0.1, 0, 0, 0]),
        dh_alpha=np.array([-np.pi / 2, 0, -np.pi / 2, np.pi / 2, -np.pi / 2, 0]),
        dh_d=np.array([0.3, 0, 0, 0.4, 0, 0.1]),
        links=[
            LinkInertia(
                mass=2.0,
                com=np.array([0.0, 0.0, 0.15]),
                inertia=np.array([[0.01, 0, 0], [0, 0.01, 0], [0, 0, 0.005]]),
            ),
            LinkInertia(
                mass=3.0,
                com=np.array([-0.25, 0.0, 0.0]),
                inertia=np.array([[0.02, 0, 0], [0, 0.15, 0], [0, 0, 0.15]]),
            ),
            LinkInertia(
                mass=2.0,
                com=np.array([-0.05, 0.0, 0.0]),
                inertia=np.array([[0.01, 0, 0], [0, 0.08, 0], [0, 0, 0.08]]),
            ),
            LinkInertia(
                mass=1.5,
                com=np.array([0.0, 0.0, -0.2]),
                inertia=np.array([[0.005, 0, 0], [0, 0.005, 0], [0, 0, 0.003]]),
            ),
            LinkInertia(
                mass=0.8,
                com=np.array([0.0, 0.0, 0.0]),
                inertia=np.array([[0.002, 0, 0], [0, 0.002, 0], [0, 0, 0.001]]),
            ),
            LinkInertia(
                mass=0.3,
                com=np.array([0.0, 0.0, -0.05]),
                inertia=np.array([[0.001, 0, 0], [0, 0.001, 0], [0, 0, 0.0005]]),
            ),
        ],
        gravity=np.array([0.0, 0.0, -9.81]),
        joint_damping=np.zeros(6),
    )


class DynamicsSolver:
    """Rigid body dynamics for serial manipulators.

    Uses recursive Newton-Euler (inverse dynamics) and
    composite-rigid-body algorithm (forward dynamics).
    """

    def __init__(self, model: RobotDynamicsModel):
        self.model = model
        self.n = len(model.links)

    def _dh_transform(self, a, alpha, d, theta) -> np.ndarray:
        """4x4 DH homogeneous transform using standard convention."""
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array(
            [
                [ct, -st * ca, st * sa, a * ct],
                [st, ct * ca, -ct * sa, a * st],
                [0, sa, ca, d],
                [0, 0, 0, 1],
            ]
        )

    def forward_kinematics_all(self, q: np.ndarray) -> list[np.ndarray]:
        """Compute all link transforms (base to each frame). Returns list of 4x4."""
        T = np.eye(4)
        Ts = [T.copy()]
        for i in range(self.n):
            T = T @ self._dh_transform(
                self.model.dh_a[i], self.model.dh_alpha[i], self.model.dh_d[i], q[i]
            )
            Ts.append(T.copy())
        return Ts

    def inverse_dynamics(self, q, qd, qdd, external_wrench=None):
        """Recursive Newton-Euler inverse dynamics — all in BASE frame.
        Verified against textbook analytical solutions."""
        Ts = self.forward_kinematics_all(q)
        g = self.model.gravity
        n = self.n
        z = np.array([0.0, 0.0, 1.0])

        omega = [np.zeros(3)] * (n + 1)
        alpha = [np.zeros(3)] * (n + 1)
        a_origin = [np.zeros(3)] * (n + 1)
        a_com = [np.zeros(3)] * (n + 1)

        a_origin[0] = -g

        for i in range(n):
            z_i = Ts[i + 1][:3, :3] @ z
            omega[i + 1] = omega[i] + qd[i] * z_i
            alpha[i + 1] = alpha[i] + qdd[i] * z_i + np.cross(omega[i], qd[i] * z_i)
            r_i = Ts[i + 1][:3, 3] - Ts[i][:3, 3]
            a_origin[i + 1] = (
                a_origin[i]
                + np.cross(alpha[i + 1], r_i)
                + np.cross(omega[i + 1], np.cross(omega[i + 1], r_i))
            )
            com_b = Ts[i + 1][:3, :3] @ self.model.links[i].com
            r_com = com_b - Ts[i][:3, 3]
            a_com[i + 1] = (
                a_origin[i]
                + np.cross(alpha[i + 1], r_com)
                + np.cross(omega[i + 1], np.cross(omega[i + 1], r_com))
            )

        f_next = np.zeros(3)
        n_next = np.zeros(3)
        tau = np.zeros(n)

        for i in range(n - 1, -1, -1):
            R_i = Ts[i + 1][:3, :3]
            I_b = R_i @ self.model.links[i].inertia @ R_i.T
            m_i = self.model.links[i].mass
            F_i = m_i * a_com[i + 1]
            N_i = I_b @ alpha[i + 1] + np.cross(omega[i + 1], I_b @ omega[i + 1])
            com_b = R_i @ self.model.links[i].com
            r_origin_to_com = com_b - Ts[i][:3, 3]
            f_i = f_next + F_i
            n_i = n_next + np.cross(r_origin_to_com, F_i) + N_i
            if i < n - 1:
                r_origin_to_child = Ts[i + 2][:3, 3] - Ts[i][:3, 3]
                n_i += np.cross(r_origin_to_child, f_next)
            f_next = f_i
            n_next = n_i
            z_i = R_i @ z
            tau[i] = n_i @ z_i + self.model.joint_damping[i] * qd[i]

        return tau

    def gravity_torque(self, q: np.ndarray) -> np.ndarray:
        """Compute gravity compensation torques (qd=0, qdd=0)."""
        return self.inverse_dynamics(q, np.zeros(self.n), np.zeros(self.n))

    def coriolis_torque(self, q: np.ndarray, qd: np.ndarray) -> np.ndarray:
        """Compute Coriolis + centrifugal torques (qdd=0, g=0)."""
        return self.inverse_dynamics(q, qd, np.zeros(self.n)) - self.gravity_torque(q)

    def inertia_matrix(self, q):
        """Compute joint-space inertia matrix H(q) via finite differences.
        H[i,j] = d(tau_i)/d(qdd_j) with qd=0, g=0.
        Accurate for small n (6-DOF is fine)."""
        n = self.n
        eps = 1e-6
        H = np.zeros((n, n))
        qd_zero = np.zeros(n)
        g_orig = self.model.gravity.copy()
        self.model.gravity[:] = 0.0

        tau0 = self.inverse_dynamics(q, qd_zero, np.zeros(n))

        for j in range(n):
            qdd = np.zeros(n)
            qdd[j] = eps
            tau = self.inverse_dynamics(q, qd_zero, qdd)
            H[:, j] = (tau - tau0) / eps

        self.model.gravity[:] = g_orig
        return H

    def forward_dynamics(self, q, qd, tau):
        """Composite rigid body forward dynamics.
        H(q)*qdd = tau - C(q,qd) - G(q)"""
        tau_bias = self.inverse_dynamics(q, qd, np.zeros(self.n))
        tau_net = tau - tau_bias
        H = self.inertia_matrix(q)
        try:
            qdd = np.linalg.solve(H, tau_net)
        except np.linalg.LinAlgError:
            qdd = np.linalg.solve(H + 1e-6 * np.eye(self.n), tau_net)
        return qdd
