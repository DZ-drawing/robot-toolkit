"""3D visualization for the IK solver using matplotlib."""

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

from robot_ik.ik.solver import RobotModel


def plot_robot(
    robot: RobotModel,
    joint_angles: np.ndarray,
    target_pose: np.ndarray = None,
    title: str = "Robot Pose",
    ax: Axes3D = None,
):
    """Plot the robot arm in 3D at given joint angles.

    Args:
        robot: RobotModel instance.
        joint_angles: 6-element joint angle array.
        target_pose: Optional 4x4 target pose to display as ghost frame.
        title: Plot title.
        ax: Optional existing matplotlib 3D axis.
    """
    if ax is None:
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection="3d")

    # Get all link transforms
    _, transforms = robot.forward_kinematics(joint_angles, return_all=True)

    # Extract joint positions
    xs, ys, zs = [], [], []
    for T in transforms:
        xs.append(T[0, 3])
        ys.append(T[1, 3])
        zs.append(T[2, 3])

    # Plot arm as connected line
    ax.plot(xs, ys, zs, "b-o", linewidth=2, markersize=6, label="Arm links")
    ax.scatter(xs[0], ys[0], zs[0], c="green", s=100, marker="s", label="Base")
    ax.scatter(xs[-1], ys[-1], zs[-1], c="red", s=100, marker="^", label="End-effector")

    # Plot target if provided
    if target_pose is not None:
        tp = target_pose[:3, 3]
        ax.scatter(
            tp[0],
            tp[1],
            tp[2],
            c="orange",
            s=120,
            marker="*",
            label="Target",
            alpha=0.8,
            edgecolors="black",
            linewidth=0.5,
        )

        # Draw target orientation axes
        axis_len = 0.15
        colors = ["r", "g", "b"]
        for i, color in enumerate(colors):
            ax.quiver(
                tp[0],
                tp[1],
                tp[2],
                target_pose[0, i] * axis_len,
                target_pose[1, i] * axis_len,
                target_pose[2, i] * axis_len,
                color=color,
                alpha=0.6,
                linewidth=1.5,
            )

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(title)
    ax.legend(loc="upper left", fontsize=8)

    # Set equal aspect ratio
    all_coords = np.array([xs, ys, zs])
    max_range = np.max(np.ptp(all_coords, axis=1)) / 2
    mid = np.mean(all_coords, axis=1)
    ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
    ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
    ax.set_zlim(mid[2] - max_range, mid[2] + max_range)

    return ax


def plot_convergence(errors, title="IK Convergence"):
    """Plot IK error over iterations."""
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.semilogy(errors, "b-", linewidth=1.5)
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Error (log scale)")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    return ax


def demo():
    """Run a full demo: solve IK for random poses and visualize."""
    from robot_ik.ik_solver import six_dof_articulated

    robot = six_dof_articulated()
    np.random.seed(42)

    fig = plt.figure(figsize=(14, 10))

    for i in range(4):
        # Random target
        q_rand = np.random.uniform(-1, 1, 6)
        target = robot.forward_kinematics(q_rand)

        # Solve IK
        success, q_solved, iters, errors = robot.ik_solve(target)

        # Plot robot at solved config
        ax = fig.add_subplot(2, 2, i + 1, projection="3d")
        plot_robot(
            robot,
            q_solved,
            target_pose=target,
            title=f"Solved in {iters} iters (err={errors[-1]:.1e})",
            ax=ax,
        )

    plt.tight_layout()
    plt.savefig("demo_ik_solutions.png", dpi=150, bbox_inches="tight")
    print("Demo saved to demo_ik_solutions.png")
    plt.show()


if __name__ == "__main__":
    demo()
