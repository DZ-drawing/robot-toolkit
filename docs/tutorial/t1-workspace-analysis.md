# Tutorial 1: Dual-Arm Workspace Analysis

**Objective**: Visualize and analyze the overlapping reachable workspace of two
6-DOF manipulators placed side by side.

**Features demonstrated**:
- `six_dof_articulated()` model creation
- Forward kinematics for workspace point sampling
- Dual-arm setup with base offset
- Workspace intersection estimation

---

## How it works

We create two instances of the standard 6-DOF articulated robot. The second
arm is offset by 1.5 m along the X axis. We sample random joint-angle
configurations for each arm, compute the end-effector position via forward
kinematics, and then estimate the volume of their shared workspace.

```python
"""Tutorial 1: Dual-Arm Workspace Analysis.

Visualize and analyze the overlapping reachable workspace of two 6-DOF
manipulators. Run with:  python -m docs.tutorial.t1_workspace_analysis
"""

from robot_ik import six_dof_articulated
import numpy as np


def sample_workspace(robot, n_samples: int = 5000, seed: int = 42) -> np.ndarray:
    """Sample reachable end-effector positions by random joint-angle sweep.

    Returns:
        (n_samples, 3) array of [x, y, z] positions.
    """
    rng = np.random.default_rng(seed)
    points = np.zeros((n_samples, 3))

    for i in range(n_samples):
        # Random angles within the robot's joint limits
        q = np.array([
            rng.uniform(lo, hi)
            for lo, hi in robot.joint_limits
        ])
        T = robot.forward_kinematics(q)
        points[i] = T[:3, 3]

    return points


def estimate_overlap_volume(
    pts_a: np.ndarray,
    pts_b: np.ndarray,
    resolution: float = 0.1,
) -> dict:
    """Roughly estimate the shared workspace volume using voxel occupancy.

    Args:
        pts_a, pts_b: (N, 3) workspace point clouds.
        resolution: Voxel edge length in metres.

    Returns:
        Dict with 'volume', 'overlap_count', keys.
    """
    # Merge clouds, snap to voxel grid
    all_pts = np.vstack([pts_a, pts_b])

    vox_a = set()
    vox_b = set()

    for p in pts_a:
        vox_a.add(tuple(np.floor(p / resolution).astype(int)))
    for p in pts_b:
        vox_b.add(tuple(np.floor(p / resolution).astype(int)))

    overlap_voxels = vox_a & vox_b
    voxel_volume = resolution ** 3

    return {
        "volume": len(overlap_voxels) * voxel_volume,
        "overlap_count": len(overlap_voxels),
        "voxel_resolution": resolution,
    }


def main() -> None:
    # -- Build the two arms ------------------------------------------------
    arm1 = six_dof_articulated()
    arm2 = six_dof_articulated()

    # Offset arm2's base by 1.5 m along X.
    # RobotModel stores DH params; we shift FK output for arm2.
    arm2_base_offset = np.array([1.5, 0.0, 0.0])

    def fk_arm2(q: np.ndarray) -> np.ndarray:
        T = arm2.forward_kinematics(q)
        T[:3, 3] += arm2_base_offset
        return T

    # -- Sample workspaces -------------------------------------------------
    n = 5000
    ws1 = sample_workspace(arm1, n_samples=n)
    ws2_raw = sample_workspace(arm2, n_samples=n)
    # Apply the base offset to arm2's workspace points
    ws2 = ws2_raw + arm2_base_offset

    print(f"Arm 1 workspace: {n} samples, "
          f"X range [{ws1[:, 0].min():.2f}, {ws1[:, 0].max():.2f}] m")
    print(f"Arm 2 workspace: {n} samples, "
          f"X range [{ws2[:, 0].min():.2f}, {ws2[:, 0].max():.2f}] m")

    # -- Compute overlap ---------------------------------------------------
    overlap = estimate_overlap_volume(ws1, ws2)
    print(f"\nShared workspace volume: {overlap['volume']:.4f} m^3  "
          f"({overlap['overlap_count']} voxels @ {overlap['voxel_resolution']} m)")

    return ws1, ws2, overlap


if __name__ == "__main__":
    main()
```

## Learning outcomes

- Workspace sampling via random joint-angle sweep and forward kinematics
- Dual-arm setup and coordinate-frame offsets
- Voxel-based overlap estimation for shared task planning
- Practical understanding of how base placement affects usable workspace

## Notes

- Increasing `n_samples` improves accuracy at the cost of speed.
- The voxel resolution (`0.1 m` by default) controls the granularity of the
  volume estimate.
- For publication-quality workspace maps, use `matplotlib` 3D scatter plots
  or `pyvista` meshes.
