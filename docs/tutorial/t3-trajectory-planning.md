[English](t3-trajectory-planning.md) | [中文](../zh/tutorial/t3-trajectory-planning.md)
# Tutorial 3: Coordinated Dual-Arm Trajectory Planning

**Objective**: Generate time-synchronized trajectories for two arms moving a
shared object, using waypoint-based interpolation with S-curve velocity
profiling.

**Features demonstrated**:
- `joint_cubic_interpolation` for smooth single-segment motion
- `s_curve_profile` for jerk-limited trajectories
- `waypoint_trajectory` for multi-waypoint paths
- Dual-arm temporal coordination

---

## How it works

Each arm follows its own sequence of joint-space waypoints. We generate
trajectories with matching time vectors so both arms start and stop
simultaneously — a requirement when they jointly carry an object.

```python
"""Tutorial 3: Coordinated Dual-Arm Trajectory Planning.

Generate synchronised joint-space trajectories for two arms performing
a pick-and-place operation on a shared object.

Run with:  python -m docs.tutorial.t3_trajectory_planning
"""

import numpy as np
from robot_ik.trajectory import (
    TrajectoryResult,
    joint_cubic_interpolation,
    joint_quintic_interpolation,
    s_curve_profile,
    waypoint_trajectory,
    trapezoidal_velocity_profile,
)


def print_summary(label: str, traj: TrajectoryResult) -> None:
    """Print trajectory statistics."""
    q = traj.joint_positions
    v = traj.joint_velocities
    print(f"\n{label}:")
    print(f"  Duration       : {traj.duration:.2f} s  "
          f"({len(traj.time_points)} samples)")
    print(f"  Joint range    : [{q.min():.3f}, {q.max():.3f}] rad")
    print(f"  Max |velocity| : {np.abs(v).max():.3f} rad/s")
    print(f"  Start position : {q[0]}")
    print(f"  End position   : {q[-1]}")


def main() -> None:
    dof = 6
    dt = 0.01  # 100 Hz

    # ------------------------------------------------------------------
    # 1. Cubic interpolation (smooth, zero velocity at boundaries)
    # ------------------------------------------------------------------
    q_start = np.zeros(dof)
    q_end = np.array([0.5, -0.3, 0.4, 0.0, 0.2, 0.0])

    traj_cubic = joint_cubic_interpolation(q_start, q_end, duration=2.0, dt=dt)
    print_summary("Cubic interpolation", traj_cubic)

    # ------------------------------------------------------------------
    # 2. S-curve profile (jerk-limited, C2 continuous)
    # ------------------------------------------------------------------
    v_max = np.full(dof, 2.0)   # rad/s
    a_max = np.full(dof, 4.0)   # rad/s^2
    j_max = np.full(dof, 20.0)  # rad/s^3

    traj_scurve = s_curve_profile(
        q_start, q_end, duration=2.0, v_max=v_max, a_max=a_max, j_max=j_max, dt=dt
    )
    print_summary("S-curve profile", traj_scurve)

    # ------------------------------------------------------------------
    # 3. Multi-waypoint trajectory (quintic between each pair)
    # ------------------------------------------------------------------
    waypoints = [
        np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),   # home
        np.array([0.3, 0.2, -0.5, 0.0, 0.5, 0.0]),   # approach
        np.array([0.3, 0.2, -0.3, 0.0, 0.5, 0.0]),   # grasp
        np.array([0.3, -0.2, -0.3, 0.0, 0.5, 0.0]),  # lift
        np.array([0.5, -0.3, 0.0, 0.0, 0.2, 0.0]),   # place
    ]
    times = [0.0, 1.0, 2.0, 3.5, 5.0]  # arrival time at each waypoint

    traj_wp = waypoint_trajectory(waypoints, times, method="quintic", dt=dt)
    print_summary("Multi-waypoint (quintic)", traj_wp)

    # ------------------------------------------------------------------
    # 4. Coordinated dual-arm trajectories (same time vector)
    # ------------------------------------------------------------------
    # Arm 1 picks at location A, arm 2 grips the other side (0.6 m offset in Y)
    arm1_waypoints = [
        np.array([0.5, 0.0, 0.3, 0, 0, 0]),
        np.array([0.5, 0.3, 0.3, 0, 0, 0]),
        np.array([0.5, -0.3, 0.3, 0, 0, 0]),
    ]
    arm2_waypoints = [
        wp + np.array([0.0, 0.6, 0.0, 0.0, 0.0, 0.0])
        for wp in arm1_waypoints
    ]
    times_dual = [0.0, 2.0, 4.0]

    traj_arm1 = waypoint_trajectory(arm1_waypoints, times_dual, method="cubic", dt=dt)
    traj_arm2 = waypoint_trajectory(arm2_waypoints, times_dual, method="cubic", dt=dt)

    # Verify time synchronisation
    assert len(traj_arm1.time_points) == len(traj_arm2.time_points), \
        "Trajectories must have the same number of samples"
    max_time_error = np.max(np.abs(traj_arm1.time_points - traj_arm2.time_points))
    assert max_time_error < 1e-9, f"Time mismatch: {max_time_error}"

    print_summary("Dual-arm arm1", traj_arm1)
    print_summary("Dual-arm arm2", traj_arm2)
    print(f"\nTime synchronisation OK  (max dt error = {max_time_error:.1e} s)")


if __name__ == "__main__":
    main()
```

## Learning outcomes

- Cubic / quintic polynomial interpolation between two joint configurations
- S-curve velocity profiles with jerk limiting
- Multi-waypoint trajectory generation with arrival-time specification
- Dual-arm temporal coordination via shared time vectors
- Interpreting `TrajectoryResult` fields

## Key API notes

| Function | Signature | Notes |
|---|---|---|
| `joint_cubic_interpolation` | `(q_start, q_end, duration, dt)` | Zero velocity at endpoints |
| `joint_quintic_interpolation` | `(q_start, q_end, duration, v_start, v_end, ...)` | Full boundary control |
| `s_curve_profile` | `(q_start, q_end, duration, v_max, a_max, j_max, dt)` | Jerk-limited (C2) |
| `trapezoidal_velocity_profile` | `(q_start, q_end, duration, v_max, a_max, dt)` | 3-phase trapezoid |
| `waypoint_trajectory` | `(waypoints, times, method, blend_radius, dt)` | `method`: "linear"/"cubic"/"quintic" |

`TrajectoryResult` fields: `.time_points`, `.joint_positions`, `.joint_velocities`,
`.joint_accelerations`, `.duration`.
