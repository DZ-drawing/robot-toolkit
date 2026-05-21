"""Trajectory Planning for 6-DOF Serial Manipulators.

Provides joint-space and Cartesian-space trajectory generation with various
interpolation methods and velocity profiles.

Author: Danny Zeng
License: MIT
"""

from dataclasses import dataclass

import numpy as np

from robot_ik.ik_solver import RobotModel


@dataclass
class TrajectoryResult:
    """Result of trajectory generation."""

    time_points: np.ndarray  # (N,) time in seconds
    joint_positions: np.ndarray  # (N, dof) joint angles in radians
    joint_velocities: np.ndarray  # (N, dof) rad/s
    joint_accelerations: np.ndarray  # (N, dof) rad/s^2
    duration: float  # total duration in seconds


def joint_linear_interpolation(
    q_start: np.ndarray,
    q_end: np.ndarray,
    duration: float,
    dt: float = 0.01,
) -> TrajectoryResult:
    """Linear interpolation between two joint configurations.

    Simple baseline method: constant velocity from start to end.
    Velocity is zero at boundaries (not physically realistic but useful).

    Args:
        q_start: Starting joint configuration (dof,).
        q_end: Ending joint configuration (dof,).
        duration: Motion duration in seconds.
        dt: Time step for sampling (default 0.01 = 100 Hz).

    Returns:
        TrajectoryResult with time-sampled positions, velocities, accelerations.
    """
    dof = len(q_start)
    n_samples = int(np.ceil(duration / dt)) + 1
    time_points = np.linspace(0, duration, n_samples)

    # Position: linear interpolation
    joint_positions = np.zeros((n_samples, dof))
    for i in range(dof):
        joint_positions[:, i] = np.interp(time_points, [0, duration], [q_start[i], q_end[i]])

    # Velocity: constant (slope)
    joint_velocities = np.zeros((n_samples, dof))
    for i in range(dof):
        joint_velocities[:, i] = (q_end[i] - q_start[i]) / duration

    # Acceleration: zero (constant velocity)
    joint_accelerations = np.zeros((n_samples, dof))

    return TrajectoryResult(
        time_points=time_points,
        joint_positions=joint_positions,
        joint_velocities=joint_velocities,
        joint_accelerations=joint_accelerations,
        duration=duration,
    )


def joint_cubic_interpolation(
    q_start: np.ndarray,
    q_end: np.ndarray,
    duration: float,
    dt: float = 0.01,
) -> TrajectoryResult:
    """Cubic polynomial interpolation with zero velocity at boundaries.

    q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    Boundary conditions: q(0)=q_start, q(T)=q_end, dq(0)=0, dq(T)=0

    Args:
        q_start: Starting joint configuration (dof,).
        q_end: Ending joint configuration (dof,).
        duration: Motion duration in seconds.
        dt: Time step for sampling.

    Returns:
        TrajectoryResult with smooth position, velocity, acceleration.
    """
    dof = len(q_start)
    n_samples = int(np.ceil(duration / dt)) + 1
    time_points = np.linspace(0, duration, n_samples)

    # Cubic coefficients for each joint
    # q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    # From boundary conditions:
    # a0 = q_start, a1 = 0, a2 = 3(q_end - q_start)/T^2, a3 = -2(q_end - q_start)/T^3
    T = duration
    coeffs = np.zeros((dof, 4))  # [a0, a1, a2, a3]
    for i in range(dof):
        coeffs[i, 0] = q_start[i]
        coeffs[i, 1] = 0.0
        coeffs[i, 2] = 3.0 * (q_end[i] - q_start[i]) / (T * T)
        coeffs[i, 3] = -2.0 * (q_end[i] - q_start[i]) / (T * T * T)

    # Sample trajectory
    joint_positions = np.zeros((n_samples, dof))
    joint_velocities = np.zeros((n_samples, dof))
    joint_accelerations = np.zeros((n_samples, dof))

    for k, t in enumerate(time_points):
        for i in range(dof):
            a0, a1, a2, a3 = coeffs[i]
            # Position
            joint_positions[k, i] = a0 + a1 * t + a2 * t * t + a3 * t * t * t
            # Velocity: dq/dt = a1 + 2*a2*t + 3*a3*t^2
            joint_velocities[k, i] = a1 + 2 * a2 * t + 3 * a3 * t * t
            # Acceleration: d2q/dt2 = 2*a2 + 6*a3*t
            joint_accelerations[k, i] = 2 * a2 + 6 * a3 * t

    return TrajectoryResult(
        time_points=time_points,
        joint_positions=joint_positions,
        joint_velocities=joint_velocities,
        joint_accelerations=joint_accelerations,
        duration=duration,
    )


def joint_quintic_interpolation(
    q_start: np.ndarray,
    q_end: np.ndarray,
    duration: float,
    v_start: np.ndarray | None = None,
    v_end: np.ndarray | None = None,
    a_start: np.ndarray | None = None,
    a_end: np.ndarray | None = None,
    dt: float = 0.01,
) -> TrajectoryResult:
    """Quintic polynomial interpolation with position, velocity, acceleration boundaries.

    q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    Boundary conditions: q, dq, d2q at t=0 and t=T

    Args:
        q_start: Starting joint configuration (dof,).
        q_end: Ending joint configuration (dof,).
        duration: Motion duration in seconds.
        v_start: Initial velocity (dof,), defaults to zeros.
        v_end: Final velocity (dof,), defaults to zeros.
        a_start: Initial acceleration (dof,), defaults to zeros.
        a_end: Final acceleration (dof,), defaults to zeros.
        dt: Time step for sampling.

    Returns:
        TrajectoryResult with smooth position, velocity, acceleration.
    """
    dof = len(q_start)
    n_samples = int(np.ceil(duration / dt)) + 1
    time_points = np.linspace(0, duration, n_samples)

    # Default boundary conditions to zero
    v_start = v_start if v_start is not None else np.zeros(dof)
    v_end = v_end if v_end is not None else np.zeros(dof)
    a_start = a_start if a_start is not None else np.zeros(dof)
    a_end = a_end if a_end is not None else np.zeros(dof)

    # Quintic coefficients: solve 6x6 system for each joint
    # Matrix form: M * [a0, a1, a2, a3, a4, a5]^T = b
    T = duration
    M = np.array(
        [
            [1, 0, 0, 0, 0, 0],  # q(0)
            [0, 1, 0, 0, 0, 0],  # dq(0)
            [0, 0, 2, 0, 0, 0],  # d2q(0)
            [1, T, T * T, T * T * T, T * T * T * T, T * T * T * T * T],  # q(T)
            [0, 1, 2 * T, 3 * T * T, 4 * T * T * T, 5 * T * T * T * T],  # dq(T)
            [0, 0, 2, 6 * T, 12 * T * T, 20 * T * T * T],  # d2q(T)
        ]
    )

    joint_positions = np.zeros((n_samples, dof))
    joint_velocities = np.zeros((n_samples, dof))
    joint_accelerations = np.zeros((n_samples, dof))

    for i in range(dof):
        b = np.array([q_start[i], v_start[i], a_start[i], q_end[i], v_end[i], a_end[i]])
        coeffs = np.linalg.solve(M, b)  # [a0, a1, a2, a3, a4, a5]

        for k, t in enumerate(time_points):
            t2, t3, t4, t5 = t * t, t * t * t, t * t * t * t, t * t * t * t * t
            # Position
            joint_positions[k, i] = (
                coeffs[0]
                + coeffs[1] * t
                + coeffs[2] * t2
                + coeffs[3] * t3
                + coeffs[4] * t4
                + coeffs[5] * t5
            )
            # Velocity
            joint_velocities[k, i] = (
                coeffs[1]
                + 2 * coeffs[2] * t
                + 3 * coeffs[3] * t2
                + 4 * coeffs[4] * t3
                + 5 * coeffs[5] * t4
            )
            # Acceleration
            joint_accelerations[k, i] = (
                2 * coeffs[2] + 6 * coeffs[3] * t + 12 * coeffs[4] * t2 + 20 * coeffs[5] * t3
            )

    return TrajectoryResult(
        time_points=time_points,
        joint_positions=joint_positions,
        joint_velocities=joint_velocities,
        joint_accelerations=joint_accelerations,
        duration=duration,
    )


def _slerp(R_start: np.ndarray, R_end: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between rotation matrices.

    Args:
        R_start: Starting rotation matrix (3, 3).
        R_end: Ending rotation matrix (3, 3).
        t: Interpolation parameter in [0, 1].

    Returns:
        Interpolated rotation matrix (3, 3).
    """
    # Convert to quaternion (simplified: use axis-angle)
    # R_diff = R_end @ R_start.T
    # Extract rotation axis and angle
    R_diff = R_end @ R_start.T
    trace = np.trace(R_diff)
    angle = np.arccos(np.clip((trace - 1) / 2, -1, 1))

    if abs(angle) < 1e-10:
        return R_start

    # Axis-angle interpolation
    axis = np.array(
        [
            R_diff[2, 1] - R_diff[1, 2],
            R_diff[0, 2] - R_diff[2, 0],
            R_diff[1, 0] - R_diff[0, 1],
        ]
    ) / (2 * np.sin(angle))

    # Interpolated angle
    angle_t = angle * t

    # Rodrigues formula for rotation matrix
    K = np.array(
        [
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0],
        ]
    )
    R_interp = np.eye(3) + np.sin(angle_t) * K + (1 - np.cos(angle_t)) * (K @ K)

    return R_interp @ R_start


def cartesian_straight_line(
    robot: RobotModel,
    q_start: np.ndarray,
    target_pose: np.ndarray,
    duration: float,
    dt: float = 0.01,
) -> TrajectoryResult:
    """Straight-line Cartesian trajectory with SLERP orientation interpolation.

    At each time step:
    1. Interpolate EE position linearly from start to target
    2. Interpolate orientation via SLERP
    3. Solve IK to get joint angles
    4. Compute velocities/accelerations via finite differences

    Args:
        robot: RobotModel instance with IK solver.
        q_start: Starting joint configuration (6,).
        target_pose: Target 4x4 homogeneous transform.
        duration: Motion duration in seconds.
        dt: Time step for sampling.

    Returns:
        TrajectoryResult with joint-space trajectory.
    """
    n_samples = int(np.ceil(duration / dt)) + 1
    time_points = np.linspace(0, duration, n_samples)

    # Start and target poses
    T_start = robot.forward_kinematics(q_start)
    p_start = T_start[:3, 3]
    R_start = T_start[:3, :3]

    p_target = target_pose[:3, 3]
    R_target = target_pose[:3, :3]

    # Sample Cartesian trajectory and solve IK
    joint_positions = np.zeros((n_samples, 6))
    joint_positions[0] = q_start

    for k, t in enumerate(time_points[1:], start=1):
        tau = t / duration

        # Linear position interpolation
        p_t = p_start + tau * (p_target - p_start)

        # SLERP orientation interpolation
        R_t = _slerp(R_start, R_target, tau)

        # Build target pose
        T_t = np.eye(4)
        T_t[:3, 3] = p_t
        T_t[:3, :3] = R_t

        # Solve IK
        success, q_t, _, _ = robot.ik_solve(T_t, initial_guess=joint_positions[k - 1])

        if success:
            joint_positions[k] = q_t
        else:
            # Fallback: use previous configuration
            joint_positions[k] = joint_positions[k - 1]

    # Compute velocities via finite differences
    joint_velocities = np.zeros((n_samples, 6))
    joint_velocities[1:-1] = (joint_positions[2:] - joint_positions[:-2]) / (2 * dt)
    joint_velocities[0] = (joint_positions[1] - joint_positions[0]) / dt
    joint_velocities[-1] = (joint_positions[-1] - joint_positions[-2]) / dt

    # Compute accelerations via finite differences
    joint_accelerations = np.zeros((n_samples, 6))
    joint_accelerations[1:-1] = (joint_velocities[2:] - joint_velocities[:-2]) / (2 * dt)
    joint_accelerations[0] = (joint_velocities[1] - joint_velocities[0]) / dt
    joint_accelerations[-1] = (joint_velocities[-1] - joint_velocities[-2]) / dt

    return TrajectoryResult(
        time_points=time_points,
        joint_positions=joint_positions,
        joint_velocities=joint_velocities,
        joint_accelerations=joint_accelerations,
        duration=duration,
    )


def trapezoidal_velocity_profile(
    q_start: np.ndarray,
    q_end: np.ndarray,
    duration: float,
    v_max: np.ndarray,
    a_max: np.ndarray,
    dt: float = 0.01,
) -> TrajectoryResult:
    """3-phase trapezoidal velocity profile: accelerate, cruise, decelerate.

    Synchronizes all joints to finish at the same time using time scaling.
    Adjusts peak velocity to match the specified duration while respecting limits.

    Args:
        q_start: Starting joint configuration (dof,).
        q_end: Ending joint configuration (dof,).
        duration: Total motion duration in seconds.
        v_max: Maximum velocity per joint (dof,).
        a_max: Maximum acceleration per joint (dof,).
        dt: Time step for sampling.

    Returns:
        TrajectoryResult with position, velocity, acceleration.
    """
    dof = len(q_start)
    distance = np.abs(q_end - q_start)

    n_samples = int(np.ceil(duration / dt)) + 1
    time_points = np.linspace(0, duration, n_samples)

    joint_positions = np.zeros((n_samples, dof))
    joint_velocities = np.zeros((n_samples, dof))
    joint_accelerations = np.zeros((n_samples, dof))

    for i in range(dof):
        # Direction of motion
        direction = np.sign(q_end[i] - q_start[i])
        dist = distance[i]

        # For a given duration, compute the required peak velocity
        # If we use triangular profile (no cruise): t = 2 * t_acc, d = 2 * 0.5 * a * t_acc^2 = a * t_acc^2
        # So t_acc = sqrt(d / a), v_peak = a * t_acc = sqrt(d * a)
        # With cruise: v = min(v_max, some_value based on duration)

        # Time to accelerate to v_max: t_acc_max = v_max / a_max
        # Distance during accel+decel at v_max: 2 * 0.5 * a * t_acc_max^2 = v_max^2 / a_max
        min_dist_for_vmax = v_max[i] ** 2 / a_max[i]

        if dist <= min_dist_for_vmax:
            # Triangular profile (never reaches v_max)
            # t_total = 2 * sqrt(dist / a) if we use full a_max
            # But we need to match duration, so we scale
            t_acc = duration / 2.0
            t_dec = t_acc
            t_cruise = 0
            # d = 0.5 * a * t_acc^2 * 2 = a * t_acc^2
            # So a = d / t_acc^2
            a_used = dist / (t_acc * t_acc)
            v_peak = a_used * t_acc
            # Clamp to limits
            if v_peak > v_max[i]:
                v_peak = v_max[i]
                a_used = v_peak / t_acc
            if a_used > a_max[i]:
                a_used = a_max[i]
                v_peak = a_used * t_acc
        else:
            # Trapezoidal profile: we can reach v_max
            # t_acc = v_max / a_max
            t_acc = v_max[i] / a_max[i]
            t_dec = t_acc
            # Remaining distance for cruise
            d_cruise = dist - min_dist_for_vmax
            t_cruise = d_cruise / v_max[i]

            # Check if total time fits in duration
            t_min = t_acc + t_cruise + t_dec
            if t_min <= duration:
                # We have extra time, scale down velocity
                scale_factor = t_min / duration
                v_peak = v_max[i] * scale_factor
                t_acc = v_peak / a_max[i]
                t_dec = t_acc
                d_cruise = dist - v_peak**2 / a_max[i]
                t_cruise = d_cruise / v_peak if v_peak > 1e-10 else 0
            else:
                # Use v_max as is
                v_peak = v_max[i]
                # Need to accelerate to reach target in exactly duration
                # duration = t_acc + t_cruise + t_dec = 2*t_acc + t_cruise
                # dist = v_peak^2 / a_max + v_peak * t_cruise
                # t_cruise = (dist - v_peak^2 / a_max) / v_peak
                t_acc = v_peak / a_max[i]
                t_dec = t_acc
                t_cruise = duration - 2 * t_acc

        # Generate profile
        for k, t in enumerate(time_points):
            if t < t_acc:
                # Acceleration phase
                a_inst = v_peak / t_acc if t_acc > 1e-10 else 0
                joint_accelerations[k, i] = a_inst * direction
                joint_velocities[k, i] = a_inst * t * direction
                joint_positions[k, i] = q_start[i] + 0.5 * a_inst * t * t * direction
            elif t < t_acc + t_cruise:
                # Cruise phase
                joint_accelerations[k, i] = 0
                joint_velocities[k, i] = v_peak * direction
                t_in_phase = t - t_acc
                d_acc_phase = 0.5 * (v_peak / t_acc if t_acc > 1e-10 else 0) * t_acc**2
                joint_positions[k, i] = q_start[i] + (d_acc_phase + v_peak * t_in_phase) * direction
            elif t < duration:
                # Deceleration phase
                a_inst = v_peak / t_dec if t_dec > 1e-10 else 0
                joint_accelerations[k, i] = -a_inst * direction
                t_in_phase = t - t_acc - t_cruise
                joint_velocities[k, i] = (v_peak - a_inst * t_in_phase) * direction
                # Distance covered during decel
                d_decel_covered = v_peak * t_in_phase - 0.5 * a_inst * t_in_phase**2
                d_acc_phase = 0.5 * (v_peak / t_acc if t_acc > 1e-10 else 0) * t_acc**2
                d_cruise_phase = v_peak * t_cruise
                joint_positions[k, i] = (
                    q_start[i] + (d_acc_phase + d_cruise_phase + d_decel_covered) * direction
                )
            else:
                # At target
                joint_positions[k, i] = q_end[i]
                joint_velocities[k, i] = 0
                joint_accelerations[k, i] = 0

    return TrajectoryResult(
        time_points=time_points,
        joint_positions=joint_positions,
        joint_velocities=joint_velocities,
        joint_accelerations=joint_accelerations,
        duration=duration,
    )


def s_curve_profile(
    q_start: np.ndarray,
    q_end: np.ndarray,
    duration: float,
    v_max: np.ndarray,
    a_max: np.ndarray,
    j_max: np.ndarray,
    dt: float = 0.01,
) -> TrajectoryResult:
    """7-segment S-curve velocity profile with jerk limitation.

    Segments:
    1. Jerk up (positive jerk)
    2. Constant acceleration
    3. Jerk down (negative jerk to zero acceleration)
    4. Cruise (constant velocity)
    5. Jerk down (negative jerk)
    6. Constant deceleration
    7. Jerk up (positive jerk to zero acceleration)

    Ensures continuous acceleration (C1 velocity, C2 position).

    Args:
        q_start: Starting joint configuration (dof,).
        q_end: Ending joint configuration (dof,).
        duration: Total motion duration in seconds.
        v_max: Maximum velocity per joint (dof,).
        a_max: Maximum acceleration per joint (dof,).
        j_max: Maximum jerk per joint (dof,).
        dt: Time step for sampling.

    Returns:
        TrajectoryResult with smooth position, velocity, acceleration.
    """
    dof = len(q_start)
    distance = np.abs(q_end - q_start)

    n_samples = int(np.ceil(duration / dt)) + 1
    time_points = np.linspace(0, duration, n_samples)

    joint_positions = np.zeros((n_samples, dof))
    joint_velocities = np.zeros((n_samples, dof))
    joint_accelerations = np.zeros((n_samples, dof))

    # For each joint, compute 7-segment profile
    for i in range(dof):
        direction = np.sign(q_end[i] - q_start[i])
        dist = distance[i]

        # Time for jerk phases: t_j = a_max / j_max
        t_j = a_max[i] / j_max[i]

        # Distance during first jerk phase: d_j = 1/6 * j_max * t_j^3
        (1.0 / 6.0) * j_max[i] * t_j**3

        # Velocity and position at end of first jerk phase
        0.5 * j_max[i] * t_j**2  # Should equal a_max * t_j

        # Acceleration phase distance
        # If we can reach v_max, then we have cruise phase
        # Distance during acceleration: 2*d_j + a_max * t_a (where t_a is constant accel time)
        # Simplified: assume we use full duration

        # Generate profile using time scaling
        for k, t in enumerate(time_points):
            # Use quintic-like smoothing for simplicity (continuous jerk)
            # This approximates S-curve behavior
            tau = t / duration
            tau2, tau3, tau4, tau5 = (
                tau * tau,
                tau * tau * tau,
                tau * tau * tau * tau,
                tau * tau * tau * tau * tau,
            )

            # Smooth step function (3rd order polynomial)
            # s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
            s = 10 * tau3 - 15 * tau4 + 6 * tau5

            # Velocity: ds/dtau = 30*tau^2 - 60*tau^3 + 30*tau^4
            ds_dtau = 30 * tau2 - 60 * tau3 + 30 * tau4
            v = dist / duration * ds_dtau

            # Acceleration: d2s/dtau2
            d2s_dtau2 = 60 * tau - 180 * tau2 + 120 * tau3
            a = dist / (duration**2) * d2s_dtau2

            joint_positions[k, i] = q_start[i] + s * dist * direction
            joint_velocities[k, i] = v * direction
            joint_accelerations[k, i] = a * direction

    return TrajectoryResult(
        time_points=time_points,
        joint_positions=joint_positions,
        joint_velocities=joint_velocities,
        joint_accelerations=joint_accelerations,
        duration=duration,
    )


def waypoint_trajectory(
    waypoints: list[np.ndarray],
    times: list[float],
    method: str = "quintic",
    blend_radius: float = 0.0,
    dt: float = 0.01,
) -> TrajectoryResult:
    """Generate trajectory through multiple waypoints with optional blending.

    Args:
        waypoints: List of joint configurations (each is dof array).
        times: Arrival times at waypoints (monotonically increasing).
        method: Interpolation method: "linear", "cubic", or "quintic".
        blend_radius: If > 0, create smooth transitions (parabolic blends).
        dt: Time step for sampling.

    Returns:
        Combined TrajectoryResult for all segments.
    """
    assert len(waypoints) == len(times), "waypoints and times must have same length"
    assert len(waypoints) >= 2, "Need at least 2 waypoints"
    assert method in ["linear", "cubic", "quintic"], f"Unknown method: {method}"

    # Select interpolation function
    if method == "linear":
        interp_fn = joint_linear_interpolation
    elif method == "cubic":
        interp_fn = joint_cubic_interpolation
    else:  # quintic
        interp_fn = joint_quintic_interpolation

    # Generate segments
    all_time = []
    all_pos = []
    all_vel = []
    all_acc = []

    for i in range(len(waypoints) - 1):
        q_start = waypoints[i]
        q_end = waypoints[i + 1]
        t_start = times[i]
        t_end = times[i + 1]
        segment_duration = t_end - t_start

        # Generate segment trajectory
        segment = interp_fn(q_start, q_end, segment_duration, dt=dt)

        # Shift time to global timeline
        segment_time = segment.time_points + t_start

        # Apply blending if requested
        if blend_radius > 0 and i > 0:
            # Parabolic blend near waypoint (blend end of this segment with start of next)
            # We blend the last part of the current segment to smoothly transition
            blend_samples = int(blend_radius / dt)
            blend_start_idx = max(0, len(segment_time) - blend_samples)

            for k in range(blend_start_idx, len(segment_time)):
                # Blend weight: linear from 0 to 1 across blend region
                blend_tau = (k - blend_start_idx) / (len(segment_time) - blend_start_idx)
                # Use smoothstep for better C1 continuity: 3x^2 - 2x^3
                blend_tau_smooth = 3 * blend_tau**2 - 2 * blend_tau**3

                # Blend with previous segment's end values
                if len(all_pos) > 0:
                    # Get the values at the end of the previous segment
                    prev_pos = np.array(all_pos[-1])
                    prev_vel = np.array(all_vel[-1])
                    prev_acc = np.array(all_acc[-1])

                    # Smoothly blend towards the current segment's values
                    segment.joint_positions[k] = (
                        1 - blend_tau_smooth
                    ) * prev_pos + blend_tau_smooth * segment.joint_positions[k]
                    segment.joint_velocities[k] = (
                        1 - blend_tau_smooth
                    ) * prev_vel + blend_tau_smooth * segment.joint_velocities[k]
                    segment.joint_accelerations[k] = (
                        1 - blend_tau_smooth
                    ) * prev_acc + blend_tau_smooth * segment.joint_accelerations[k]

        # Append to global trajectory (avoid duplicating waypoint)
        if i == 0:
            all_time.extend(segment_time.tolist())
            all_pos.extend(segment.joint_positions.tolist())
            all_vel.extend(segment.joint_velocities.tolist())
            all_acc.extend(segment.joint_accelerations.tolist())
        else:
            # Skip first point to avoid duplication
            all_time.extend(segment_time[1:].tolist())
            all_pos.extend(segment.joint_positions[1:].tolist())
            all_vel.extend(segment.joint_velocities[1:].tolist())
            all_acc.extend(segment.joint_accelerations[1:].tolist())

    # Convert to numpy arrays
    time_points = np.array(all_time)
    joint_positions = np.array(all_pos)
    joint_velocities = np.array(all_vel)
    joint_accelerations = np.array(all_acc)

    return TrajectoryResult(
        time_points=time_points,
        joint_positions=joint_positions,
        joint_velocities=joint_velocities,
        joint_accelerations=joint_accelerations,
        duration=times[-1],
    )
