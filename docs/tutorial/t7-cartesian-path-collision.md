[English](t7-cartesian-path-collision.md) | [中文](../zh/tutorial/t7-cartesian-path-collision.md)
# T7: Cartesian Straight-Line Path with Collision Checking

**Difficulty:** Advanced | **Time:** 30 min | **Modules:** `robot_ik.trajectory`, `robot_ik.collision`

This tutorial combines Cartesian straight-line trajectory planning with live
collision checking. You will generate a task-space straight-line path with
SLERP orientation interpolation, build a collision environment with obstacles,
and validate every waypoint against self-collision and environment collision.

## What You Will Learn

1. Generate Cartesian straight-line trajectories with `cartesian_straight_line`
2. Build a collision scene with `CollisionChecker`, `Sphere`, `Capsule`, and `Box`
3. Check every trajectory waypoint for collisions
4. Classify collisions as self-collision vs. environment collision
5. Plot the trajectory and collision results

## Prerequisites

```bash
pip install robot-ik numpy matplotlib
```

```python
import numpy as np
import matplotlib.pyplot as plt
```

---

## Step 1: Generate a Cartesian Straight-Line Trajectory

`cartesian_straight_line` plans a straight-line motion in Cartesian space
from a starting joint configuration to a target pose (4x4 homogeneous
transform). Position is interpolated linearly; orientation uses SLERP
(spherical linear interpolation). At each step, IK is solved to get the
corresponding joint angles.

```python
from robot_ik import six_dof_articulated, cartesian_straight_line

robot = six_dof_articulated()

# Starting configuration
q_start = np.array([0.0, np.pi/6, 0.0, 0.0, np.pi/4, 0.0])

# Verify the starting end-effector pose
T_start = robot.forward_kinematics(q_start)
print("Start pose (translation):")
print(f"  x={T_start[0,3]:.3f}, y={T_start[1,3]:.3f}, z={T_start[2,3]:.3f}")

# Define target pose (shifted 0.15m in x, 0.1m in z)
T_target = T_start.copy()
T_target[0, 3] += 0.15  # move forward in x
T_target[2, 3] += 0.10  # move up in z

print("Target pose (translation):")
print(f"  x={T_target[0,3]:.3f}, y={T_target[1,3]:.3f}, z={T_target[2,3]:.3f}")

# Generate trajectory (3 seconds, 50 Hz)
traj = cartesian_straight_line(
    robot=robot,
    q_start=q_start,
    target_pose=T_target,
    duration=3.0,
    dt=0.02,
)

print(f"\nTrajectory generated:")
print(f"  Waypoints:  {len(traj.time_points)}")
print(f"  Duration:   {traj.duration:.1f} s")
print(f"  dt:         {traj.time_points[1] - traj.time_points[0]:.3f} s")
```

The `TrajectoryResult` contains:

| Field | Shape | Description |
|-------|-------|-------------|
| `time_points` | `(N,)` | Timestamps in seconds |
| `joint_positions` | `(N, 6)` | Joint angles in radians |
| `joint_velocities` | `(N, 6)` | Joint velocities (rad/s) |
| `joint_accelerations` | `(N, 6)` | Joint accelerations (rad/s^2) |
| `duration` | scalar | Total duration in seconds |

### Plot the Joint Trajectory

```python
fig, axes = plt.subplots(6, 1, figsize=(10, 8), sharex=True)
labels = [f"Joint {i}" for i in range(6)]

for i in range(6):
    axes[i].plot(traj.time_points, np.degrees(traj.joint_positions[:, i]))
    axes[i].set_ylabel(labels[i])
    axes[i].grid(True, alpha=0.3)

axes[-1].set_xlabel("Time (s)")
fig.suptitle("Cartesian Straight-Line: Joint Trajectory")
plt.tight_layout()
plt.savefig("t7_joint_trajectory.png", dpi=150)
plt.close()
```

### Plot the Cartesian Path

```python
# Compute end-effector positions along the trajectory
ee_positions = []
for i in range(len(traj.time_points)):
    T = robot.forward_kinematics(traj.joint_positions[i])
    ee_positions.append(T[:3, 3])

ee_positions = np.array(ee_positions)

fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")
ax.plot(ee_positions[:, 0], ee_positions[:, 1], ee_positions[:, 2],
        "b.-", linewidth=1.5, label="EE path")
ax.scatter(*ee_positions[0],  color="green", s=100, label="Start")
ax.scatter(*ee_positions[-1], color="red",   s=100, label="End")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("End-Effector Cartesian Path")
ax.legend()
plt.tight_layout()
plt.savefig("t7_cartesian_path.png", dpi=150)
plt.close()
```

## Step 2: Set Up the Collision Checker

Create a `CollisionChecker` and add collision geometry for robot links and
environment obstacles.

```python
from robot_ik import CollisionChecker, Sphere, Capsule, Box

checker = CollisionChecker()
```

### Add Link Geometries

For each link, add collision primitives positioned in the link's local frame.
`add_link_geometry` accepts multiple geometries per link:

```python
# Joint spheres at each joint position
for i in range(6):
    T_joint = robot.forward_kinematics(q_start, return_all=True)[1][i + 1]
    checker.add_link_geometry(
        f"joint{i}",
        Sphere(radius=0.06, pose=T_joint.copy()),
    )

# Link capsules between consecutive joint origins
transforms = robot.forward_kinematics(q_start, return_all=True)[1]
for i in range(5):
    p1 = transforms[i + 1][:3, 3]
    p2 = transforms[i + 2][:3, 3]
    checker.add_link_geometry(
        f"link{i}",
        Capsule(p1=p1, p2=p2, radius=0.04),
    )
```

### Add Environment Obstacles

Add static obstacles in the world frame using `add_obstacle`:

```python
# Table surface (a box below the robot)
table = Box(
    size=np.array([0.8, 0.8, 0.05]),
    pose=np.array([
        [1, 0, 0, 0.0],
        [0, 1, 0, 0.0],
        [0, 0, 1, -0.35],
        [0, 0, 0, 1],
    ]),
)
checker.add_obstacle(table)

# Vertical pillar (obstacle in the workspace)
pillar = Box(
    size=np.array([0.08, 0.08, 0.6]),
    pose=np.array([
        [1, 0, 0, 0.45],
        [0, 1, 0, 0.0],
        [0, 0, 1, -0.1],
        [0, 0, 0, 1],
    ]),
)
checker.add_obstacle(pillar)

print(f"Links with geometry: {list(checker.link_geometries.keys())}")
print(f"Obstacles:           {len(checker.obstacles)}")
```

## Step 3: Check Collision at Each Waypoint

Build a function that validates a single joint configuration against both
self-collision and environment collision:

```python
def check_waypoint(q, checker, robot, ignore_adjacent=True):
    """Check a single configuration for collisions.

    Returns:
        dict with 'self_collision', 'env_collision', 'safe' keys.
    """
    # Compute all link transforms
    _, transforms = robot.forward_kinematics(q, return_all=True)

    link_transforms = {}
    for i in range(6):
        link_transforms[f"joint{i}"] = transforms[i + 1]
    for i in range(5):
        link_transforms[f"link{i}"] = transforms[i + 1]

    # Self-collision check
    self_result = checker.check_self_collision(
        link_transforms, ignore_adjacent=ignore_adjacent
    )

    # Environment collision check
    env_result = checker.check_environment_collision(link_transforms)

    return {
        "safe": self_result is None and env_result is None,
        "self_collision": self_result,
        "env_collision": env_result,
    }
```

Now scan the entire trajectory:

```python
safe_count = 0
self_col_count = 0
env_col_count = 0
collision_points = []

print("Checking trajectory waypoints for collisions...")
print(f"{'Waypoint':>8}  {'Time':>6}  {'Status':>12}  {'Details'}")
print("-" * 60)

for i in range(len(traj.time_points)):
    q = traj.joint_positions[i]
    result = check_waypoint(q, checker, robot)

    if result["safe"]:
        safe_count += 1
        status = "SAFE"
        detail = ""
    elif result["self_collision"]:
        self_col_count += 1
        status = "SELF-COL"
        pair = result["self_collision"].pair
        detail = f"{pair[0]} <-> {pair[1]}"
        collision_points.append(i)
    elif result["env_collision"]:
        env_col_count += 1
        status = "ENV-COL"
        detail = f"{result['env_collision'].pair[0]} <-> obstacle"
        collision_points.append(i)
    else:
        safe_count += 1
        status = "SAFE"
        detail = ""

    # Print every 25th waypoint or collisions
    if i % 25 == 0 or not result["safe"]:
        t = traj.time_points[i]
        print(f"{i:>8d}  {t:>6.2f}s  {status:>12}  {detail}")

print()
print(f"Summary: {safe_count} safe, "
      f"{self_col_count} self-collisions, "
      f"{env_col_count} environment collisions")
print(f"Total waypoints: {len(traj.time_points)}")
```

## Step 4: Visualize Collision Results

Plot the Cartesian path with collision points highlighted:

```python
fig = plt.figure(figsize=(10, 6))

# Position subplot
ax1 = fig.add_subplot(121, projection="3d")
safe_mask = np.ones(len(ee_positions), dtype=bool)
for idx in collision_points:
    safe_mask[idx] = False

# Safe portion in blue
ax1.plot(ee_positions[safe_mask, 0],
         ee_positions[safe_mask, 1],
         ee_positions[safe_mask, 2],
         "b-", linewidth=1.5, label="Safe")

# Collision points in red
if len(collision_points) > 0:
    col_pos = ee_positions[collision_points]
    ax1.scatter(col_pos[:, 0], col_pos[:, 1], col_pos[:, 2],
                color="red", s=50, label="Collision", zorder=5)

# Obstacles
pillar_pos = pillar.pose[:3, 3]
ax1.scatter(*pillar_pos, color="orange", s=200, marker="s",
            label="Pillar")

ax1.set_xlabel("X (m)")
ax1.set_ylabel("Y (m)")
ax1.set_zlabel("Z (m)")
ax1.set_title("Path with Collision Check")
ax1.legend()

# Joint space subplot
ax2 = fig.add_subplot(122)
for i in range(6):
    color = "blue" if all(idx not in collision_points for idx in range(len(traj.time_points)))
    ax2.plot(traj.time_points,
             np.degrees(traj.joint_positions[:, i]),
             label=f"J{i}")

for idx in collision_points:
    ax2.axvline(traj.time_points[idx], color="red", alpha=0.3, linewidth=0.5)

ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Joint angle (deg)")
ax2.set_title("Joint Trajectory (red = collision)")
ax2.legend(fontsize=8)
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("t7_collision_results.png", dpi=150)
plt.close()
```

## Step 5: Collision Margin and Safety Threshold

The `_check_geometry_collision` method uses a `collision_threshold` parameter
(default 0.0). You can use a positive threshold to trigger warnings before
actual contact — useful for safety margins:

```python
# Check with a 2cm safety margin
# (This requires calling _check_geometry_collision directly or
#  modifying the collision_threshold in a subclass)

# Simple approach: check distance without threshold by inspecting
# the distance field on CollisionResult
def min_clearance(q, checker, robot):
    """Compute minimum clearance to any collision."""
    _, transforms = robot.forward_kinematics(q, return_all=True)
    link_transforms = {}
    for i in range(6):
        link_transforms[f"joint{i}"] = transforms[i + 1]
    for i in range(5):
        link_transforms[f"link{i}"] = transforms[i + 1]

    min_dist = float("inf")
    self_result = checker.check_self_collision(link_transforms)
    env_result = checker.check_environment_collision(link_transforms)

    if self_result is not None:
        min_dist = min(min_dist, self_result.distance)
    if env_result is not None:
        min_dist = min(min_dist, env_result.distance)

    return min_dist

# Check clearance at a few waypoints
print("Clearance analysis:")
safety_margin = 0.02  # 2 cm
for idx in [0, len(traj.time_points)//4, len(traj.time_points)//2,
            3*len(traj.time_points)//4, -1]:
    q = traj.joint_positions[idx]
    d = min_clearance(q, checker, robot)
    t = traj.time_points[idx]
    status = "OK" if d > safety_margin else "WARNING"
    print(f"  t={t:.2f}s: clearance={d:.4f}m  [{status}]")
```

## Step 6: Putting It All Together — Safe Path Planning

Combine trajectory generation, collision checking, and re-planning in a
complete workflow:

```python
from robot_ik import cartesian_straight_line

def plan_safe_cartesian_path(robot, checker, q_start, T_target,
                              duration=3.0, dt=0.02):
    """Generate Cartesian path and validate against collisions.

    Returns:
        tuple of (trajectory, collision_indices) or (None, []) if unsafe.
    """
    traj = cartesian_straight_line(robot, q_start, T_target, duration, dt)

    collision_indices = []
    for i in range(len(traj.time_points)):
        result = check_waypoint(traj.joint_positions[i], checker, robot)
        if not result["safe"]:
            collision_indices.append(i)

    if len(collision_indices) > 0:
        print(f"WARNING: {len(collision_indices)}/{len(traj.time_points)} "
              f"waypoints in collision")
    else:
        print(f"Path validated: all {len(traj.time_points)} waypoints safe")

    return traj, collision_indices


# Plan the path
traj, collisions = plan_safe_cartesian_path(
    robot, checker, q_start, T_target, duration=3.0
)

# If collisions found, you could:
# 1. Re-plan with a different target pose
# 2. Use RRT* path planning (T4) to find a collision-free path
# 3. Add intermediate waypoints to avoid obstacles
```

## Geometry Reference

The collision module provides three primitive types:

| Type | Constructor | Parameters |
|------|-------------|------------|
| `Sphere` | `Sphere(radius, pose)` | `radius` in m, `pose` as 4x4 |
| `Capsule` | `Capsule(p1, p2, radius, pose)` | Line segment endpoints, `radius` in m |
| `Box` | `Box(size, pose)` | `size` as (x,y,z), `pose` as 4x4 |

Supported pair-wise distance functions:

- `distance_sphere_to_sphere`
- `distance_sphere_to_capsule`
- `distance_capsule_to_capsule`
- `distance_sphere_to_box`
- `distance_box_to_box`
- `distance_point_to_sphere`
- `distance_point_to_box`

## Key Takeaways

| Function | Purpose |
|----------|---------|
| `cartesian_straight_line(robot, q_start, target_pose, duration)` | Plan Cartesian straight-line with SLERP |
| `CollisionChecker()` | Create collision checking environment |
| `checker.add_link_geometry(name, geom)` | Attach collision shape to a link |
| `checker.add_obstacle(geom)` | Add environment obstacle |
| `checker.check_self_collision(transforms)` | Check link-link collisions |
| `checker.check_environment_collision(transforms)` | Check link-obstacle collisions |

## Next Steps

- **T4:** Use RRT* for collision-free path planning around obstacles
- **T5:** Add dynamics analysis to validate torque limits along the trajectory
- **T6:** Visualize the collision-free path in 3D with Meshcat
