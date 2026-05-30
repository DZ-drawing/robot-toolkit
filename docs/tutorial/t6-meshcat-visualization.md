[English](t6-meshcat-visualization.md) | [中文](../zh/tutorial/t6-meshcat-visualization.md)
# T6: Meshcat 3D Visualization — Web-Based Real-Time Robot Visualization

**Difficulty:** Advanced | **Time:** 30 min | **Module:** `robot_ik.visualize_meshcat`

This tutorial shows how to use `MeshcatVisualizer` for interactive 3D robot
visualization in the browser and Jupyter notebooks. You will set up the
visualizer, load a robot model, animate joint motions, and stream live
state from a hardware interface.

## What You Will Learn

1. Initialize `MeshcatVisualizer` and connect to the browser
2. Load a robot model with procedural 3D geometry
3. Update joint angles and see real-time motion
4. Embed the visualizer in a Jupyter notebook
5. Stream live joint states from a hardware interface

## Prerequisites

```bash
pip install robot-ik[meshcat]
# or: pip install meshcat
```

For Jupyter display:

```bash
pip install ipython
```

```python
import numpy as np
```

---

## Step 1: Create the Visualizer

`MeshcatVisualizer` launches a ZeroMQ-based web server. Open the URL in your
browser (Chrome recommended). It supports use as a context manager for
reliable cleanup.

```python
from robot_ik.visualize_meshcat import MeshcatVisualizer

# Create visualizer (starts server on port 7000 by default)
vis = MeshcatVisualizer(port=7000)
```

Or using the context manager (recommended for scripts):

```python
with MeshcatVisualizer(port=7000) as vis:
    # ... work with vis ...
    pass  # cleanup happens automatically
```

Open **http://127.0.0.1:7000/static/** in your browser. You should see an
empty 3D viewport with orbit controls.

### Connecting to an Existing Server

If you already have a Meshcat server running (e.g., from another process):

```python
vis = MeshcatVisualizer(zmq_url="tcp://127.0.0.1:6000")
```

## Step 2: Load a Robot Model

`set_robot()` generates procedural 3D geometry from the robot's DH parameters:

- **Base:** Box
- **Links 1-N:** Cylinders
- **Joints 1-N:** Spheres (gray)
- **Each joint frame:** RGB coordinate triad
- **End-effector:** RGB coordinate triad

```python
from robot_ik import six_dof_articulated

robot = six_dof_articulated()

# Load robot with default blue color
vis.set_robot(robot)

# Or with a custom RGBA color [R, G, B, A]
vis.set_robot(robot, color=[0.2, 0.8, 0.3, 1.0])  # green
```

Your browser should now show the full 6-DOF robot in its home pose.

### Robot Model Requirements

The robot must have:
- `dof` attribute or `dh_params` attribute (to infer number of joints)
- `forward_kinematics(q, return_all=True)` method returning `(pose, transforms)`
  where `transforms` is a list of 4x4 homogeneous matrices

## Step 3: Update Joint Angles

`update_joints(q)` computes forward kinematics and updates every link, joint,
and frame transform in the 3D scene:

```python
# Home position
vis.update_joints(np.zeros(6))

# Reach forward
vis.update_joints(np.array([0.0, np.pi/6, 0.0, 0.0, np.pi/4, 0.0]))

# Raised elbow
vis.update_joints(np.array([0.0, np.pi/3, -np.pi/4, 0.0, np.pi/6, 0.0]))
```

### Validation

The method validates the joint array shape and that a robot has been set:

```python
try:
    vis.update_joints(np.zeros(5))  # wrong size
except ValueError as e:
    print(f"Expected error: {e}")
```

## Step 4: Animate a Trajectory

Combine with `joint_cubic_interpolation` for smooth animation:

```python
import time
from robot_ik import joint_cubic_interpolation

q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q_end   = np.array([0.5, np.pi/4, -np.pi/6, 0.0, np.pi/3, 0.0])

traj = joint_cubic_interpolation(q_start, q_end, duration=3.0, dt=0.02)

print(f"Trajectory: {len(traj.time_points)} waypoints, "
      f"{traj.duration:.1f}s")

# Animate in real time
for i, t in enumerate(traj.time_points):
    vis.update_joints(traj.joint_positions[i])
    time.sleep(0.02)  # match the dt sampling rate
```

### Two-Arm Animation

For dual-arm setups, use separate MeshcatVisualizer instances with
different ports and namespace paths:

```python
vis_left  = MeshcatVisualizer(port=7000)
vis_right = MeshcatVisualizer(port=7001)

robot_left  = six_dof_articulated()
robot_right = six_dof_articulated()

vis_left.set_robot(robot_left)
vis_right.set_robot(robot_right)

# Mirror motion
for i in range(len(traj.time_points)):
    q = traj.joint_positions[i]
    vis_left.update_joints(q)
    vis_right.update_joints(q)  # or mirrored: q * [-1, 1, 1, -1, 1, 1]
    time.sleep(0.02)
```

## Step 5: Jupyter Notebook Display

`start_jupyter()` returns an `IFrame` that embeds the 3D view directly in a
notebook cell:

```python
# In a Jupyter notebook cell:
iframe = vis.start_jupyter()
display(iframe)  # renders inline at 800x600 pixels
```

The URL is auto-detected from Meshcat (supports both old `.url()` and new
`.viewer_url()` API).

## Step 6: Real-Time Hardware Streaming

`start_realtime_stream()` runs a background thread that polls a hardware
interface and updates the visualization continuously.

### Hardware Interface Contract

Your hardware class must implement:

```python
class MyHardware:
    def get_joint_positions(self) -> np.ndarray:
        """Return current joint angles as (6,) array."""
        # Read from serial, EtherCAT, etc.
        return np.array([q1, q2, q3, q4, q5, q6])
```

### Start/Stop Streaming

```python
hardware = MyHardware()

# Start at 30 Hz (default)
vis.start_realtime_stream(hardware, freq=30)

# The background thread runs automatically.
# Do other work here...

# Stop when done
vis.stop_realtime_stream()
```

The stream is thread-safe with a lock. Starting twice raises `RuntimeError`:

```python
try:
    vis.start_realtime_stream(hardware)
    vis.start_realtime_stream(hardware)  # error!
except RuntimeError as e:
    print(f"Expected: {e}")
```

### Context Manager Cleanup

When using `with MeshcatVisualizer() as vis:`, the stream is automatically
stopped on exit — even if an exception occurs:

```python
with MeshcatVisualizer() as vis:
    vis.set_robot(robot)
    vis.start_realtime_stream(hardware, freq=30)
    time.sleep(5.0)  # monitor for 5 seconds
# stream stopped automatically here
```

## Configuration Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | 7000 | Meshcat server port |
| `zmq_url` | None | ZeroMQ URL (auto-generated if None) |
| `DEFAULT_FREQ` | 30 | Default streaming frequency (Hz) |
| `BASE_SIZE` | [0.1, 0.1, 0.1] | Base box dimensions |
| `LINK_RADIUS` | 0.05 | Procedural link cylinder radius |
| `JOINT_RADIUS` | 0.06 | Procedural joint sphere radius |
| `DEFAULT_COLOR` | [0.3, 0.6, 0.9, 1.0] | Default link color (RGBA) |

## Error Handling

| Exception | When |
|-----------|------|
| `InitializationError` | meshcat not installed, or server fails to start |
| `MeshcatError` | Robot not set, FK fails, model creation fails |
| `StreamingError` | Hardware interface lacks `get_joint_positions()` |
| `ValueError` | Wrong joint array shape |

## Key Takeaways

| Method | Purpose |
|--------|---------|
| `MeshcatVisualizer(port)` | Create visualizer with web server |
| `set_robot(robot, color)` | Load robot with procedural 3D geometry |
| `update_joints(q)` | Update all joint transforms via FK |
| `start_jupyter()` | Get IFrame for notebook embedding |
| `start_realtime_stream(hw, freq)` | Start background hardware polling |
| `stop_realtime_stream()` | Stop background polling |

## Next Steps

- **T7:** Combine Cartesian trajectory planning with collision checking
