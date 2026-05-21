# Tutorial 0 — Getting Started with robot-ik

A step-by-step guide to installing robot-ik, building a robot model,
running forward and inverse kinematics, and loading a robot from URDF.

---

## 1. Install

```bash
pip install robot-ik
```

Or install from source:

```bash
git clone https://github.com/your-org/robot-toolkit.git
cd robot-toolkit
pip install -e .
```

---

## 2. Create a Robot Model

`six_dof_articulated()` returns a pre-configured 6-DOF industrial arm
with DH parameters, joint limits, and a spherical wrist — ready to use.

```python
import numpy as np
from robot_ik.ik_solver import six_dof_articulated

# Build the model
robot = six_dof_articulated()

print(type(robot))   # <class 'robot_ik.ik_solver.RobotModel'>
print(robot.dh_params)       # list of 6 DHParam objects
print(robot.joint_limits)    # [(min, max), ...] in radians
```

---

## 3. Forward Kinematics

Given six joint angles (radians), `forward_kinematics` returns the 4×4
homogeneous transform from base to end-effector.

```python
# Joint angles: [base, shoulder, elbow, wrist1, wrist2, wrist3]
q = np.array([0.0, 0.5, -0.3, 0.0, 0.2, 0.0])

T = robot.forward_kinematics(q)
print("End-effector position:", T[:3, 3])
print("End-effector rotation:\n", T[:3, :3])

# Get every link transform along the chain
T_ee, all_transforms = robot.forward_kinematics(q, return_all=True)
```

---

## 4. Inverse Kinematics

`ik_solve` finds joint angles that place the end-effector at a target pose.
It uses a damped least-squares (Levenberg-Marquardt) solver.

```python
# Define a target pose as a 4x4 homogeneous matrix
target = np.eye(4)
target[:3, 3] = [0.3, 0.4, 0.6]  # desired xyz position

# Run IK
success, q_sol, iterations, errors = robot.ik_solve(
    target_pose=target,
    initial_guess=None,        # defaults to all zeros
    max_iterations=200,
    position_tolerance=1e-4,    # meters
    orientation_tolerance=1e-3, # radians
    damping=0.1,
)

print("Converged:", success)
print("Solution (rad):", q_sol)
print("Iterations:", iterations)

# Verify: run FK on the solution
T_check = robot.forward_kinematics(q_sol)
print("FK position error:", np.linalg.norm(T_check[:3, 3] - target[:3, 3]))
```

---

## 5. Jacobian

The geometric Jacobian maps joint velocities to end-effector twist:

```python
J = robot.compute_jacobian(q)
print("Jacobian shape:", J.shape)  # (6, 6)
```

---

## 6. Load a Robot from URDF

`urdf_to_dynamics_model` parses a URDF file into a `RobotDynamicsModel`
with DH parameters and per-link inertia data.

```python
from robot_ik.urdf_parser import urdf_to_dynamics_model

dyn_model = urdf_to_dynamics_model("my_robot.urdf")
print("Links parsed:", len(dyn_model.links))
print("DH params (a):", dyn_model.dh_a)
```

### Sample URDF

Save this as `simple_arm.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_6dof">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <!-- Shoulder link -->
  <link name="shoulder_link">
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 0.15"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>
  <!-- Elbow link -->
  <link name="elbow_link">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.1"/>
      <inertia ixx="0.003" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.003"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.3"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="shoulder_link"/>
    <child link="elbow_link"/>
    <origin xyz="0 0.5 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57"/>
  </joint>
</robot>
```

```python
# Quick helper: get a DynamicsSolver directly from a URDF path
from robot_ik.urdf_parser import quick_urdf

solver = quick_urdf("simple_arm.urdf")
```

---

## 7. Next Steps

- **Tutorial 1** — Dynamics: forward/inverse dynamics, gravity compensation.
- **Tutorial 2** — Trajectory planning and motion profiles.
- **API Reference** — full docs for every class and function.
