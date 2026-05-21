# T5: Robot Dynamics — Inverse Dynamics, Gravity Compensation, and Inertia Analysis

**Difficulty:** Intermediate | **Time:** 25 min | **Module:** `robot_ik.robot_dyn`

This tutorial covers rigid body dynamics for a 6-DOF serial manipulator using the
`robot_ik.robot_dyn` module. You will learn to compute joint torques via inverse
dynamics, compensate for gravity, isolate Coriolis/centrifugal effects, and
analyze the joint-space inertia matrix.

## What You Will Learn

1. Create a `RobotDynamicsModel` with DH parameters and link inertias
2. Use `DynamicsSolver.inverse_dynamics()` for full torque computation
3. Compute gravity compensation torques
4. Isolate Coriolis and centrifugal torques
5. Compute and analyze the joint-space inertia matrix `H(q)`
6. Run forward dynamics simulation

## Prerequisites

```bash
pip install robot-ik numpy
```

```python
import numpy as np
```

---

## Step 1: Load the Dynamics Model

The `six_dof_articulated_dyn()` factory returns a pre-configured 6-DOF robot
with realistic mass, center-of-mass, and inertia tensor for each link.

```python
from robot_ik import six_dof_articulated_dyn, DynamicsSolver

model = six_dof_articulated_dyn()

# Inspect the model
print(f"Number of links: {len(model.links)}")
print(f"Gravity vector:  {model.gravity}")
print(f"Damping:         {model.joint_damping}")
print()

for i, link in enumerate(model.links):
    print(f"Link {i}: mass={link.mass:.1f} kg, "
          f"com={link.com.round(3)}, "
          f"I_diag={np.diag(link.inertia).round(4)}")
```

Each `LinkInertia` has three fields:

| Field | Type | Description |
|-------|------|-------------|
| `mass` | `float` | Link mass in kg |
| `com` | `np.ndarray (3,)` | Center of mass in link frame |
| `inertia` | `np.ndarray (3,3)` | Inertia tensor about COM in kg*m^2 |

## Step 2: Inverse Dynamics

Given joint positions `q`, velocities `qd`, and accelerations `qdd`, the
recursive Newton-Euler algorithm computes the joint torques `tau` required to
produce that motion.

```python
solver = DynamicsSolver(model)

# Joint state: all at zero
q = np.zeros(6)
qd = np.zeros(6)
qdd = np.zeros(6)

# Compute torques (at rest, this equals gravity compensation)
tau = solver.inverse_dynamics(q, qd, qdd)
print(f"Gravity torques at home pose:  {tau.round(3)} Nm")

# Now with joint acceleration on joint 2
qdd = np.array([0.0, 2.0, 0.0, 0.0, 0.0, 0.0])
tau_accel = solver.inverse_dynamics(q, qd, qdd)
print(f"Torques with qdd[1]=2:        {tau_accel.round(3)} Nm")
print(f"Incremental torque on joint 2: {tau_accel[1] - tau[1]:.3f} Nm")
```

### External Wrench

You can also pass an optional `external_wrench` to model forces/torques applied
at the end-effector:

```python
# 10N downward force at the end-effector
wrench = np.array([0, 0, -10, 0, 0, 0])  # [fx, fy, fz, nx, ny, nz]
tau_wrench = solver.inverse_dynamics(q, qd, qdd, external_wrench=wrench)
print(f"Torques with 10N payload: {tau_wrench.round(3)} Nm")
```

## Step 3: Gravity Compensation

Gravity compensation computes the torques needed to hold the robot stationary
against gravity. This is equivalent to calling `inverse_dynamics` with
`qd=0, qdd=0`.

```python
# Scan gravity torques across joint 1 range
q_scan = np.zeros(6)
print("Joint 1 angle | Gravity torques (Nm)")
print("-" * 50)
for angle in np.linspace(-np.pi, np.pi, 7):
    q_scan[0] = angle
    g_tau = solver.gravity_torque(q_scan)
    print(f"  {np.degrees(angle):+7.1f} deg  | {g_tau.round(3)}")
```

Gravity torques are largest when the arm is extended horizontally and smallest
when hanging straight down — this is what you should see in the output.

## Step 4: Coriolis and Centrifugal Torques

`coriolis_torque(q, qd)` isolates velocity-dependent torques by subtracting
gravity from the full inverse dynamics with `qdd=0`:

```
tau_coriolis = inverse_dynamics(q, qd, 0) - gravity_torque(q)
```

```python
# Fast rotation on joint 1
q = np.array([0.0, np.pi/4, 0, 0, 0, 0])
qd = np.array([3.0, 0, 0, 0, 0, 0])  # 3 rad/s on joint 1

c_tau = solver.coriolis_torque(q, qd)
g_tau = solver.gravity_torque(q)

print(f"Coriolis torques (q1 spinning at 3 rad/s): {c_tau.round(4)} Nm")
print(f"Gravity torques (same config):              {g_tau.round(4)} Nm")
print()
print("Notice: centrifugal coupling from joint 1 "
      "loads joints 2-6 even though only joint 1 is moving.")
```

## Step 5: Joint-Space Inertia Matrix

`inertia_matrix(q)` computes `H(q)` — the 6x6 symmetric positive-definite
mass matrix. Diagonal entries represent the effective inertia seen at each
joint; off-diagonal entries show coupling between joints.

```python
# Inertia at home (all zeros)
H_home = solver.inertia_matrix(np.zeros(6))

# Inertia with arm extended
q_extended = np.array([0.0, np.pi/2, 0, 0, 0, 0])
H_extended = solver.inertia_matrix(q_extended)

print("H(q) at home pose:")
print(np.array2string(H_home, precision=4, suppress_small=True))
print()
print("H(q) with shoulder raised 90 deg:")
print(np.array2string(H_extended, precision=4, suppress_small=True))

# Diagonal analysis
print()
print("Effective joint inertias:")
for name, H in [("Home", H_home), ("Extended", H_extended)]:
    diag = np.diag(H)
    print(f"  {name:8s}: {diag.round(4)}")
```

The inertia at joint 2 is significantly larger when the arm is extended
because the entire arm's mass must be accelerated. This is the reflected
inertia effect — important for motor sizing and control design.

### Manipulability from H(q)

The condition number of `H(q)` indicates how well-conditioned the inertia
matrix is. Poor conditioning means some directions are much harder to
accelerate than others:

```python
cond_home = np.linalg.cond(H_home)
cond_extended = np.linalg.cond(H_extended)
print(f"Condition number at home:      {cond_home:.1f}")
print(f"Condition number extended:     {cond_extended:.1f}")
```

## Step 6: Forward Dynamics

Given joint torques `tau`, forward dynamics computes the resulting joint
accelerations by solving:

```
H(q) * qdd = tau - C(q, qd) - G(q)
```

This uses the composite rigid body algorithm for `H(q)` and Newton-Euler for
the bias forces.

```python
q = np.array([0.0, np.pi/4, 0, 0, 0, 0])
qd = np.zeros(6)
tau = np.array([5.0, 0, 0, 0, 0, 0])  # Apply 5 Nm on joint 1

qdd = solver.forward_dynamics(q, qd, tau)
print(f"Applied torques:  {tau}")
print(f"Resulting accel:  {qdd.round(4)} rad/s^2")
print()

# Simple Euler integration for 1 second
q_sim = q.copy()
qd_sim = qd.copy()
dt = 0.01

for _ in range(int(1.0 / dt)):
    qdd = solver.forward_dynamics(q_sim, qd_sim, tau)
    qd_sim += qdd * dt
    q_sim += qd_sim * dt

print(f"After 1s simulation:")
print(f"  q  = {q_sim.round(3)} rad")
print(f"  qd = {qd_sim.round(3)} rad/s")
```

## Key Takeaways

| Method | Purpose | Signature |
|--------|---------|-----------|
| `inverse_dynamics` | Full Newton-Euler torque computation | `(q, qd, qdd, external_wrench=None)` |
| `gravity_torque` | Static gravity compensation | `(q)` |
| `coriolis_torque` | Velocity-dependent torques | `(q, qd)` |
| `inertia_matrix` | Joint-space mass matrix H(q) | `(q)` |
| `forward_dynamics` | Compute joint accelerations from torques | `(q, qd, tau)` |

## Next Steps

- **T6:** Visualize the robot in 3D with Meshcat
- **T7:** Plan Cartesian straight-line paths with collision checking
