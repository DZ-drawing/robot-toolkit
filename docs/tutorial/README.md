# robot-toolkit Tutorials

Step-by-step tutorials for the robot-toolkit — a pure-Python 6-DOF inverse kinematics, dynamics, trajectory, collision, and path planning library.

## Quick Start

Install and verify:

```bash
pip install robot-ik
python -c "from robot_ik import six_dof_articulated; print('OK')"
```

## Tutorials by Difficulty

### Beginner

| # | Tutorial | Topics | Est. Time |
|---|----------|--------|-----------|
| [T0](./t0-getting-started.md) | Getting Started & URDF | Install, URDF import, FK, IK | 15 min |
| [T1](./t1-workspace-analysis.md) | Dual-Arm Workspace | FK sampling, 3D visualization, overlap | 20 min |
| [T2](./t2-collision-detection.md) | Collision Detection | CollisionChecker, primitives, safety checks | 20 min |

### Intermediate

| # | Tutorial | Topics | Est. Time |
|---|----------|--------|-----------|
| [T3](./t3-trajectory-planning.md) | Trajectory Planning | Multi-waypoint, S-curve, dual-arm sync | 25 min |
| [T4](./t4-path-planning-rrt.md) | RRT* Path Planning | Sampling-based planning, collision constraints | 25 min |
| [T5](./t5-dynamics.md) | Robot Dynamics | Inverse dynamics, gravity comp, inertia | 25 min |

### Advanced

| # | Tutorial | Topics | Est. Time |
|---|----------|--------|-----------|
| [T6](./t6-meshcat-visualization.md) | Meshcat 3D Visualization | Web-based real-time robot viz | 30 min |
| [T7](./t7-cartesian-path-collision.md) | Cartesian Path + Collision | Straight-line Cartesian, live collision check | 30 min |

## Module Coverage

| Module | Tutorials |
|--------|-----------|
| `ik_solver` | T0, T1, T2, T3, T4, T7 |
| `robot_dyn` | T5 |
| `trajectory` | T3, T7 |
| `collision` | T2, T4, T7 |
| `path_planning` | T4, T7 |
| `urdf_parser` | T0 |
| `visualize` (matplotlib) | T1, T2 |
| `visualize_meshcat` | T6 |
| `hardware` | T6 |

## Prerequisites

- Python 3.10+
- `numpy` (auto-installed)
- `matplotlib` (for T1, T2)
- `meshcat` (for T6)
- No hardware required — all tutorials run in software simulation

## Running

Each tutorial is a standalone Markdown file with runnable code blocks. Copy code into a `.py` file or Jupyter notebook to execute.

## Contributing

See [contributing guide](../contributing.md) for how to add new tutorials.
