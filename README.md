# robot-toolkit

[![CI](https://github.com/DZ-drawing/robot-toolkit/workflows/CI/badge.svg)](https://github.com/DZ-drawing/robot-toolkit/actions)
|[![codecov](https://codecov.io/gh/DZ-drawing/robot-toolkit/branch/main/graph/badge.svg)](https://codecov.io/gh/DZ-drawing/robot-toolkit)
|[![Python](https://img.shields.io/badge/python-3.10%2B-blue)](https://www.python.org/downloads/)
|[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

[English](README.md) | [中文](docs/zh/README.md)

Fast 6-DOF serial manipulator toolkit with IK, rigid body dynamics, trajectory planning, collision detection, RRT* path planning, and URDF parsing. C++ accelerated.

**Key features:**
- DH parameter forward kinematics
- Damped least-squares IK (Levenberg-Marquardt)
- Geometric Jacobian computation
- Rigid body dynamics (RNEA, CRBA)
- Trajectory planning (linear, cubic, quintic, trapezoidal, S-curve, Cartesian, waypoints)
- Collision detection (sphere, box, capsule)
- RRT* path planning
- URDF parser
- 3D visualization (matplotlib + meshcat)
- Hardware abstraction layer
- C++ extensions (137x faster IK, 358x faster dynamics)

## Quick Start

```bash
pip install -e .

# Run tests
pytest tests/ -v
```

## Usage

```python
from robot_ik import six_dof_articulated
import numpy as np

robot = six_dof_articulated()

# Define target pose as 4x4 homogeneous transform
target = np.array([
    [0, -1,  0, 0.5],
    [0,  0, -1, 0.2],
    [1,  0,  0, 0.4],
    [0,  0,  0, 1.0],
])

# Solve IK
success, joint_angles, iterations, errors = robot.ik_solve(target)
print(f"Solved in {iterations} iterations, angles: {joint_angles}")

# Verify
T = robot.forward_kinematics(joint_angles)
print(f"Position error: {np.linalg.norm(T[:3,3] - target[:3,3]):.6f} m")
```

## Custom Robot

```python
from robot_ik import RobotModel, DHParam
import numpy as np

my_robot = RobotModel([
    DHParam(a=0,   alpha=-np.pi/2, d=0.35, theta=0),
    DHParam(a=0.6, alpha=0,        d=0,    theta=0),
    DHParam(a=0.1, alpha=-np.pi/2, d=0,    theta=0),
    DHParam(a=0,   alpha=np.pi/2,  d=0.4,  theta=0),
    DHParam(a=0,   alpha=-np.pi/2, d=0,    theta=0),
    DHParam(a=0,   alpha=0,        d=0.08, theta=0),
], joint_limits=[(-3.14, 3.14)] * 6)
```

## Performance

| Metric | Value |
|--------|-------|
| Avg solve time | ~3 ms |
| P50 solve time | ~2 ms |
| P95 solve time | ~8 ms |
| Typical iterations | 5-15 |
| Position accuracy | <0.1 mm |
| Orientation accuracy | <0.001 rad |

Benchmarked on 6-DOF articulated robot, 200 random target poses.

## Project Structure

```
src/robot_ik/               # Source (src layout)
├── __init__.py             # Backward-compat re-exports
├── ik/                     # IK solver + C++ wrapper
├── dynamics/               # Rigid body dynamics
├── trajectory/             # Trajectory planning
├── collision/              # Collision detection
├── path_planning/          # RRT* path planning
├── urdf/                   # URDF parsing
├── visualization/          # matplotlib + meshcat
└── hardware/               # Hardware abstraction layer

tests/                       # Tests by module (67 tests)
csrc/                        # C++ extension sources
docs/tutorial/               # 8 standalone tutorials
examples/                    # Project demos
```

## C++ Extension (137x faster)

```bash
python setup.py build_ext --inplace
```

```python
from robot_ik.ik import FastIKSolver

solver = FastIKSolver(dh_params, joint_limits)
success, angles, iters, errors = solver.ik_solve(target_pose)
# Average: 0.09 ms (vs 12.6 ms pure Python)
```

| Metric | Python | C++ | Speedup |
|--------|--------|-----|---------|
| Avg solve | 12.6 ms | 0.09 ms | **137x** |
| P50 solve | 5.4 ms | 0.03 ms | 180x |
| P95 solve | 36.9 ms | 0.56 ms | 66x |

## License

MIT — see LICENSE file.
