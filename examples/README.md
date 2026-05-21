# Examples

Robotics project ideas and demos built with robot-toolkit. Each example demonstrates a real-world use case.

Some examples are **complete and runnable**, others are **work-in-progress** showing potential toolkit improvements and missing features needed for real applications.

## Running Examples

```bash
pip install -e .        # install from source
python examples/<name>.py
```

## Complete Examples

| Example | Description | Modules Used |
|---------|-------------|--------------|
| [solve_ik.py](./solve_ik.py) | Solve IK for a target position, verify with FK | ik_solver |
| [tutorial_ik.ipynb](./tutorial_ik.ipynb) | Interactive Jupyter walkthrough | ik_solver |

## Project Demos

| Example | Status | Description | Modules Used |
|---------|--------|-------------|--------------|
| [dual_arm_pick_place.py](./dual_arm_pick_place.py) | Draft | Dual-arm pick-and-place with collision checking | ik_solver, trajectory, collision, path_planning |
| [force_simulation.py](./force_simulation.py) | Draft | Simple force/torque simulation using dynamics solver | robot_dyn, trajectory |

## Contributing Examples

Feel free to add project demos:

1. Create `your_example.py` in this directory
2. Add a docstring explaining the project idea
3. Mark as **Draft** or **Complete** in this README
4. If it reveals a missing toolkit feature, add a `# TODO:` comment explaining what's needed

### Example Template

```python
"""
PROJECT NAME — Status: Draft/Complete

DESCRIPTION of the robotics project.

Modules used: ik_solver, collision, trajectory
Missing features: [list any toolkit improvements needed]
"""

import numpy as np
from robot_ik import ...

# TODO: implement
```
