[English](t2-collision-detection.md) | [中文](../zh/tutorial/t2-collision-detection.md)
# Tutorial 2: Self-Collision Detection

**Objective**: Detect collisions between a robot's own links and between the
robot and environment obstacles using primitive geometric shapes.

**Features demonstrated**:
- `CollisionChecker` with `Sphere`, `Capsule`, and `Box` primitives
- `add_link_geometry` and `add_obstacle` for building a collision world
- `check_self_collision` with adjacent-link filtering
- `check_environment_collision` for obstacle avoidance

---

## How it works

The `CollisionChecker` stores collision geometries (spheres, capsules, boxes)
 keyed by link name. At each configuration we provide the 4x4 homogeneous
transform of every link; the checker moves each primitive into world frame
and tests distances. Pairs whose gap is less than or equal to zero are
reported as collisions.

```python
"""Tutorial 2: Self-Collision Detection.

Set up a simple collision world for a 6-DOF robot, sweep through
configurations, and report any self-collisions or environment collisions.

Run with:  python -m docs.tutorial.t2_collision_detection
"""

import numpy as np
from robot_ik.collision import (
    CollisionChecker,
    CollisionResult,
    Sphere,
    Capsule,
    Box,
)


def build_collision_model(base_offset: np.ndarray = np.zeros(3)):
    """Build a CollisionChecker with capsule link geometries and a box obstacle.

    Args:
        base_offset: Translation applied to all link geometries (for arm2).

    Returns:
        (checker, link_names) tuple.
    """
    checker = CollisionChecker()
    # Approximate link lengths (m) for the 6-DOF articulated robot
    link_lengths = [0.0, 0.3, 0.5, 0.1, 0.4, 0.1]

    link_names = []
    for i in range(6):
        name = f"link{i}"
        link_names.append(name)
        # Each link is modelled as a capsule along the local Z axis
        checker.add_link_geometry(
            name,
            Capsule(
                p1=np.array([0.0, 0.0, 0.0]) + base_offset,
                p2=np.array([0.0, 0.0, link_lengths[i]]) + base_offset,
                radius=0.05,
            ),
        )

    # Add a table-like box obstacle at z = 0
    obstacle_pose = np.eye(4)
    obstacle_pose[:3, 3] = np.array([0.4, 0.0, -0.05])
    checker.add_obstacle(
        Box(size=np.array([0.8, 0.8, 0.1]), pose=obstacle_pose)
    )

    return checker, link_names


def make_link_transforms(
    q: np.ndarray,
    link_names: list[str],
    base_offset: np.ndarray = np.zeros(3),
) -> dict[str, np.ndarray]:
    """Build a dict of 4x4 link transforms from joint angles.

    This is a simplified placeholder; a real implementation would walk
    the DH chain. Each link frame is shifted along Z by the approximate
    link length, with a rotation about Z equal to the joint angle.

    Args:
        q: Joint angles (6,).
        link_names: List of link names.
        base_offset: World-frame offset for the base.

    Returns:
        Dict mapping link name -> 4x4 transform.
    """
    link_lengths = [0.0, 0.3, 0.5, 0.1, 0.4, 0.1]
    T = np.eye(4)
    T[:3, 3] = base_offset

    transforms: dict[str, np.ndarray] = {}
    for i, name in enumerate(link_names):
        # Rotate about Z by joint angle, then translate along Z by link length
        Rz = np.eye(4)
        c, s = np.cos(q[i]), np.sin(q[i])
        Rz[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
        T = T @ Rz
        transforms[name] = T.copy()
        T[:3, 3] += T[:2, 2].reshape(3)  # step along local Z
        # Approximate: just shift Z
        tz = np.eye(4)
        tz[2, 3] = link_lengths[i]
        T = T @ tz

    return transforms


def main() -> None:
    # Build collision model
    checker, link_names = build_collision_model()

    # Sweep through a grid of configurations and check for collisions
    n_steps = 20
    collision_count = 0

    for i in range(n_steps):
        for j in range(n_steps):
            q = np.zeros(6)
            q[0] = np.pi * (2 * i / n_steps - 1)  # base sweep
            q[1] = np.pi / 2 * (2 * j / n_steps - 1)  # shoulder sweep

            transforms = make_link_transforms(q, link_names)

            # Self-collision check (ignores adjacent links by default)
            sc: CollisionResult | None = checker.check_self_collision(
                transforms, ignore_adjacent=True
            )
            if sc is not None:
                collision_count += 1
                print(f"  Self-collision at q=[{q[0]:+.2f}, {q[1]:+.2f}, ...]  "
                      f"pair={sc.pair}  dist={sc.distance:.4f}")

            # Environment collision check
            ec: CollisionResult | None = checker.check_environment_collision(
                transforms
            )
            if ec is not None:
                print(f"  Env collision at q=[{q[0]:+.2f}, {q[1]:+.2f}, ...]  "
                      f"link={ec.pair[0]}  dist={ec.distance:.4f}")

    total = n_steps * n_steps
    print(f"\nChecked {total} configurations, "
          f"found {collision_count} self-collisions.")


if __name__ == "__main__":
    main()
```

## Learning outcomes

- Modelling robot links with collision primitives (capsules, spheres, boxes)
- Building a collision world with `CollisionChecker`
- Self-collision detection with adjacent-link filtering
- Environment (obstacle) collision detection
- Interpreting `CollisionResult` fields (`is_colliding`, `distance`, `contact_point`)

## Key API notes

| Concept | API |
|---|---|
| Create checker | `checker = CollisionChecker()` |
| Add link geometry | `checker.add_link_geometry(name, geometry)` |
| Add obstacle | `checker.add_obstacle(geometry)` |
| Self-collision | `checker.check_self_collision(link_transforms)` |
| Env collision | `checker.check_environment_collision(link_transforms)` |
| Result fields | `.is_colliding`, `.distance`, `.contact_point`, `.pair` |
