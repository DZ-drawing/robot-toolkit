[English](collision.md) | [中文](zh/collision.md)

# Collision Detection

## Overview

robot-toolkit provides a three-tier collision detection system for robot manipulators:

| Tier | Method | Use Case | Performance |
|------|--------|----------|-------------|
| **Tier 1** | Analytical formulas | Primitive–primitive (sphere, capsule, box) | O(1) per pair |
| **Tier 2** | GJK / EPA | Any pair involving `TriangleMesh` | Iterative, typically 8–32 iterations |
| **Tier 3** | `CollisionChecker` | Full-robot dispatch (auto-selects Tier 1 or 2) | Depends on geometry types |

- **Tier 1** uses closed-form distance formulas for common primitive shapes.
- **Tier 2** uses the GJK algorithm for intersection/distance queries and EPA for
  penetration depth and contact information when shapes overlap.
- **Tier 3** is the unified `CollisionChecker` that automatically dispatches to the
  appropriate tier based on the geometry types of the input pair.

---

## TriangleMesh

`TriangleMesh` is a convex triangle mesh dataclass that serves as the primary
geometry representation for the GJK/EPA pipeline. It carries vertex positions,
triangle face indices, a 4x4 pose transform, and an optional name.

```python
from robot_ik.collision import TriangleMesh
import numpy as np
```

### Data Fields

| Field | Type | Description |
|-------|------|-------------|
| `vertices` | `np.ndarray (N, 3)` | Vertex positions in local frame |
| `faces` | `np.ndarray (M, 3, int)` | Triangle face indices into `vertices` |
| `pose` | `np.ndarray (4, 4)` | Homogeneous transform (local → world) |
| `name` | `str` | Identifier for the mesh |

### Support Function

The **support function** returns the farthest vertex in a given direction. This
is the core operation consumed by GJK. It correctly handles rotation by
transforming the query direction into the local frame before searching.

```python
mesh = TriangleMesh.from_box([1.0, 1.0, 1.0])
direction = np.array([1.0, 0.0, 0.0])
extreme = mesh.support(direction)  # farthest vertex in +X
```

### Updating Pose

```python
T = np.eye(4)
T[:3, 3] = [0.5, 0.0, 0.0]  # translate 0.5 m along X
mesh.update_pose(T)
```

### Factory Methods

#### `from_box(size, pose=None, name="box")`

Creates a box with 8 vertices and 12 triangular faces.

```python
mesh = TriangleMesh.from_box([0.6, 0.4, 0.2])
```

#### `from_sphere(radius=1.0, subdivisions=1, pose=None, name="sphere")`

Creates an icosphere approximation. `subdivisions=0` yields 20 faces;
each level quadruples the face count.

```python
mesh = TriangleMesh.from_sphere(radius=0.05, subdivisions=2)
```

#### `from_capsule(p1, p2, radius=0.05, subdivisions=1, pose=None, name="capsule")`

Creates a capsule mesh via convex hull of generated surface points.

```python
mesh = TriangleMesh.from_capsule(
    p1=np.array([0, 0, 0]),
    p2=np.array([0, 0, 0.3]),
    radius=0.04,
)
```

#### `from_convex_hull(points, pose=None, name="convex_hull")`

Creates a mesh from the convex hull of an arbitrary point cloud using
`scipy.spatial.ConvexHull`.

```python
points = np.random.randn(50, 3)  # random point cloud
mesh = TriangleMesh.from_convex_hull(points)
```

---

## GJK Algorithm

The **Gilbert–Johnson–Keerthi (GJK)** algorithm operates on the Minkowski
difference of two convex shapes. Two shapes intersect if and only if the
origin lies inside (or on the boundary of) their Minkowski difference.

```python
from robot_ik.collision import gjk_intersect, gjk_distance
```

### `gjk_intersect(shape_a, shape_b, max_iterations=64) -> bool`

Returns `True` if two convex shapes intersect. Internally builds a simplex
in the Minkowski difference and iteratively refines it toward the origin.

```python
box = TriangleMesh.from_box([1.0, 1.0, 1.0])
sphere = TriangleMesh.from_sphere(radius=0.5)

overlapping = gjk_intersect(box, sphere)
print(f"Overlapping: {overlapping}")
```

### `gjk_distance(shape_a, shape_b, max_iterations=64) -> tuple`

Returns `(distance, closest_on_a, closest_on_b)`:
- `distance`: Euclidean separation distance (0.0 if overlapping).
- `closest_on_a`, `closest_on_b`: 3D witness points in world frame, or `None`
  if the shapes overlap.

```python
box = TriangleMesh.from_box([1.0, 1.0, 1.0])
box.update_pose(np.eye(4))  # at origin

sphere = TriangleMesh.from_sphere(radius=0.3)
T = np.eye(4)
T[:3, 3] = [1.5, 0.0, 0.0]  # offset along X
sphere.update_pose(T)

dist, pt_a, pt_b = gjk_distance(box, sphere)
print(f"Distance: {dist:.4f}")
print(f"Closest on box:  {pt_a}")
print(f"Closest on sphere: {pt_b}")
```

---

## EPA Algorithm

The **Expanding Polytope Algorithm (EPA)** computes penetration depth, collision
normal, and contact point when two convex shapes are overlapping. It builds on
the GJK result by expanding the simplex into a polytope that approximates the
Minkowski difference boundary.

```python
from robot_ik.collision import epa_penetration
```

### `epa_penetration(shape_a, shape_b, max_iterations=64, tolerance=1e-6) -> tuple`

Returns `(depth, normal, contact_point)`:
- `depth` (≥ 0): penetration depth.
- `normal`: unit collision normal (from B toward A).
- `contact_point`: world-frame contact point (midpoint of closest witness pair).

```python
box = TriangleMesh.from_box([1.0, 1.0, 1.0])
sphere = TriangleMesh.from_sphere(radius=0.6)

depth, normal, contact = epa_penetration(box, sphere)
print(f"Penetration depth: {depth:.4f}")
print(f"Collision normal:  {normal}")
print(f"Contact point:     {contact}")
```

> **Note:** EPA internally calls `gjk_intersect` first. If the shapes do not
> overlap, it returns `(0.0, [1,0,0], [0,0,0])`.

---

## CollisionChecker Integration

`CollisionChecker` is the unified entry point that manages robot link geometries
and environment obstacles, automatically dispatching to the appropriate collision
method.

```python
from robot_ik.collision import CollisionChecker, Sphere, Capsule, Box, TriangleMesh
```

### Three-Tier Dispatch Logic

```
_check_geometry_collision(g1, g2)
├── If either is TriangleMesh → Tier 2: GJK/EPA (_check_pair_mesh)
└── Otherwise                  → Tier 1: Analytical   (_check_pair_primitive)
```

When Tier 2 is selected, primitive geometries are automatically converted to
`TriangleMesh` via their `.to_mesh()` methods before running GJK/EPA.

### Setting Up a Robot

```python
checker = CollisionChecker()

# Add link geometries (primitives)
checker.add_link_geometry("link1", Sphere(radius=0.05))
checker.add_link_geometry("link2", Capsule(
    p1=np.array([0, 0, 0]),
    p2=np.array([0, 0, 0.3]),
    radius=0.04,
))

# Add an environment obstacle (mesh)
obstacle = TriangleMesh.from_box([2.0, 2.0, 0.1])
obstacle.update_pose(np.eye(4))  # on the ground plane
checker.add_obstacle(obstacle)
```

### Self-Collision Check

```python
link_transforms = {
    "link1": np.eye(4),
    "link2": np.eye(4),
}

result = checker.check_self_collision(link_transforms, ignore_adjacent=True)
if result is not None:
    print(f"Self-collision: {result.pair}, depth={-result.distance:.4f}")
else:
    print("No self-collision")
```

### Environment Collision Check

```python
result = checker.check_environment_collision(link_transforms)
if result is not None:
    print(f"Environment collision: {result.pair}")
```

### `to_mesh()` Convenience

Every primitive (`Sphere`, `Capsule`, `Box`) provides a `.to_mesh()` method for
manual Tier 2 queries:

```python
sphere = Sphere(radius=0.05)
sphere.pose = np.eye(4)
mesh = sphere.to_mesh()  # TriangleMesh (icosphere)

box = Box(size=np.array([0.1, 0.1, 0.1]))
box_mesh = box.to_mesh()  # TriangleMesh (12 triangles)
```

### CollisionResult

| Field | Type | Description |
|-------|------|-------------|
| `is_colliding` | `bool` | Whether the pair is in collision |
| `distance` | `float` | Signed distance (negative if penetrating) |
| `contact_point` | `np.ndarray or None` | Contact point in world frame |
| `pair` | `tuple[str, str]` | Names of the colliding pair |

---

## API Reference

### Core Functions

| Function | Signature | Returns |
|----------|-----------|---------|
| `gjk_intersect` | `(TriangleMesh, TriangleMesh, max_iter=64)` | `bool` |
| `gjk_distance` | `(TriangleMesh, TriangleMesh, max_iter=64)` | `(float, ndarray, ndarray)` |
| `epa_penetration` | `(TriangleMesh, TriangleMesh, max_iter=64, tol=1e-6)` | `(float, ndarray, ndarray)` |

### Primitive Distance Functions

| Function | Pair |
|----------|------|
| `distance_sphere_to_sphere` | Sphere ↔ Sphere |
| `distance_sphere_to_capsule` | Sphere ↔ Capsule |
| `distance_capsule_to_capsule` | Capsule ↔ Capsule |
| `distance_sphere_to_box` | Sphere ↔ Box |
| `distance_box_to_box` | Box ↔ Box |
| `distance_point_to_sphere` | Point ↔ Sphere |
| `distance_point_to_box` | Point ↔ Box |

### Classes

| Class | Description |
|-------|-------------|
| `TriangleMesh` | Convex triangle mesh with GJK support function |
| `Sphere` | Sphere primitive with `.to_mesh()` |
| `Capsule` | Capsule primitive with `.to_mesh()` |
| `Box` | Box primitive with `.to_mesh()` |
| `CollisionChecker` | Unified collision dispatcher |
| `CollisionResult` | Collision result dataclass |

---

## Performance Characteristics

| Operation | Complexity | Typical Iterations |
|-----------|-----------|-------------------|
| Sphere–Sphere (Tier 1) | O(1) | 1 |
| Capsule–Capsule (Tier 1) | O(1) | 1 |
| GJK intersect (Tier 2) | O(k·n) | 4–16 |
| GJK distance (Tier 2) | O(k·n) | 8–32 |
| EPA penetration (Tier 2) | O(k·n²) | 8–32 |

Where `k` = iteration count (bounded by `max_iterations`), `n` = vertex count
of the mesh. The support function is O(n) per call for a naive implementation
(vertex scan), giving GJK an overall cost of O(k·n). EPA has an additional cost
for polytope face management, roughly O(k·n²) in the worst case due to BFS
horizon computation.

**Tips for performance:**
- Use Tier 1 primitives when possible — they are significantly faster than GJK.
- Keep mesh vertex counts low. A box (8 vertices) is much cheaper than a highly
  subdivided sphere.
- Use `subdivisions=0` or `1` for sphere/capsule meshes unless you need high
  precision.
- Cache mesh conversions: call `.to_mesh()` once and reuse the result rather
  than converting every frame.
