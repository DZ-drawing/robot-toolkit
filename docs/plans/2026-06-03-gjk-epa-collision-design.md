# Mesh Collision Detection (GJK + EPA) Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Replace simple primitive collision detection with GJK + EPA algorithms supporting triangle mesh collision bodies, providing accurate distance queries, penetration depth, and contact information.

**Architecture:** Add `mesh.py` (TriangleMesh, convex hull) and `gjk.py`/`epa.py` (GJK/EPA algorithms) to `collision/`. `CollisionChecker` keeps its API but internally uses GJK/EPA for mesh-mesh checks. Pure NumPy, no new dependencies.

**Tech Stack:** NumPy (existing), scipy.spatial.ConvexHull for mesh convex hull generation

---

### Task 1: Create TriangleMesh data class

**Objective:** Define triangle mesh representation with vertex/face storage and convex hull computation.

**Files:**
- Create: `src/robot_ik/collision/mesh.py`
- Test: `tests/collision/test_mesh.py`

**Step 1: Write failing tests**

```python
# tests/collision/test_mesh.py
import numpy as np
import pytest
from robot_ik.collision.mesh import TriangleMesh

def test_create_from_vertices_and_faces():
    vertices = np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1]], dtype=float)
    faces = np.array([[0,1,2],[0,1,3],[0,2,3],[1,2,3]])
    mesh = TriangleMesh(vertices, faces)
    assert mesh.num_vertices == 4
    assert mesh.num_faces == 4

def test_from_convex_hull():
    """Generate convex hull from point cloud."""
    points = np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1]], dtype=float)
    mesh = TriangleMesh.from_convex_hull(points)
    assert mesh.is_convex
    assert mesh.num_faces >= 4

def test_support_function():
    """Support function returns farthest point in given direction."""
    vertices = np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1]], dtype=float)
    faces = np.array([[0,1,2],[0,1,3],[0,2,3],[1,2,3]])
    mesh = TriangleMesh(vertices, faces)
    support = mesh.support(np.array([1.0, 0.0, 0.0]))
    assert np.allclose(support, [1.0, 0.0, 0.0])

def test_support_with_transform():
    """Support function applies pose transform."""
    mesh = TriangleMesh.from_convex_hull(
        np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1]], dtype=float)
    )
    mesh.pose = np.eye(4)
    mesh.pose[0, 3] = 10.0  # translate x by 10
    support = mesh.support(np.array([1.0, 0.0, 0.0]))
    assert abs(support[0] - 11.0) < 1e-6

def test_from_box():
    """Create mesh from box dimensions."""
    mesh = TriangleMesh.from_box(np.array([1.0, 1.0, 1.0]))
    assert mesh.is_convex
    assert mesh.num_faces >= 12

def test_from_sphere():
    """Create mesh from sphere approximation."""
    mesh = TriangleMesh.from_sphere(radius=0.5, subdivisions=1)
    assert mesh.is_convex
    assert mesh.num_faces >= 8

def test_from_capsule():
    """Create mesh from capsule approximation."""
    mesh = TriangleMesh.from_capsule(p1=np.zeros(3), p2=np.array([0,0,0.2]), radius=0.03, subdivisions=6)
    assert mesh.is_convex
```

**Step 2: Implement TriangleMesh**

```python
# src/robot_ik/collision/mesh.py
"""Triangle mesh collision geometry with GJK support."""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field

@dataclass
class TriangleMesh:
    """Convex triangle mesh for collision detection.
    
    Stores vertices and triangular faces. Provides support function
    for GJK algorithm. Assumes convex geometry.
    """
    vertices: np.ndarray  # (N, 3) vertex positions
    faces: np.ndarray     # (M, 3) face indices
    pose: np.ndarray = field(default_factory=lambda: np.eye(4))
    name: str = ""

    def __post_init__(self):
        self.vertices = np.asarray(self.vertices, dtype=float)
        self.faces = np.asarray(self.faces, dtype=int)
        self._rot = self.pose[:3, :3]
        self._trans = self.pose[:3, 3]

    @property
    def num_vertices(self) -> int:
        return len(self.vertices)

    @property
    def num_faces(self) -> int:
        return len(self.faces)

    @property
    def is_convex(self) -> bool:
        """Quick check: if we generated from convex hull, assume True."""
        return self._is_convex

    def support(self, direction: np.ndarray) -> np.ndarray:
        """Find farthest vertex in given direction (GJK support).
        
        Args:
            direction: (3,) search direction in world frame.
        
        Returns:
            (3,) support point in world frame.
        """
        d = self._rot @ direction
        dots = self.vertices @ d
        idx = np.argmax(dots)
        local_point = self.vertices[idx]
        return self._rot @ local_point + self._trans

    def update_pose(self, pose: np.ndarray):
        """Update pose and cached transform."""
        self.pose = pose
        self._rot = pose[:3, :3]
        self._trans = pose[:3, 3]

    @classmethod
    def from_convex_hull(cls, points: np.ndarray) -> TriangleMesh:
        """Create convex hull mesh from point cloud using scipy."""
        from scipy.spatial import ConvexHull
        points = np.asarray(points, dtype=float)
        hull = ConvexHull(points)
        mesh = cls(vertices=points[hull.vertices], faces=hull.simplices)
        mesh._is_convex = True
        return mesh

    @classmethod
    def from_box(cls, size: np.ndarray) -> TriangleMesh:
        """Create box mesh from dimensions (x, y, z)."""
        hx, hy, hz = np.asarray(size, dtype=float) / 2
        v = np.array([
            [-hx,-hy,-hz], [hx,-hy,-hz], [hx,hy,-hz], [-hx,hy,-hz],
            [-hx,-hy, hz], [hx,-hy, hz], [hx,hy, hz], [-hx,hy, hz],
        ])
        f = np.array([
            [0,1,2],[0,2,3],  # bottom
            [4,5,6],[4,6,7],  # top
            [0,1,5],[0,5,4],  # front
            [2,3,7],[2,7,6],  # back
            [0,3,7],[0,7,4],  # left
            [1,2,6],[1,6,5],  # right
        ])
        mesh = cls(v, f)
        mesh._is_convex = True
        return mesh

    @classmethod
    def from_sphere(cls, radius: float = 1.0, subdivisions: int = 2) -> TriangleMesh:
        """Create icosphere approximation. subdivisions=1 → 42 verts."""
        # Icosahedron base
        phi = (1 + np.sqrt(5)) / 2
        raw = np.array([
            [-1,phi,0],[1,phi,0],[-1,-phi,0],[1,-phi,0],
            [0,-1,phi],[0,1,phi],[0,-1,-phi],[0,1,-phi],
            [phi,0,-1],[phi,0,1],[-phi,0,-1],[-phi,0,1],
        ], dtype=float)
        vertices = raw / np.linalg.norm(raw[0]) * radius
        
        faces = np.array([
            [0,11,5],[0,5,1],[0,1,7],[0,7,10],[0,10,11],
            [1,5,9],[5,11,4],[11,10,2],[10,7,6],[7,1,8],
            [3,9,4],[3,4,2],[3,2,6],[3,6,8],[3,8,9],
            [4,9,5],[2,4,11],[6,2,10],[8,6,7],[9,8,1],
        ])
        
        # Subdivide
        for _ in range(subdivisions):
            vertices, faces = cls._subdivide(vertices, faces)
        vertices *= radius / np.max(np.linalg.norm(vertices, axis=1))
        
        mesh = cls(vertices, faces)
        mesh._is_convex = True
        return mesh

    @classmethod
    def from_capsule(cls, p1: np.ndarray, p2: np.ndarray, 
                     radius: float = 0.03, subdivisions: int = 8) -> TriangleMesh:
        """Create capsule mesh approximation (cylinder + hemisphere caps)."""
        p1, p2 = np.asarray(p1, dtype=float), np.asarray(p2, dtype=float)
        axis = p2 - p1
        length = np.linalg.norm(axis)
        if length < 1e-10:
            return cls.from_sphere(radius)
        
        verts, faces = [], []
        n = subdivisions
        
        # Generate points along capsule
        for i in range(n + 1):
            theta = 2 * np.pi * i / n
            # Ring at p1 hemisphere bottom
            z = -radius
            x = radius * np.cos(theta)
            y = radius * np.sin(theta)
            verts.append(np.array([0, 0, z]) + p1)
        
        for i in range(n):
            theta = 2 * np.pi * i / n
            x = radius * np.cos(theta)
            y = radius * np.sin(theta)
            verts.append(np.array([x, y, 0]) + p1)
            verts.append(np.array([x, y, length]) + p1)
        
        for i in range(n + 1):
            theta = 2 * np.pi * i / n
            x = radius * np.cos(theta)
            y = radius * np.sin(theta)
            verts.append(np.array([0, 0, length + radius]) + p1)
        
        # Connect faces (simplified: cylinder bands + caps)
        # This is a simplified mesh — sufficient for convex hull
        verts = np.array(verts)
        mesh = cls.from_convex_hull(verts)
        mesh._is_convex = True
        return mesh

    @classmethod
    def _subdivide(cls, vertices, faces):
        """Subdivide each triangle into 4."""
        edge_midpoints = {}
        new_verts = list(vertices)
        new_faces = []
        
        def get_midpoint(i, j):
            key = (min(i,j), max(i,j))
            if key not in edge_midpoints:
                mid = (vertices[i] + vertices[j]) / 2
                edge_midpoints[key] = len(new_verts)
                new_verts.append(mid)
            return edge_midpoints[key]
        
        for f in faces:
            a, b, c = f
            ab = get_midpoint(a, b)
            bc = get_midpoint(b, c)
            ca = get_midpoint(c, a)
            new_faces.extend([
                [a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]
            ])
        
        return np.array(new_verts), np.array(new_faces)
```

**Step 3: Run tests**

```bash
pytest tests/collision/test_mesh.py -v
```

**Step 4: Commit**

```bash
git add src/robot_ik/collision/mesh.py tests/collision/test_mesh.py
git commit -m "feat(collision): add TriangleMesh class with support function for GJK"
```

---

### Task 2: Implement GJK algorithm

**Objective:** Implement Gilbert-Johnson-Keerthi distance algorithm for convex shapes.

**Files:**
- Create: `src/robot_ik/collision/gjk.py`
- Test: `tests/collision/test_gjk.py`

**Step 1: Write failing tests**

```python
# tests/collision/test_gjk.py
import numpy as np
from robot_ik.collision.mesh import TriangleMesh
from robot_ik.collision.gjk import gjk_distance, gjk_intersect

def test_gjk_separated_spheres():
    """Two spheres 0.5 apart → distance ≈ 0.5."""
    s1 = TriangleMesh.from_sphere(0.1, subdivisions=1)
    s1.update_pose(np.eye(4))
    
    s2 = TriangleMesh.from_sphere(0.1, subdivisions=1)
    pose2 = np.eye(4)
    pose2[0, 3] = 0.7
    s2.update_pose(pose2)
    
    distance, closest1, closest2 = gjk_distance(s1, s2)
    assert abs(distance - 0.5) < 0.05

def test_gjk_overlapping():
    """Two overlapping spheres → intersecting."""
    s1 = TriangleMesh.from_sphere(0.1, subdivisions=1)
    s2 = TriangleMesh.from_sphere(0.1, subdivisions=1)
    pose2 = np.eye(4)
    pose2[0, 3] = 0.1
    s2.update_pose(pose2)
    
    assert gjk_intersect(s1, s2) is True

def test_gjk_touching():
    """Two spheres exactly touching → distance ≈ 0."""
    s1 = TriangleMesh.from_sphere(0.1, subdivisions=1)
    s2 = TriangleMesh.from_sphere(0.1, subdivisions=1)
    pose2 = np.eye(4)
    pose2[0, 3] = 0.2
    s2.update_pose(pose2)
    
    distance, _, _ = gjk_distance(s1, s2)
    assert abs(distance) < 0.05

def test_gjk_box_separation():
    """Two boxes separated."""
    b1 = TriangleMesh.from_box(np.array([1, 1, 1]))
    b2 = TriangleMesh.from_box(np.array([1, 1, 1]))
    pose2 = np.eye(4); pose2[0, 3] = 3.0
    b2.update_pose(pose2)
    
    distance, _, _ = gjk_distance(b1, b2)
    assert distance > 0

def test_gjk_empty_shapes():
    """Single vertex 'shapes' should still work."""
    p1 = TriangleMesh(np.array([[0,0,0]]), np.array([[0,0,0]]))
    p2 = TriangleMesh(np.array([[1,0,0]]), np.array([[0,0,0]]))
    distance, c1, c2 = gjk_distance(p1, p2)
    assert abs(distance - 1.0) < 0.01
```

**Step 2: Implement GJK**

```python
# src/robot_ik/collision/gjk.py
"""GJK (Gilbert-Johnson-Keerthi) collision detection algorithm.

Computes minimum distance between two convex shapes using the
Minkowski difference and iterative simplex evolution."""

from __future__ import annotations

import numpy as np


def _support_minkowski(
    shape_a: TriangleMesh, shape_b: TriangleMesh, direction: np.ndarray
) -> np.ndarray:
    """Support function on Minkowski difference A ⊖ B."""
    return shape_a.support(direction) - shape_b.support(-direction)


def _triple_product(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> np.ndarray:
    """Triple product: (A × B) × C."""
    return np.cross(np.cross(a, b), c)


def _closest_on_line(origin: np.ndarray, direction: np.ndarray, point: np.ndarray) -> np.ndarray:
    """Closest point on line (origin + t*direction) to given point."""
    d = point - origin
    t = np.dot(d, direction) / (np.dot(direction, direction) + 1e-30)
    return origin + t * direction


def _closest_on_triangle(a: np.ndarray, b: np.ndarray, c: np.ndarray, origin: np.ndarray) -> tuple:
    """Find closest point on triangle ABC to origin. Returns (point, barycentric)."""
    ab = b - a
    ac = c - a
    ao = -a

    d1 = np.dot(ab, ao)
    d2 = np.dot(ac, ao)
    if d1 <= 0 and d2 <= 0:
        return a.copy(), np.array([1.0, 0.0, 0.0])

    bo = -b
    d3 = np.dot(ab, bo)
    d4 = np.dot(ac, bo)
    if d3 >= 0 and d4 <= d3:
        return b.copy(), np.array([0.0, 1.0, 0.0])

    vc = d1 * d4 - d3 * d2
    if vc <= 0 and d1 >= 0 and d3 <= 0:
        v = d1 / (d1 - d3)
        return a + v * ab, np.array([1-v, v, 0.0])

    co = -c
    d5 = np.dot(ab, co)
    d6 = np.dot(ac, co)
    if d6 >= 0 and d5 <= d6:
        return c.copy(), np.array([0.0, 0.0, 1.0])

    vb = d5 * d2 - d1 * d6
    if vb <= 0 and d2 >= 0 and d6 <= 0:
        w = d2 / (d2 - d6)
        return a + w * ac, np.array([1-w, 0.0, w])

    va = d3 * d6 - d5 * d4
    if va <= 0 and (d4 - d3) >= 0 and (d5 - d6) >= 0:
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return b + w * (c - b), np.array([0.0, 1-w, w])

    denom = 1.0 / (va + vb + vc + 1e-30)
    v = vb * denom
    w = vc * denom
    return a + ab * v + ac * w, np.array([1-v-w, v, w])


def _evolve_simplex(simplex: list, direction: np.ndarray) -> tuple:
    """Evolve simplex, return (new_direction, contains_origin, updated_simplex)."""
    n = len(simplex)
    if n == 2:
        return _evolve_line(simplex, direction)
    elif n == 3:
        return _evolve_triangle(simplex, direction)
    elif n == 4:
        return _evolve_tetrahedron(simplex, direction)
    return direction, False, simplex


def _evolve_line(simplex, direction):
    a, b = simplex[0], simplex[1]
    ab = b - a
    ao = -a
    d = np.dot(ab, ao)
    if d > 0:
        return _triple_product(ab, ao, ab), False, [b, a]
    else:
        return ao, False, [a]


def _evolve_triangle(simplex, direction):
    a, b, c = simplex[0], simplex[1], simplex[2]
    ab = b - a
    ac = c - a
    ao = -a

    n = np.cross(ab, ac)
    if np.dot(n, ao) > 0:
        if np.dot(np.cross(ab, n), ao) > 0:
            return _triple_product(ab, ao, ab), False, [b, a]
        elif np.dot(np.cross(n, ac), ao) > 0:
            return _triple_product(ac, ao, ac), False, [c, a]
        else:
            return n, False, [b, c, a]
    else:
        # Origin above triangle plane or below
        if np.dot(np.cross(ab, n), ao) > 0:
            return _triple_product(ab, ao, ab), False, [b, a]
        elif np.dot(np.cross(n, ac), ao) > 0:
            return _triple_product(ac, ao, ac), False, [c, a]
        else:
            # Origin is inside the triangle
            return n, False, [b, c, a]


def _evolve_tetrahedron(simplex, direction):
    a, b, c, d = simplex[0], simplex[1], simplex[2], simplex[3]
    
    # Check each face
    ab = b - a
    ac = c - a
    ad = d - a
    ao = -a

    # Face ABC
    n_abc = np.cross(ab, ac)
    if np.dot(n_abc, ao) > 0:
        return _evolve_triangle([a, b, c], direction)
    
    # Face ABD
    n_abd = np.cross(ab, ad)
    if np.dot(n_abd, ao) > 0:
        return _evolve_triangle([a, d, b], direction)
    
    # Face ACD
    n_acd = np.cross(ac, ad)
    if np.dot(n_acd, ao) > 0:
        return _evolve_triangle([a, c, d], direction)
    
    # Face BCD
    bc = c - b
    bd = d - b
    bo = -b
    n_bcd = np.cross(bc, bd)
    if np.dot(n_bcd, bo) > 0:
        return _evolve_triangle([b, c, d], direction)
    
    # Origin inside tetrahedron
    return direction, True, simplex


def gjk_intersect(shape_a: TriangleMesh, shape_b: TriangleMesh,
                  max_iterations: int = 64) -> bool:
    """Check if two convex shapes intersect using GJK.
    
    Args:
        shape_a, shape_b: TriangleMesh instances.
        max_iterations: Maximum simplex iterations.
    
    Returns:
        True if shapes intersect (origin inside Minkowski difference).
    """
    d = np.array([1.0, 0.0, 0.0])
    
    # Get initial support point
    a = _support_minkowski(shape_a, shape_b, d)
    if np.dot(a, a) < 1e-12:
        return True  # Origin on boundary
    
    d = -a
    simplex = [a]
    
    for _ in range(max_iterations):
        a = _support_minkowski(shape_a, shape_b, d)
        
        if np.dot(a, d) < 0:
            return False  # No intersection
        
        simplex.insert(0, a)
        d, contains_origin, simplex = _evolve_simplex(simplex, d)
        
        if contains_origin:
            return True
        
        d_len = np.linalg.norm(d)
        if d_len < 1e-12:
            return True  # Degenerate
    
    return False


def gjk_distance(shape_a: TriangleMesh, shape_b: TriangleMesh,
                  max_iterations: int = 64) -> tuple:
    """Compute minimum distance between two convex shapes.
    
    Args:
        shape_a, shape_b: TriangleMesh instances.
        max_iterations: Maximum iterations.
    
    Returns:
        (distance, closest_on_a, closest_on_b)
    """
    d = np.array([1.0, 0.0, 0.0])
    a = _support_minkowski(shape_a, shape_b, d)
    simplex = [a]
    d = -a
    
    for _ in range(max_iterations):
        a = _support_minkowski(shape_a, shape_b, d)
        
        if np.dot(a, d) < -1e-10:
            # Shapes are separated — compute distance from simplex to origin
            dist, _ = _simplex_distance(simplex)
            return abs(dist), None, None  # Approximate
        
        simplex.insert(0, a)
        d, contains_origin, simplex = _evolve_simplex(simplex, d)
        
        if contains_origin:
            return 0.0, None, None  # Intersecting
    
    # Separated: distance from final simplex to origin
    dist, closest = _simplex_distance(simplex)
    return abs(dist), closest, closest  # TODO: decompose back to A, B


def _simplex_distance(simplex: list) -> tuple:
    """Distance from simplex to origin and closest point."""
    if len(simplex) == 1:
        return np.linalg.norm(simplex[0]), simplex[0]
    elif len(simplex) == 2:
        return _segment_distance(simplex[0], simplex[1])
    elif len(simplex) == 3:
        return _triangle_distance(simplex[0], simplex[1], simplex[2])
    else:
        # Tetrahedron: check all faces
        s0, s1, s2, s3 = simplex
        faces = [(s0, s1, s2), (s0, s1, s3), (s0, s2, s3), (s1, s2, s3)]
        min_dist = float('inf')
        min_point = None
        for f in faces:
            d, p = _triangle_distance(*f)
            if d < min_dist:
                min_dist = d
                min_point = p
        return min_dist, min_point


def _segment_distance(a, b) -> tuple:
    """Distance from segment AB to origin."""
    ab = b - a
    ao = -a
    t = np.dot(ab, ao) / (np.dot(ab, ab) + 1e-30)
    t = np.clip(t, 0, 1)
    closest = a + t * ab
    return np.linalg.norm(closest), closest


def _triangle_distance(a, b, c) -> tuple:
    """Distance from triangle ABC to origin."""
    closest, _ = _closest_on_triangle(a, b, c, np.zeros(3))
    return np.linalg.norm(closest), closest
```

**Step 3: Run tests**

```bash
pytest tests/collision/test_gjk.py -v
```

**Step 4: Commit**

```bash
git add src/robot_ik/collision/gjk.py tests/collision/test_gjk.py
git commit -m "feat(collision): implement GJK algorithm for convex shape distance query"
```

---

### Task 3: Implement EPA algorithm

**Objective:** Implement Expanding Polytope Algorithm for penetration depth and contact info.

**Files:**
- Create: `src/robot_ik/collision/epa.py`
- Test: `tests/collision/test_epa.py`

**Step 1: Write failing tests**

```python
# tests/collision/test_epa.py
import numpy as np
from robot_ik.collision.mesh import TriangleMesh
from robot_ik.collision.epa import epa_penetration

def test_epa_sphere_overlap():
    """Two overlapping spheres → penetration depth > 0."""
    s1 = TriangleMesh.from_sphere(0.1, subdivisions=2)
    s2 = TriangleMesh.from_sphere(0.1, subdivisions=2)
    pose2 = np.eye(4); pose2[0, 3] = 0.15
    s2.update_pose(pose2)
    
    depth, normal, contact = epa_penetration(s1, s2)
    assert depth > 0
    assert np.linalg.norm(normal) > 0.9  # Unit vector
    assert contact is not None

def test_epa_deep_penetration():
    """Deeply overlapping boxes."""
    b1 = TriangleMesh.from_box(np.array([1, 1, 1]))
    b2 = TriangleMesh.from_box(np.array([1, 1, 1]))
    pose2 = np.eye(4); pose2[0, 3] = 0.5
    b2.update_pose(pose2)
    
    depth, normal, contact = epa_penetration(b1, b2)
    assert depth > 0.5
    assert abs(normal[0]) > 0.9  # Normal along x
```

**Step 2: Implement EPA**

```python
# src/robot_ik/collision/epa.py
"""EPA (Expanding Polytope Algorithm) for penetration depth.

Given a GJK simplex that contains the origin, EPA expands the simplex
to find the penetration depth, contact normal, and contact point."""

from __future__ import annotations

import numpy as np
from collections import deque


def epa_penetration(
    shape_a: TriangleMesh,
    shape_b: TriangleMesh,
    gjk_simplex: list | None = None,
    max_iterations: int = 64,
    tolerance: float = 1e-6,
) -> tuple:
    """Compute penetration depth, contact normal, and contact point.
    
    Args:
        shape_a, shape_b: Intersecting convex shapes.
        gjk_simplex: Pre-computed GJK simplex (4 vertices containing origin).
                     If None, will compute via GJK.
        max_iterations: EPA expansion limit.
        tolerance: Convergence tolerance.
    
    Returns:
        (depth, normal, contact_point)
        - depth: Penetration depth (> 0).
        - normal: Contact normal pointing from B to A.
        - contact_point: Contact point in world frame.
    """
    from robot_ik.collision.gjk import (
        _support_minkowski, _evolve_simplex, gjk_intersect,
    )
    
    # If no simplex provided, build one
    if gjk_simplex is None or len(gjk_simplex) < 4:
        # Run GJK to get simplex
        d = np.array([1.0, 0.0, 0.0])
        a = _support_minkowski(shape_a, shape_b, d)
        simplex = [a]
        d = -a
        for _ in range(64):
            a = _support_minkowski(shape_a, shape_b, d)
            simplex.insert(0, a)
            d, contains, simplex = _evolve_simplex(simplex, d)
            if contains:
                break
        gjk_simplex = simplex
    
    # Ensure we have a tetrahedron
    if len(gjk_simplex) < 4:
        return 0.0, np.array([1.0, 0, 0]), np.zeros(3)
    
    # Build face list: each face = (normal, distance_to_origin, [v0, v1, v2])
    vertices = list(gjk_simplex[:4])
    faces = []
    
    # Create 4 faces of tetrahedron
    for i in range(4):
        others = [j for j in range(4) if j != i]
        v0, v1, v2 = vertices[others[0]], vertices[others[1]], vertices[others[2]]
        normal = np.cross(v1 - v0, v2 - v0)
        n_len = np.linalg.norm(normal)
        if n_len > 1e-12:
            normal /= n_len
        dist = np.dot(normal, v0)
        faces.append((normal, dist, [others[0], others[1], others[2]]))
    
    # EPA iteration: find closest face, expand along its normal
    for _ in range(max_iterations):
        # Find closest face to origin
        min_dist = float('inf')
        min_face_idx = 0
        for i, (normal, dist, _) in enumerate(faces):
            if dist < min_dist:
                min_dist = dist
                min_face_idx = i
        
        if min_dist < tolerance:
            break
        
        # Get support point in face normal direction
        normal = faces[min_face_idx][0]
        support = _support_minkowski(shape_a, shape_b, normal)
        
        # Check if support is close to face (converged)
        new_dist = np.dot(normal, support)
        if new_dist - min_dist < tolerance:
            break
        
        # Add support point, remove visible faces, fill horizon
        faces, vertices = _expand_polytope(
            faces, vertices, support, min_face_idx
        )
        
        if not faces:
            break
    
    # Extract result from closest face
    min_dist = float('inf')
    best_normal = np.array([1.0, 0, 0])
    for normal, dist, idx in faces:
        if dist < min_dist:
            min_dist = dist
            best_normal = normal
    
    # Contact point = face centroid projected
    min_face = None
    for normal, dist, idx in faces:
        if dist < min_dist + 1e-10:
            min_face = idx
            break
    
    if min_face is not None:
        centroid = sum(vertices[i] for i in min_face) / len(min_face)
    else:
        centroid = np.zeros(3)
    
    return min_dist, best_normal / (np.linalg.norm(best_normal) + 1e-12), centroid


def _expand_polytope(faces, vertices, support, visible_idx):
    """Expand polytope by adding support point, removing visible faces."""
    from collections import deque
    
    # Find all faces visible from support point
    visible = set()
    queue = deque([visible_idx])
    
    normal_vis = faces[visible_idx][0]
    support_dist = np.dot(normal_vis, support)
    
    if support_dist <= faces[visible_idx][1] + 1e-10:
        # Not expanding — degenerate case
        return faces, vertices
    
    visible.add(visible_idx)
    horizon_edges = []
    
    while queue:
        fi = queue.popleft()
        face_normal, face_dist, face_verts = faces[fi]
        
        for i in range(3):
            edge = (face_verts[i], face_verts[(i+1) % 3])
            
            # Find adjacent face sharing this edge
            adj = _find_adjacent_face(faces, fi, edge)
            if adj is None:
                continue
            
            if adj in visible:
                continue
            
            adj_normal = faces[adj][0]
            if np.dot(adj_normal, support) > faces[adj][1] + 1e-10:
                visible.add(adj)
                queue.append(adj)
            else:
                horizon_edges.append((edge[0], edge[1]))
    
    # Remove visible faces
    new_faces = [f for i, f in enumerate(faces) if i not in visible]
    
    # Add new vertex
    new_vert_idx = len(vertices)
    vertices.append(support)
    
    # Create new faces from horizon edges to support point
    for e0, e1 in horizon_edges:
        v0, v1, v2 = vertices[e0], vertices[e1], support
        normal = np.cross(v1 - v0, v2 - v0)
        n_len = np.linalg.norm(normal)
        if n_len > 1e-12:
            normal /= n_len
        dist = np.dot(normal, v0)
        new_faces.append((normal, dist, [e0, e1, new_vert_idx]))
    
    return new_faces, vertices


def _find_adjacent_face(faces, current_idx, edge):
    """Find face adjacent to current face sharing given edge."""
    e0, e1 = edge
    for i, (_, _, verts) in enumerate(faces):
        if i == current_idx:
            continue
        for j in range(3):
            if verts[j] == e1 and verts[(j+1) % 3] == e0:
                return i
            if verts[j] == e0 and verts[(j+1) % 3] == e1:
                return i
    return None
```

**Step 3: Run tests**

```bash
pytest tests/collision/test_epa.py -v
```

**Step 4: Commit**

```bash
git add src/robot_ik/collision/epa.py tests/collision/test_epa.py
git commit -m "feat(collision): implement EPA for penetration depth and contact info"
```

---

### Task 4: Integrate GJK/EPA into CollisionChecker

**Objective:** Update CollisionChecker to use GJK/EPA internally while keeping the same public API.

**Files:**
- Modify: `src/robot_ik/collision/module.py`
- Test: `tests/collision/test_collision.py` (extend existing)

**Step 1: Write integration test**

```python
def test_gjk_based_self_collision():
    """Self-collision using GJK mesh-based detection."""
    checker = CollisionChecker()
    
    link1 = TriangleMesh.from_sphere(0.05, subdivisions=1)
    link1.name = "link1"
    checker.add_link_geometry("link1", link1)
    
    link3 = TriangleMesh.from_sphere(0.05, subdivisions=1)
    link3.name = "link3"
    checker.add_link_geometry("link3", link3)
    
    # Overlapping
    transforms = {"link1": np.eye(4), "link3": np.eye(4)}
    result = checker.check_self_collision(transforms, ignore_adjacent=True)
    assert result is not None
    assert result.is_colliding

def test_gjk_based_env_collision():
    """Environment collision using mesh-based detection."""
    checker = CollisionChecker()
    
    link = TriangleMesh.from_sphere(0.05, subdivisions=1)
    checker.add_link_geometry("link1", link)
    
    obstacle = TriangleMesh.from_box(np.array([0.1, 0.1, 0.1]))
    pose = np.eye(4); pose[0, 3] = 0.2
    obstacle.update_pose(pose)
    checker.add_obstacle(obstacle)
    
    # No collision
    transforms = {"link1": np.eye(4)}
    result = checker.check_environment_collision(transforms)
    assert result is None
    
    # Collision
    transforms["link1"][:3, 3] = np.array([0.15, 0, 0])
    result = checker.check_environment_collision(transforms)
    assert result is not None
    assert result.is_colliding
```

**Step 2: Update CollisionChecker**

Modify `_check_geometry_collision` to detect TriangleMesh and use GJK/EPA:

```python
def _check_geometry_collision(self, g1, g2, collision_threshold=0.0):
    from robot_ik.collision.mesh import TriangleMesh
    from robot_ik.collision.gjk import gjk_intersect, gjk_distance
    from robot_ik.collision.epa import epa_penetration
    
    # Apply poses
    from robot_ik.collision.mesh import TriangleMesh
    if isinstance(g1, TriangleMesh) and isinstance(g2, TriangleMesh):
        if gjk_intersect(g1, g2):
            depth, normal, contact = epa_penetration(g1, g2)
            return CollisionResult(
                is_colliding=True, distance=-depth,
                contact_point=contact, pair=("", "")
            )
        dist, _, _ = gjk_distance(g1, g2)
        return CollisionResult(
            is_colliding=dist <= collision_threshold,
            distance=dist, contact_point=None, pair=("", "")
        )
    
    # Fall back to existing primitive checks for backward compat
    ...existing code...
```

**Step 3: Run all collision tests**

```bash
pytest tests/collision/ -v
```

**Step 4: Commit**

```bash
git add src/robot_ik/collision/ tests/collision/
git commit -m "feat(collision): integrate GJK/EPA into CollisionChecker"
```

---

### Task 5: Update __init__.py and pyproject.toml

**Objective:** Export new API, add scipy dependency.

**Files:**
- Modify: `src/robot_ik/collision/__init__.py`
- Modify: `pyproject.toml`

Add to `__init__.py`:
```python
from robot_ik.collision.mesh import TriangleMesh
from robot_ik.collision.gjk import gjk_distance, gjk_intersect
from robot_ik.collision.epa import epa_penetration
```

Add to `pyproject.toml` dependencies:
```toml
[project.optional-dependencies]
collision = ["scipy>=1.10"]
```

**Step: Run full test suite**

```bash
pytest tests/ -v
```

**Commit:**
```bash
git add src/robot_ik/collision/__init__.py pyproject.toml
git commit -m "feat(collision): export GJK/EPA API, add [collision] optional extra"
```

---

### Task 6: Write Chinese documentation

**Objective:** Add Chinese docs for new collision module.

**Files:**
- Create: `docs/zh/collision.md`
- Modify: `docs/zh/README.md` (add link)

Document: GJK/EPA 算法说明、TriangleMesh 用法、迁移指南。
