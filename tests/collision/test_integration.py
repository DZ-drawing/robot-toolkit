"""Integration tests for three-tier collision dispatch.

Tests that CollisionChecker correctly dispatches:
- Tier 1: primitive-primitive (existing analytical fast path)
- Tier 2: mesh-mesh (GJK/EPA)
- Tier 2: primitive-mesh (auto-convert primitive to mesh, then GJK/EPA)
"""

import numpy as np
import pytest

from robot_ik.collision import (
    Box,
    Capsule,
    CollisionChecker,
    CollisionResult,
    Sphere,
    TriangleMesh,
)

# ------------------------------------------------------------------
# Helper: create a box mesh at a given position
# ------------------------------------------------------------------


def _make_box_mesh(size, position):
    """Create a TriangleMesh box at the given world position."""
    mesh = TriangleMesh.from_box(size)
    pose = np.eye(4)
    pose[:3, 3] = np.asarray(position, dtype=float)
    mesh.pose = pose
    return mesh


def _make_sphere_mesh(radius, position, subdivisions=1):
    """Create a TriangleMesh sphere at the given world position."""
    mesh = TriangleMesh.from_sphere(radius, subdivisions=subdivisions)
    pose = np.eye(4)
    pose[:3, 3] = np.asarray(position, dtype=float)
    mesh.pose = pose
    return mesh


# ------------------------------------------------------------------
# Tier 1: primitive-only collision still uses analytical fast path
# ------------------------------------------------------------------


def test_primitive_sphere_sphere_still_works():
    """Sphere-sphere collision uses Tier 1 (unchanged)."""
    checker = CollisionChecker()

    s1 = Sphere(radius=0.1)
    s1.pose[:3, 3] = np.array([0.0, 0.0, 0.0])

    s2 = Sphere(radius=0.1)
    s2.pose[:3, 3] = np.array([0.15, 0.0, 0.0])  # Overlapping

    checker.add_link_geometry("link1", s1)
    checker.add_link_geometry("link2", s2)

    transforms = {"link1": np.eye(4), "link2": np.eye(4)}
    result = checker.check_self_collision(transforms, ignore_adjacent=False)

    assert result is not None
    assert result.is_colliding
    assert result.distance < 0


def test_primitive_no_collision():
    """Non-overlapping primitives use Tier 1 and report no collision."""
    checker = CollisionChecker()

    s1 = Sphere(radius=0.05)
    s1.pose[:3, 3] = np.array([0.0, 0.0, 0.0])
    s2 = Sphere(radius=0.05)
    s2.pose[:3, 3] = np.array([1.0, 0.0, 0.0])  # Far apart

    checker.add_link_geometry("link1", s1)
    checker.add_link_geometry("link2", s2)

    transforms = {"link1": np.eye(4), "link2": np.eye(4)}
    result = checker.check_self_collision(transforms, ignore_adjacent=False)

    assert result is None  # No collision


def test_primitive_environment_collision():
    """Sphere vs Box obstacle uses Tier 1."""
    checker = CollisionChecker()

    link_sphere = Sphere(radius=0.05)
    checker.add_link_geometry("link1", link_sphere)

    obstacle = Box(size=np.array([0.1, 0.1, 0.1]))
    obstacle.pose[:3, 3] = np.array([0.2, 0.0, 0.0])
    checker.add_obstacle(obstacle)

    # No collision
    transforms = {"link1": np.eye(4)}
    result = checker.check_environment_collision(transforms)
    assert result is None

    # Collision
    transforms["link1"][:3, 3] = np.array([0.15, 0.0, 0.0])
    result = checker.check_environment_collision(transforms)
    assert result is not None
    assert result.is_colliding


# ------------------------------------------------------------------
# Tier 2: mesh-mesh collision via CollisionChecker
# ------------------------------------------------------------------


def test_mesh_mesh_overlapping_boxes():
    """Two overlapping box meshes should collide."""
    checker = CollisionChecker()

    m1 = _make_box_mesh([0.2, 0.2, 0.2], [0.0, 0.0, 0.0])
    m2 = _make_box_mesh([0.2, 0.2, 0.2], [0.15, 0.0, 0.0])

    checker.add_link_geometry("mesh_link1", m1)
    checker.add_link_geometry("mesh_link2", m2)

    transforms = {"mesh_link1": np.eye(4), "mesh_link2": np.eye(4)}
    result = checker.check_self_collision(transforms, ignore_adjacent=False)

    assert result is not None
    assert result.is_colliding
    assert result.distance < 0  # Penetrating


def test_mesh_mesh_separated_boxes():
    """Two separated box meshes should not collide."""
    checker = CollisionChecker()

    m1 = _make_box_mesh([0.2, 0.2, 0.2], [0.0, 0.0, 0.0])
    m2 = _make_box_mesh([0.2, 0.2, 0.2], [1.0, 0.0, 0.0])

    checker.add_link_geometry("mesh_link1", m1)
    checker.add_link_geometry("mesh_link2", m2)

    transforms = {"mesh_link1": np.eye(4), "mesh_link2": np.eye(4)}
    result = checker.check_self_collision(transforms, ignore_adjacent=False)

    assert result is None  # No collision


def test_mesh_mesh_overlapping_spheres():
    """Two overlapping sphere meshes should collide."""
    checker = CollisionChecker()

    m1 = _make_sphere_mesh(0.1, [0.0, 0.0, 0.0])
    m2 = _make_sphere_mesh(0.1, [0.1, 0.0, 0.0])  # Overlapping

    checker.add_link_geometry("mesh_link1", m1)
    checker.add_link_geometry("mesh_link2", m2)

    transforms = {"mesh_link1": np.eye(4), "mesh_link2": np.eye(4)}
    result = checker.check_self_collision(transforms, ignore_adjacent=False)

    assert result is not None
    assert result.is_colliding


def test_mesh_mesh_with_link_transforms():
    """Mesh-mesh collision with link transforms applied."""
    checker = CollisionChecker()

    # Two box meshes defined at local origin
    m1 = TriangleMesh.from_box([0.2, 0.2, 0.2])
    m2 = TriangleMesh.from_box([0.2, 0.2, 0.2])

    checker.add_link_geometry("mesh_link1", m1)
    checker.add_link_geometry("mesh_link2", m2)

    # Transforms place them overlapping
    T1 = np.eye(4)
    T2 = np.eye(4)
    T2[:3, 3] = np.array([0.15, 0.0, 0.0])

    result = checker.check_self_collision(
        {"mesh_link1": T1, "mesh_link2": T2},
        ignore_adjacent=False,
    )

    assert result is not None
    assert result.is_colliding


# ------------------------------------------------------------------
# Tier 2: mixed primitive + mesh collision
# ------------------------------------------------------------------


def test_primitive_sphere_vs_mesh_box():
    """Sphere (primitive) vs Box (mesh) → auto-convert sphere to mesh."""
    checker = CollisionChecker()

    s1 = Sphere(radius=0.1)
    s1.pose[:3, 3] = np.array([0.0, 0.0, 0.0])

    m2 = _make_box_mesh([0.2, 0.2, 0.2], [0.15, 0.0, 0.0])

    checker.add_link_geometry("prim_link", s1)
    checker.add_link_geometry("mesh_link", m2)

    transforms = {"prim_link": np.eye(4), "mesh_link": np.eye(4)}
    result = checker.check_self_collision(transforms, ignore_adjacent=False)

    assert result is not None
    assert result.is_colliding


def test_primitive_box_vs_mesh_sphere():
    """Box (primitive) vs Sphere (mesh) → auto-convert box to mesh."""
    checker = CollisionChecker()

    b1 = Box(size=np.array([0.2, 0.2, 0.2]))
    b1.pose[:3, 3] = np.array([0.0, 0.0, 0.0])

    m2 = _make_sphere_mesh(0.1, [0.15, 0.0, 0.0])

    checker.add_link_geometry("prim_link", b1)
    checker.add_link_geometry("mesh_link", m2)

    transforms = {"prim_link": np.eye(4), "mesh_link": np.eye(4)}
    result = checker.check_self_collision(transforms, ignore_adjacent=False)

    assert result is not None
    assert result.is_colliding


def test_primitive_sphere_vs_mesh_box_separated():
    """Sphere (primitive) vs Box (mesh) separated → no collision."""
    checker = CollisionChecker()

    s1 = Sphere(radius=0.05)
    s1.pose[:3, 3] = np.array([0.0, 0.0, 0.0])

    m2 = _make_box_mesh([0.1, 0.1, 0.1], [1.0, 0.0, 0.0])

    checker.add_link_geometry("prim_link", s1)
    checker.add_link_geometry("mesh_link", m2)

    transforms = {"prim_link": np.eye(4), "mesh_link": np.eye(4)}
    result = checker.check_self_collision(transforms, ignore_adjacent=False)

    assert result is None  # No collision


def test_primitive_vs_mesh_environment_collision():
    """Primitive link vs mesh obstacle → auto-converts."""
    checker = CollisionChecker()

    link_sphere = Sphere(radius=0.1)
    checker.add_link_geometry("link1", link_sphere)

    obstacle_mesh = _make_box_mesh([0.2, 0.2, 0.2], [0.15, 0.0, 0.0])
    checker.add_obstacle(obstacle_mesh)

    transforms = {"link1": np.eye(4)}
    result = checker.check_environment_collision(transforms)

    assert result is not None
    assert result.is_colliding


# ------------------------------------------------------------------
# to_mesh() convenience methods
# ------------------------------------------------------------------


def test_sphere_to_mesh():
    """Sphere.to_mesh() produces a valid TriangleMesh."""
    s = Sphere(radius=0.1)
    s.pose[:3, 3] = np.array([1.0, 2.0, 3.0])
    mesh = s.to_mesh()

    assert isinstance(mesh, TriangleMesh)
    assert mesh.vertices.shape[0] > 0
    assert mesh.faces.shape[0] > 0
    np.testing.assert_array_almost_equal(mesh.pose[:3, 3], [1.0, 2.0, 3.0])


def test_box_to_mesh():
    """Box.to_mesh() produces a box mesh with correct size."""
    b = Box(size=np.array([0.2, 0.4, 0.6]))
    mesh = b.to_mesh()

    assert isinstance(mesh, TriangleMesh)
    assert mesh.vertices.shape[0] == 8
    assert mesh.faces.shape[0] == 12


def test_capsule_to_mesh():
    """Capsule.to_mesh() produces a valid TriangleMesh."""
    pytest.importorskip("scipy")
    c = Capsule(
        p1=np.array([0.0, 0.0, 0.0]),
        p2=np.array([0.0, 0.0, 0.2]),
        radius=0.03,
    )
    mesh = c.to_mesh()

    assert isinstance(mesh, TriangleMesh)
    assert mesh.vertices.shape[0] > 0
    assert mesh.faces.shape[0] > 0


# ------------------------------------------------------------------
# Direct _check_geometry_collision dispatch verification
# ------------------------------------------------------------------


def test_dispatch_mesh_mesh():
    """_check_geometry_collision dispatches mesh-mesh to GJK/EPA."""
    checker = CollisionChecker()

    m1 = _make_box_mesh([0.2, 0.2, 0.2], [0.0, 0.0, 0.0])
    m2 = _make_box_mesh([0.2, 0.2, 0.2], [0.1, 0.0, 0.0])

    result = checker._check_geometry_collision(m1, m2)
    assert isinstance(result, CollisionResult)
    assert result.is_colliding
    assert result.distance < 0


def test_dispatch_primitive_primitive():
    """_check_geometry_collision dispatches primitive-primitive to Tier 1."""
    checker = CollisionChecker()

    s1 = Sphere(radius=0.1)
    s1.pose[:3, 3] = np.array([0.0, 0.0, 0.0])
    s2 = Sphere(radius=0.1)
    s2.pose[:3, 3] = np.array([0.15, 0.0, 0.0])

    result = checker._check_geometry_collision(s1, s2)
    assert isinstance(result, CollisionResult)
    assert result.is_colliding
    # Tier 1 uses analytical distance: 0.15 - 0.1 - 0.1 = -0.05
    assert abs(result.distance - (-0.05)) < 1e-6


def test_dispatch_mixed_primitive_mesh():
    """_check_geometry_collision dispatches mixed pair to GJK/EPA."""
    checker = CollisionChecker()

    s = Sphere(radius=0.1)
    s.pose[:3, 3] = np.array([0.0, 0.0, 0.0])
    m = _make_box_mesh([0.2, 0.2, 0.2], [0.1, 0.0, 0.0])

    result = checker._check_geometry_collision(s, m)
    assert isinstance(result, CollisionResult)
    assert result.is_colliding
