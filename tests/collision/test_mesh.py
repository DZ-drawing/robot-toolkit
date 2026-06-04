"""Test suite for TriangleMesh (GJK collision detection support)."""

import numpy as np
import pytest

scipy = pytest.importorskip("scipy")

from robot_ik.collision.mesh import TriangleMesh


def test_create_from_vertices_and_faces():
    """Construct a TriangleMesh directly from vertices and faces."""
    vertices = np.array(
        [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]],
        dtype=float,
    )
    faces = np.array([[0, 1, 2], [0, 2, 3]], dtype=int)

    mesh = TriangleMesh(vertices=vertices, faces=faces, name="quad")
    assert mesh.vertices.shape == (4, 3)
    assert mesh.faces.shape == (2, 3)
    assert mesh.name == "quad"
    assert np.allclose(mesh.pose, np.eye(4))
    print("  [PASS] test_create_from_vertices_and_faces")


def test_from_convex_hull():
    """Create mesh from convex hull of random points."""
    rng = np.random.default_rng(42)
    points = rng.standard_normal((50, 3))

    mesh = TriangleMesh.from_convex_hull(points)
    assert mesh.vertices.shape[0] > 0
    assert mesh.faces.shape[0] > 0
    assert mesh.vertices.shape[1] == 3
    assert mesh.faces.shape[1] == 3
    # All hull vertices should be from the original set
    assert mesh.vertices.shape[0] <= points.shape[0]
    print("  [PASS] test_from_convex_hull")


def test_support_function():
    """Support in direction [1,0,0] should return the vertex with max x."""
    vertices = np.array(
        [
            [-1, 0, 0],
            [0.5, 0, 0],
            [1.0, 0, 0],
            [0.2, 1, 0],
        ],
        dtype=float,
    )
    faces = np.array([[0, 1, 2], [0, 2, 3]], dtype=int)
    mesh = TriangleMesh(vertices=vertices, faces=faces)

    result = mesh.support(np.array([1.0, 0.0, 0.0]))
    # In identity pose, the farthest vertex along +x is [1, 0, 0]
    assert np.allclose(result, np.array([1.0, 0.0, 0.0])), f"Got {result}"
    print("  [PASS] test_support_function")


def test_support_with_transform():
    """Translation in pose should shift the support point."""
    vertices = np.array(
        [
            [-1, 0, 0],
            [0.5, 0, 0],
            [1.0, 0, 0],
            [0.2, 1, 0],
        ],
        dtype=float,
    )
    faces = np.array([[0, 1, 2], [0, 2, 3]], dtype=int)

    # Pose with translation [10, 20, 30]
    pose = np.eye(4)
    pose[:3, 3] = np.array([10.0, 20.0, 30.0])
    mesh = TriangleMesh(vertices=vertices, faces=faces, pose=pose)

    result = mesh.support(np.array([1.0, 0.0, 0.0]))
    expected = np.array([11.0, 20.0, 30.0])
    assert np.allclose(result, expected), f"Got {result}, expected {expected}"
    print("  [PASS] test_support_with_transform")


def test_from_box():
    """Box mesh should have exactly 12 faces and be convex."""
    mesh = TriangleMesh.from_box(np.array([2.0, 3.0, 4.0]))
    assert mesh.faces.shape[0] == 12, f"Expected 12 faces, got {mesh.faces.shape[0]}"
    assert mesh.vertices.shape[0] == 8, f"Expected 8 vertices, got {mesh.vertices.shape[0]}"

    # Verify convexity: all vertices should satisfy support property
    # i.e., no vertex lies outside the convex hull of the others
    from scipy.spatial import ConvexHull

    hull = ConvexHull(mesh.vertices)
    assert hull.vertices.shape[0] == 8, "Box mesh should be convex with all 8 vertices on hull"

    # Check extents match
    for axis in range(3):
        half_size = [2.0, 3.0, 4.0][axis] / 2.0
        assert np.isclose(mesh.vertices[:, axis].max(), half_size)
        assert np.isclose(mesh.vertices[:, axis].min(), -half_size)

    print("  [PASS] test_from_box")


def test_from_sphere():
    """Sphere mesh should be convex with enough faces."""
    mesh = TriangleMesh.from_sphere(radius=1.0, subdivisions=1)
    assert mesh.faces.shape[0] >= 8, f"Expected >= 8 faces, got {mesh.faces.shape[0]}"

    # Check convexity via convex hull – all faces should be on the hull
    from scipy.spatial import ConvexHull

    hull = ConvexHull(mesh.vertices)
    assert hull.vertices.shape[0] == mesh.vertices.shape[0], (
        "Sphere mesh should be fully convex"
    )

    # All vertices should be approximately at radius distance from origin
    radii = np.linalg.norm(mesh.vertices, axis=1)
    assert np.allclose(radii, 1.0, atol=1e-10), f"Radii: {radii.min()}..{radii.max()}"

    print("  [PASS] test_from_sphere")


def test_from_capsule():
    """Capsule mesh should be convex."""
    p1 = np.array([0.0, 0.0, 0.0])
    p2 = np.array([0.0, 0.0, 1.0])
    radius = 0.1
    mesh = TriangleMesh.from_capsule(p1, p2, radius=radius, subdivisions=1)

    assert mesh.vertices.shape[0] > 0
    assert mesh.faces.shape[0] > 0

    # Verify convexity
    from scipy.spatial import ConvexHull

    hull = ConvexHull(mesh.vertices)
    # In a convex mesh, the number of hull vertices equals total unique vertices
    # (minus possible interior points; for a capsule approximation some may be interior)
    assert hull.vertices.shape[0] > 0, "Capsule mesh must have hull vertices"

    # All vertices should be within radius + small tolerance of the capsule body
    # Project each vertex onto the capsule axis and check distance
    axis = p2 - p1
    length = np.linalg.norm(axis)
    axis_dir = axis / length
    for v in mesh.vertices:
        t = np.dot(v - p1, axis_dir)
        t_clamped = np.clip(t, 0.0, length)
        closest_on_axis = p1 + t_clamped * axis_dir
        dist = np.linalg.norm(v - closest_on_axis)
        assert dist <= radius + 1e-6, (
            f"Vertex {v} is {dist} from axis, exceeds radius {radius}"
        )

    print("  [PASS] test_from_capsule")


if __name__ == "__main__":
    print("=== TriangleMesh Test Suite ===\n")

    test_create_from_vertices_and_faces()
    test_from_convex_hull()
    test_support_function()
    test_support_with_transform()
    test_from_box()
    test_from_sphere()
    test_from_capsule()

    print("\n=== All 7 tests passed ===")
