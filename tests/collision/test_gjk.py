"""Test suite for GJK (Gilbert-Johnson-Keerthi) collision algorithm."""

import numpy as np
import pytest

from robot_ik.collision.gjk import (
    _closest_on_line,
    _closest_on_triangle,
    _segment_distance,
    _simplex_distance,
    _triple_product,
    _triangle_distance,
    gjk_distance,
    gjk_intersect,
)
from robot_ik.collision.mesh import TriangleMesh


# ======================================================================
# Helper-function tests
# ======================================================================


class TestTripleProduct:
    def test_cross_cross_identity(self):
        a = np.array([1.0, 0.0, 0.0])
        b = np.array([0.0, 1.0, 0.0])
        c = np.array([0.0, 0.0, 1.0])
        # (a x b) x c  =  b(a.c) - a(b.c)  =  0 - 0 = 0
        result = _triple_product(a, b, c)
        np.testing.assert_allclose(result, [0, 0, 0], atol=1e-12)

    def test_non_trivial(self):
        a = np.array([1.0, 2.0, 3.0])
        b = np.array([4.0, 5.0, 6.0])
        c = np.array([7.0, 8.0, 9.0])
        expected = np.cross(np.cross(a, b), c)
        np.testing.assert_allclose(_triple_product(a, b, c), expected)


class TestClosestOnLine:
    def test_perpendicular(self):
        origin = np.zeros(3)
        direction = np.array([1.0, 0.0, 0.0])
        point = np.array([0.0, 5.0, 0.0])
        closest = _closest_on_line(origin, direction, point)
        np.testing.assert_allclose(closest, [0, 0, 0], atol=1e-12)

    def test_along_direction(self):
        origin = np.zeros(3)
        direction = np.array([1.0, 0.0, 0.0])
        point = np.array([3.0, 0.0, 0.0])
        closest = _closest_on_line(origin, direction, point)
        np.testing.assert_allclose(closest, [3, 0, 0], atol=1e-12)

    def test_offset_line(self):
        origin = np.array([1.0, 1.0, 1.0])
        direction = np.array([0.0, 0.0, 1.0])
        point = np.array([1.0, 3.0, 5.0])
        closest = _closest_on_line(origin, direction, point)
        np.testing.assert_allclose(closest, [1, 1, 5], atol=1e-12)


class TestClosestOnTriangle:
    def test_vertex_a(self):
        a = np.array([1.0, 0.0, 0.0])
        b = np.array([0.0, 1.0, 0.0])
        c = np.array([0.0, 0.0, 1.0])
        pt, bary = _closest_on_triangle(a, b, c, origin=np.array([2.0, -1.0, -1.0]))
        np.testing.assert_allclose(bary, [1, 0, 0], atol=1e-10)
        np.testing.assert_allclose(pt, a, atol=1e-10)

    def test_vertex_b(self):
        a = np.array([1.0, 0.0, 0.0])
        b = np.array([0.0, 1.0, 0.0])
        c = np.array([0.0, 0.0, 1.0])
        pt, bary = _closest_on_triangle(a, b, c, origin=np.array([-1.0, 2.0, -1.0]))
        np.testing.assert_allclose(bary, [0, 1, 0], atol=1e-10)
        np.testing.assert_allclose(pt, b, atol=1e-10)

    def test_vertex_c(self):
        a = np.array([1.0, 0.0, 0.0])
        b = np.array([0.0, 1.0, 0.0])
        c = np.array([0.0, 0.0, 1.0])
        pt, bary = _closest_on_triangle(a, b, c, origin=np.array([-1.0, -1.0, 2.0]))
        np.testing.assert_allclose(bary, [0, 0, 1], atol=1e-10)
        np.testing.assert_allclose(pt, c, atol=1e-10)

    def test_edge_ab(self):
        a = np.array([1.0, 0.0, 0.0])
        b = np.array([0.0, 1.0, 0.0])
        c = np.array([0.0, 0.0, 1.0])
        # Point directly above midpoint of AB
        mid_ab = (a + b) / 2.0
        pt, bary = _closest_on_triangle(a, b, c, origin=mid_ab + np.array([0, 0, -1.0]))
        np.testing.assert_allclose(bary[:2], [0.5, 0.5], atol=1e-10)
        assert bary[2] < 1e-10

    def test_face_interior(self):
        # Equilateral-ish triangle with origin at centroid
        a = np.array([1.0, 0.0, 0.0])
        b = np.array([-1.0, 1.0, 0.0])
        c = np.array([-1.0, -1.0, 0.0])
        centroid = (a + b + c) / 3.0
        pt, bary = _closest_on_triangle(a, b, c, origin=centroid)
        np.testing.assert_allclose(pt, centroid, atol=1e-10)
        assert abs(sum(bary) - 1.0) < 1e-10
        assert all(w > 0 for w in bary)

    def test_barycentric_sums_to_one(self):
        a = np.array([1.0, 0.0, 0.0])
        b = np.array([0.0, 1.0, 0.0])
        c = np.array([0.0, 0.0, 1.0])
        for query in [np.array([0.1, 0.2, 0.3]),
                       np.array([-1, -1, -1]),
                       np.array([0.5, 0.5, 0.5])]:
            _, bary = _closest_on_triangle(a, b, c, origin=query)
            assert abs(sum(bary) - 1.0) < 1e-10


# ======================================================================
# Simplex distance helpers
# ======================================================================


class TestSegmentDistance:
    def test_closest_at_interior(self):
        """Segment [-1,0,0]→[1,0,0], origin at [0,0,0] → midpoint."""
        simplex = [(np.array([-1.0, 0.0, 0.0]), None, None),
                   (np.array([1.0, 0.0, 0.0]), None, None)]
        pt, bary = _segment_distance(simplex)
        np.testing.assert_allclose(pt, [0, 0, 0], atol=1e-10)
        np.testing.assert_allclose(bary, [0.5, 0.5], atol=1e-10)

    def test_closest_at_vertex_a(self):
        """Segment [2,0,0]→[5,0,0], origin at [0,0,0] → vertex A."""
        simplex = [(np.array([2.0, 0.0, 0.0]), None, None),
                   (np.array([5.0, 0.0, 0.0]), None, None)]
        pt, bary = _segment_distance(simplex)
        np.testing.assert_allclose(pt, [2, 0, 0], atol=1e-10)
        np.testing.assert_allclose(bary, [1, 0], atol=1e-10)

    def test_closest_at_vertex_b(self):
        """Segment [-5,0,0]→[-2,0,0], origin at [0,0,0] → vertex B."""
        simplex = [(np.array([-5.0, 0.0, 0.0]), None, None),
                   (np.array([-2.0, 0.0, 0.0]), None, None)]
        pt, bary = _segment_distance(simplex)
        np.testing.assert_allclose(pt, [-2, 0, 0], atol=1e-10)
        np.testing.assert_allclose(bary, [0, 1], atol=1e-10)


class TestTriangleDistance:
    def test_origin_inside(self):
        a = np.array([1.0, 0.0, 0.0])
        b = np.array([-1.0, 0.0, 0.0])
        c = np.array([0.0, 1.0, 0.0])
        simplex = [(a, None, None), (b, None, None), (c, None, None)]
        pt, bary = _triangle_distance(simplex)
        np.testing.assert_allclose(pt, [0, 0, 0], atol=1e-10)
        assert all(w >= -1e-10 for w in bary)


# ======================================================================
# GJK intersection tests
# ======================================================================


class TestGJKIntersect:
    def test_gjk_separated_spheres(self):
        """Two spheres 0.5 apart should NOT intersect."""
        s1 = TriangleMesh.from_sphere(0.1, subdivisions=1)
        s2 = TriangleMesh.from_sphere(0.1, subdivisions=1)
        pose2 = np.eye(4)
        pose2[0, 3] = 0.7
        s2.update_pose(pose2)
        assert gjk_intersect(s1, s2) is False

    def test_gjk_overlapping(self):
        """Two overlapping spheres → intersecting."""
        s1 = TriangleMesh.from_sphere(0.1, subdivisions=1)
        s2 = TriangleMesh.from_sphere(0.1, subdivisions=1)
        pose2 = np.eye(4)
        pose2[0, 3] = 0.1
        s2.update_pose(pose2)
        assert gjk_intersect(s1, s2) is True

    def test_gjk_touching(self):
        """Two spheres exactly touching → distance ≈ 0, intersecting."""
        s1 = TriangleMesh.from_sphere(0.1, subdivisions=1)
        s2 = TriangleMesh.from_sphere(0.1, subdivisions=1)
        pose2 = np.eye(4)
        pose2[0, 3] = 0.2  # 2 * radius
        s2.update_pose(pose2)
        # Polyhedral approximation means touching ≈ intersecting
        assert gjk_intersect(s1, s2) is True

    def test_gjk_box_separation(self):
        """Two boxes separated by 1 unit → not intersecting."""
        b1 = TriangleMesh.from_box(np.array([1.0, 1.0, 1.0]))
        b2 = TriangleMesh.from_box(np.array([1.0, 1.0, 1.0]))
        pose2 = np.eye(4)
        pose2[0, 3] = 3.0
        b2.update_pose(pose2)
        assert gjk_intersect(b1, b2) is False

    def test_gjk_box_overlapping(self):
        """Two overlapping boxes → intersecting."""
        b1 = TriangleMesh.from_box(np.array([1.0, 1.0, 1.0]))
        b2 = TriangleMesh.from_box(np.array([1.0, 1.0, 1.0]))
        pose2 = np.eye(4)
        pose2[0, 3] = 0.5
        b2.update_pose(pose2)
        assert gjk_intersect(b1, b2) is True

    def test_gjk_not_intersecting_separated(self):
        """Far-apart spheres → not intersecting."""
        s1 = TriangleMesh.from_sphere(0.1, subdivisions=1)
        s2 = TriangleMesh.from_sphere(0.1, subdivisions=1)
        pose2 = np.eye(4)
        pose2[0, 3] = 5.0
        s2.update_pose(pose2)
        assert gjk_intersect(s1, s2) is False

    def test_gjk_coincident(self):
        """Identical shapes at same position → intersecting."""
        s1 = TriangleMesh.from_sphere(0.1, subdivisions=1)
        s2 = TriangleMesh.from_sphere(0.1, subdivisions=1)
        assert gjk_intersect(s1, s2) is True

    def test_gjk_large_sphere_offset(self):
        """Large sphere offset in Y — not intersecting."""
        s1 = TriangleMesh.from_sphere(1.0, subdivisions=2)
        s2 = TriangleMesh.from_sphere(0.5, subdivisions=2)
        pose2 = np.eye(4)
        pose2[1, 3] = 3.0
        s2.update_pose(pose2)
        assert gjk_intersect(s1, s2) is False


# ======================================================================
# GJK distance tests
# ======================================================================


class TestGJKDistance:
    def test_gjk_separated_spheres(self):
        """Two spheres 0.5 apart → distance ≈ 0.5."""
        s1 = TriangleMesh.from_sphere(0.1, subdivisions=1)
        s2 = TriangleMesh.from_sphere(0.1, subdivisions=1)
        pose2 = np.eye(4)
        pose2[0, 3] = 0.7
        s2.update_pose(pose2)
        distance, c1, c2 = gjk_distance(s1, s2)
        assert abs(distance - 0.5) < 0.05, f"Expected ~0.5, got {distance}"
        assert c1 is not None and c2 is not None

    def test_gjk_touching(self):
        """Two spheres exactly touching → distance ≈ 0."""
        s1 = TriangleMesh.from_sphere(0.1, subdivisions=1)
        s2 = TriangleMesh.from_sphere(0.1, subdivisions=1)
        pose2 = np.eye(4)
        pose2[0, 3] = 0.2  # 2 * radius
        s2.update_pose(pose2)
        distance, _, _ = gjk_distance(s1, s2)
        assert abs(distance) < 0.05, f"Expected ~0, got {distance}"

    def test_gjk_overlapping_distance_zero(self):
        """Two overlapping spheres → distance = 0."""
        s1 = TriangleMesh.from_sphere(0.1, subdivisions=1)
        s2 = TriangleMesh.from_sphere(0.1, subdivisions=1)
        pose2 = np.eye(4)
        pose2[0, 3] = 0.1
        s2.update_pose(pose2)
        distance, c1, c2 = gjk_distance(s1, s2)
        assert distance == 0.0
        assert c1 is None and c2 is None

    def test_gjk_box_separation(self):
        """Two boxes separated by 1 unit → distance > 0."""
        b1 = TriangleMesh.from_box(np.array([1.0, 1.0, 1.0]))
        b2 = TriangleMesh.from_box(np.array([1.0, 1.0, 1.0]))
        pose2 = np.eye(4)
        pose2[0, 3] = 3.0
        b2.update_pose(pose2)
        distance, _, _ = gjk_distance(b1, b2)
        assert distance > 0
        # Boxes are 1x1x1, gap = 3 - 0.5 - 0.5 = 2.0
        assert abs(distance - 2.0) < 0.1, f"Expected ~2.0, got {distance}"

    def test_gjk_coincident_distance(self):
        """Identical shapes at same position → distance = 0."""
        s1 = TriangleMesh.from_sphere(0.1, subdivisions=1)
        s2 = TriangleMesh.from_sphere(0.1, subdivisions=1)
        distance, c1, c2 = gjk_distance(s1, s2)
        assert distance == 0.0
        assert c1 is None and c2 is None

    def test_gjk_far_separation(self):
        """Far-apart shapes → large distance."""
        b1 = TriangleMesh.from_box(np.array([0.5, 0.5, 0.5]))
        b2 = TriangleMesh.from_box(np.array([0.5, 0.5, 0.5]))
        pose2 = np.eye(4)
        pose2[2, 3] = 10.0
        b2.update_pose(pose2)
        distance, _, _ = gjk_distance(b1, b2)
        # gap = 10 - 0.25 - 0.25 = 9.5
        assert abs(distance - 9.5) < 0.1, f"Expected ~9.5, got {distance}"

    def test_gjk_sphere_box_distance(self):
        """Sphere near a box → positive distance."""
        box = TriangleMesh.from_box(np.array([2.0, 2.0, 2.0]))
        sphere = TriangleMesh.from_sphere(0.5, subdivisions=2)
        pose_sphere = np.eye(4)
        pose_sphere[0, 3] = 2.5  # center at x=2.5
        sphere.update_pose(pose_sphere)
        # box extends to x=1, sphere center at 2.5, radius 0.5
        # gap = 2.5 - 0.5 - 1.0 = 1.0
        distance, _, _ = gjk_distance(box, sphere)
        assert abs(distance - 1.0) < 0.1, f"Expected ~1.0, got {distance}"

    def test_closest_points_on_correct_side(self):
        """Closest points should be on the facing sides of each shape."""
        s1 = TriangleMesh.from_sphere(0.1, subdivisions=2)
        s2 = TriangleMesh.from_sphere(0.1, subdivisions=2)
        pose2 = np.eye(4)
        pose2[0, 3] = 0.7
        s2.update_pose(pose2)
        distance, c1, c2 = gjk_distance(s1, s2)
        # c1 should be on the +x side of s1 (toward s2)
        # c2 should be on the -x side of s2 (toward s1)
        assert c1[0] > 0.0, f"c1 x={c1[0]}, expected positive"
        assert c2[0] < 0.65, f"c2 x={c2[0]}, expected < 0.65 (toward s1)"

    def test_distance_y_separation(self):
        """Spheres separated along Y axis."""
        s1 = TriangleMesh.from_sphere(0.5, subdivisions=2)
        s2 = TriangleMesh.from_sphere(0.5, subdivisions=2)
        pose2 = np.eye(4)
        pose2[1, 3] = 2.0
        s2.update_pose(pose2)
        distance, _, _ = gjk_distance(s1, s2)
        # gap = 2.0 - 0.5 - 0.5 = 1.0
        assert abs(distance - 1.0) < 0.1, f"Expected ~1.0, got {distance}"


# ======================================================================
# Integration tests (intersection + distance consistency)
# ======================================================================


class TestGJKConsistency:
    def test_intersect_implies_zero_distance(self):
        """If gjk_intersect returns True, gjk_distance should be 0."""
        s1 = TriangleMesh.from_sphere(0.2, subdivisions=1)
        s2 = TriangleMesh.from_sphere(0.2, subdivisions=1)
        pose2 = np.eye(4)
        pose2[0, 3] = 0.15
        s2.update_pose(pose2)
        assert gjk_intersect(s1, s2) is True
        distance, _, _ = gjk_distance(s1, s2)
        assert distance == 0.0

    def test_no_intersect_implies_positive_distance(self):
        """If gjk_intersect returns False, gjk_distance should be > 0."""
        s1 = TriangleMesh.from_sphere(0.2, subdivisions=1)
        s2 = TriangleMesh.from_sphere(0.2, subdivisions=1)
        pose2 = np.eye(4)
        pose2[0, 3] = 1.5
        s2.update_pose(pose2)
        assert gjk_intersect(s1, s2) is False
        distance, _, _ = gjk_distance(s1, s2)
        assert distance > 0

    def test_multiple_directions(self):
        """Test separation along X, Y, and Z axes."""
        b1 = TriangleMesh.from_box(np.array([1.0, 1.0, 1.0]))
        b2 = TriangleMesh.from_box(np.array([1.0, 1.0, 1.0]))
        for axis in range(3):
            pose2 = np.eye(4)
            pose2[axis, 3] = 4.0
            b2.update_pose(pose2)
            assert gjk_intersect(b1, b2) is False
            distance, _, _ = gjk_distance(b1, b2)
            # gap = 4 - 0.5 - 0.5 = 3.0
            assert abs(distance - 3.0) < 0.2, (
                f"Axis {axis}: expected ~3.0, got {distance}"
            )


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
