"""Tests for EPA (Expanding Polytope Algorithm)."""

import numpy as np
import pytest

scipy = pytest.importorskip("scipy")

from robot_ik.collision.epa import epa_penetration  # noqa: E402
from robot_ik.collision.gjk import gjk_intersect  # noqa: E402
from robot_ik.collision.mesh import TriangleMesh  # noqa: E402


class TestEPASphereOverlap:
    """EPA on overlapping icosphere meshes."""

    def test_epa_sphere_overlap_positive_depth(self):
        s1 = TriangleMesh.from_sphere(0.1, subdivisions=2)
        s2 = TriangleMesh.from_sphere(0.1, subdivisions=2)
        pose2 = np.eye(4)
        pose2[0, 3] = 0.15
        s2.update_pose(pose2)

        assert gjk_intersect(s1, s2), "precondition: shapes must intersect"
        depth, normal, contact = epa_penetration(s1, s2)
        assert depth > 0
        assert np.linalg.norm(normal) > 0.9
        assert contact is not None

    def test_epa_sphere_depth_reasonable(self):
        s1 = TriangleMesh.from_sphere(0.1, subdivisions=2)
        s2 = TriangleMesh.from_sphere(0.1, subdivisions=2)
        pose2 = np.eye(4)
        pose2[0, 3] = 0.15
        s2.update_pose(pose2)

        depth, _, _ = epa_penetration(s1, s2)
        # Overlap ≈ 2*0.1 - 0.15 = 0.05 (mesh approximation, allow wide tolerance)
        assert depth > 0.01


class TestEPABoxOverlap:
    """EPA on overlapping box meshes."""

    def test_epa_box_deep_penetration(self):
        b1 = TriangleMesh.from_box(np.array([1.0, 1.0, 1.0]))
        b2 = TriangleMesh.from_box(np.array([1.0, 1.0, 1.0]))
        pose2 = np.eye(4)
        pose2[0, 3] = 0.5
        b2.update_pose(pose2)

        assert gjk_intersect(b1, b2)
        depth, normal, contact = epa_penetration(b1, b2)
        # Overlap = 2*0.5 - 0.5 = 0.5 (mesh approximation)
        assert depth >= 0.45
        # Normal should be along x-axis (magnitude close to 1)
        assert abs(abs(normal[0]) - 1.0) < 0.3


class TestEPAConsistency:
    """Consistency checks across different configurations."""

    def test_intersect_implies_positive_depth(self):
        """If GJK says intersect, EPA depth must be > 0."""
        b1 = TriangleMesh.from_box(np.array([0.5, 0.5, 0.5]))
        b2 = TriangleMesh.from_box(np.array([0.5, 0.5, 0.5]))
        pose2 = np.eye(4)
        pose2[0, 3] = 0.3
        b2.update_pose(pose2)

        if gjk_intersect(b1, b2):
            depth, _, _ = epa_penetration(b1, b2)
            assert depth > 0

    def test_depth_axis_aligned(self):
        """Penetration along x produces normal aligned with x."""
        b1 = TriangleMesh.from_box(np.array([0.5, 0.5, 0.5]))
        b2 = TriangleMesh.from_box(np.array([0.5, 0.5, 0.5]))
        pose2 = np.eye(4)
        pose2[0, 3] = 0.3
        b2.update_pose(pose2)

        if gjk_intersect(b1, b2):
            depth, normal, _ = epa_penetration(b1, b2)
            assert depth > 0
            # Normal should be primarily along x
            assert abs(normal[0]) > 0.5

    def test_y_axis_overlap(self):
        b1 = TriangleMesh.from_box(np.array([0.4, 0.4, 0.4]))
        b2 = TriangleMesh.from_box(np.array([0.4, 0.4, 0.4]))
        pose2 = np.eye(4)
        pose2[1, 3] = 0.3
        b2.update_pose(pose2)

        if gjk_intersect(b1, b2):
            depth, normal, _ = epa_penetration(b1, b2)
            assert depth > 0.1
            # Normal should be primarily along y
            assert abs(normal[1]) > 0.5


class TestEPANonIntersection:
    """EPA should gracefully handle non-intersecting shapes."""

    def test_no_intersection_returns_zero(self):
        b1 = TriangleMesh.from_box(np.array([0.5, 0.5, 0.5]))
        b2 = TriangleMesh.from_box(np.array([0.5, 0.5, 0.5]))
        pose2 = np.eye(4)
        pose2[0, 3] = 5.0
        b2.update_pose(pose2)

        assert not gjk_intersect(b1, b2)
        depth, _, _ = epa_penetration(b1, b2)
        assert depth == 0.0
