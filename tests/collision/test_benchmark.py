"""Performance benchmarks for collision detection.

These tests measure per-call latency for various collision operations.
They are skipped by default; run with --run-benchmark to enable:
    python -m pytest tests/collision/test_benchmark.py -v --run-benchmark
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np
import pytest

from robot_ik.collision import (
    Box,
    Sphere,
    TriangleMesh,
    distance_sphere_to_sphere,
)
from robot_ik.collision.epa import epa_penetration
from robot_ik.collision.gjk import gjk_distance, gjk_intersect

# Skip all benchmarks unless --run-benchmark is passed
pytestmark = pytest.mark.skipif(
    "not config.getoption('--run-benchmark', default=False)",
    reason="benchmarks require --run-benchmark flag",
)

N_ITERATIONS = 1000
MAX_MESH_COLLISION_US = 10_000  # 10 ms in microseconds


@dataclass
class _BenchResult:
    """Simple holder for benchmark results."""

    name: str
    us_per_call: float
    n: int


def _report(result: _BenchResult) -> None:
    """Print benchmark result for visibility in test output."""
    print(
        f"\n[bench] {result.name}: {result.us_per_call:.1f} us/call "
        f"({result.n} iterations, {result.us_per_call * result.n / 1e3:.1f} ms total)"
    )


# ======================================================================
# Tier 1: Primitive-primitive collision
# ======================================================================


class TestPrimitivePrimitiveBenchmark:
    """Benchmark sphere-sphere collision (Tier 1)."""

    def test_sphere_vs_sphere_intersection(self):
        """Benchmark distance_sphere_to_sphere (analytical)."""
        s1 = Sphere(radius=0.05, pose=np.eye(4))
        s2 = Sphere(radius=0.05, pose=np.eye(4))
        t0 = time.perf_counter()
        for _ in range(N_ITERATIONS):
            distance_sphere_to_sphere(s1, s2)
        elapsed = time.perf_counter() - t0
        us = elapsed / N_ITERATIONS * 1e6
        result = _BenchResult("Sphere vs Sphere (analytical)", us, N_ITERATIONS)
        _report(result)
        assert us < 100, f"Sphere-sphere too slow: {us:.1f} us"

    def test_sphere_vs_sphere_overlapping(self):
        """Benchmark overlapping sphere-sphere check."""
        s1 = Sphere(radius=0.05, pose=np.eye(4))
        pose2 = np.eye(4)
        pose2[0, 3] = 0.06  # 6cm apart, radii 5cm each -> overlap
        s2 = Sphere(radius=0.05, pose=pose2)
        t0 = time.perf_counter()
        for _ in range(N_ITERATIONS):
            distance_sphere_to_sphere(s1, s2)
        elapsed = time.perf_counter() - t0
        us = elapsed / N_ITERATIONS * 1e6
        result = _BenchResult("Sphere vs Sphere (overlapping)", us, N_ITERATIONS)
        _report(result)
        assert us < 100, f"Sphere-sphere too slow: {us:.1f} us"


# ======================================================================
# Tier 2: Mesh-mesh collision via GJK
# ======================================================================


class TestMeshMeshBenchmark:
    """Benchmark mesh-mesh collision using GJK (Tier 2)."""

    def test_box_vs_box_gjk_intersect_separated(self):
        """Benchmark GJK intersection for two separated box meshes."""
        box_a = TriangleMesh.from_box([0.1, 0.1, 0.1])
        box_a.update_pose(np.eye(4))

        pose_b = np.eye(4)
        pose_b[0, 3] = 0.2  # 20cm apart -> separated
        box_b = TriangleMesh.from_box([0.1, 0.1, 0.1])
        box_b.update_pose(pose_b)

        t0 = time.perf_counter()
        for _ in range(N_ITERATIONS):
            gjk_intersect(box_a, box_b)
        elapsed = time.perf_counter() - t0
        us = elapsed / N_ITERATIONS * 1e6
        result = _BenchResult("Box vs Box GJK intersect (separated)", us, N_ITERATIONS)
        _report(result)
        assert us < MAX_MESH_COLLISION_US, f"Mesh collision too slow: {us:.1f} us"

    def test_box_vs_box_gjk_intersect_overlapping(self):
        """Benchmark GJK intersection for two overlapping box meshes."""
        box_a = TriangleMesh.from_box([0.1, 0.1, 0.1])
        box_a.update_pose(np.eye(4))

        pose_b = np.eye(4)
        pose_b[0, 3] = 0.05  # 5cm apart -> overlapping
        box_b = TriangleMesh.from_box([0.1, 0.1, 0.1])
        box_b.update_pose(pose_b)

        t0 = time.perf_counter()
        for _ in range(N_ITERATIONS):
            gjk_intersect(box_a, box_b)
        elapsed = time.perf_counter() - t0
        us = elapsed / N_ITERATIONS * 1e6
        result = _BenchResult("Box vs Box GJK intersect (overlapping)", us, N_ITERATIONS)
        _report(result)
        assert us < MAX_MESH_COLLISION_US, f"Mesh collision too slow: {us:.1f} us"

    def test_box_vs_box_gjk_distance(self):
        """Benchmark GJK distance for two separated box meshes."""
        box_a = TriangleMesh.from_box([0.1, 0.1, 0.1])
        box_a.update_pose(np.eye(4))

        pose_b = np.eye(4)
        pose_b[0, 3] = 0.3
        box_b = TriangleMesh.from_box([0.1, 0.1, 0.1])
        box_b.update_pose(pose_b)

        t0 = time.perf_counter()
        for _ in range(N_ITERATIONS):
            gjk_distance(box_a, box_b)
        elapsed = time.perf_counter() - t0
        us = elapsed / N_ITERATIONS * 1e6
        result = _BenchResult("Box vs Box GJK distance", us, N_ITERATIONS)
        _report(result)
        assert us < MAX_MESH_COLLISION_US, f"Mesh distance too slow: {us:.1f} us"


# ======================================================================
# EPA penetration benchmark
# ======================================================================


class TestEPABenchmark:
    """Benchmark EPA penetration computation."""

    def test_epa_box_penetration(self):
        """Benchmark EPA penetration depth for overlapping boxes."""
        box_a = TriangleMesh.from_box([0.1, 0.1, 0.1])
        box_a.update_pose(np.eye(4))

        pose_b = np.eye(4)
        pose_b[0, 3] = 0.05  # small overlap
        box_b = TriangleMesh.from_box([0.1, 0.1, 0.1])
        box_b.update_pose(pose_b)

        n_iter = 200  # EPA is heavier; fewer iterations
        t0 = time.perf_counter()
        for _ in range(n_iter):
            depth, normal, contact = epa_penetration(box_a, box_b)
            # Sanity check
            assert depth > 0.0
        elapsed = time.perf_counter() - t0
        us = elapsed / n_iter * 1e6
        result = _BenchResult("EPA penetration (box-box)", us, n_iter)
        _report(result)
        # EPA is more expensive in pure Python; allow up to 100ms
        assert us < 100_000, f"EPA too slow: {us:.1f} us"


# ======================================================================
# to_mesh() conversion benchmark
# ======================================================================


class TestToMeshBenchmark:
    """Benchmark to_mesh() conversion from primitives to TriangleMesh."""

    def test_sphere_to_mesh(self):
        """Benchmark Sphere.to_mesh()."""
        s = Sphere(radius=0.05)
        t0 = time.perf_counter()
        for _ in range(N_ITERATIONS):
            s.to_mesh(subdivisions=1)
        elapsed = time.perf_counter() - t0
        us = elapsed / N_ITERATIONS * 1e6
        result = _BenchResult("Sphere.to_mesh() (subdiv=1)", us, N_ITERATIONS)
        _report(result)
        assert us < 1000, f"to_mesh too slow: {us:.1f} us"

    def test_box_to_mesh(self):
        """Benchmark Box.to_mesh()."""
        b = Box(size=np.array([0.1, 0.1, 0.1]))
        t0 = time.perf_counter()
        for _ in range(N_ITERATIONS):
            b.to_mesh()
        elapsed = time.perf_counter() - t0
        us = elapsed / N_ITERATIONS * 1e6
        result = _BenchResult("Box.to_mesh()", us, N_ITERATIONS)
        _report(result)
        assert us < 1000, f"to_mesh too slow: {us:.1f} us"
