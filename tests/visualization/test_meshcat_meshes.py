"""Tests for real mesh loading (STL/OBJ) in MeshcatVisualizer — strict TDD.

RED phase: these tests are written FIRST; mesh loading is then implemented
in robot_ik.visualization.meshcat_viz to pass them.
"""

import struct
from pathlib import Path

import numpy as np
import pytest

meshcat = pytest.importorskip("meshcat")  # noqa: E402

from robot_ik import six_dof_articulated  # noqa: E402
from robot_ik.visualization.meshcat_viz import MeshcatVisualizer  # noqa: E402


def make_binary_stl(path: Path, num_triangles: int = 4) -> Path:
    """Write a minimal valid binary STL file (80-byte header + n triangles)."""
    data = bytearray(b"robot-toolkit test stl" + b"\0" * 59)  # 80 bytes
    data += struct.pack("<I", num_triangles)
    for _ in range(num_triangles):
        data += struct.pack("<3f", 0.0, 0.0, 1.0)  # normal
        data += struct.pack("<9f", 0, 0, 0, 1, 0, 0, 0, 1, 0)  # 3 vertices
        data += struct.pack("<H", 0)  # attribute byte count
    path.write_bytes(bytes(data))
    return path


def make_obj(path: Path) -> Path:
    """Write a minimal valid OBJ file (one quad as two triangles)."""
    path.write_text(
        "# robot-toolkit test obj\n"
        "v 0 0 0\n"
        "v 1 0 0\n"
        "v 1 1 0\n"
        "v 0 1 0\n"
        "f 1 2 3\n"
        "f 1 3 4\n"
    )
    return path


class TestLoadLinkMeshes:
    """RED phase: per-link real mesh loading API."""

    def test_load_stl_link_mesh(self, tmp_path):
        vis = MeshcatVisualizer()
        try:
            robot = six_dof_articulated()
            vis.set_robot(robot)
            stl = make_binary_stl(tmp_path / "link0.stl")
            vis.load_link_meshes({0: stl})
            assert vis._link_mesh_paths == {0: str(stl)}
            # geometry was produced by meshcat's own STL loader
            assert vis._link_mesh_geometries[0].mesh_format == "stl"
        finally:
            vis.stop_realtime_stream()

    def test_load_obj_link_mesh(self, tmp_path):
        vis = MeshcatVisualizer()
        try:
            robot = six_dof_articulated()
            vis.set_robot(robot)
            obj = make_obj(tmp_path / "link2.obj")
            vis.load_link_meshes({2: obj})
            assert vis._link_mesh_paths == {2: str(obj)}
        finally:
            vis.stop_realtime_stream()

    def test_load_mesh_without_robot_raises(self, tmp_path):
        vis = MeshcatVisualizer()
        try:
            stl = make_binary_stl(tmp_path / "link0.stl")
            with pytest.raises(MeshcatVisualizer.MeshcatError, match="[Rr]obot not set"):
                vis.load_link_meshes({0: stl})
        finally:
            vis.stop_realtime_stream()

    def test_load_mesh_missing_file_raises(self, tmp_path):
        vis = MeshcatVisualizer()
        try:
            robot = six_dof_articulated()
            vis.set_robot(robot)
            with pytest.raises(FileNotFoundError):
                vis.load_link_meshes({0: tmp_path / "nonexistent.stl"})
        finally:
            vis.stop_realtime_stream()

    def test_load_mesh_unsupported_extension_raises(self, tmp_path):
        vis = MeshcatVisualizer()
        try:
            robot = six_dof_articulated()
            vis.set_robot(robot)
            bad = tmp_path / "link0.ply"
            bad.write_text("ply\n")
            with pytest.raises(ValueError, match="[Uu]nsupported"):
                vis.load_link_meshes({0: bad})
        finally:
            vis.stop_realtime_stream()

    def test_load_mesh_bad_link_index_raises(self, tmp_path):
        vis = MeshcatVisualizer()
        try:
            robot = six_dof_articulated()
            vis.set_robot(robot)
            stl = make_binary_stl(tmp_path / "link9.stl")
            with pytest.raises(IndexError):
                vis.load_link_meshes({9: stl})
        finally:
            vis.stop_realtime_stream()

    def test_meshes_persist_across_update_joints(self, tmp_path):
        """Loaded meshes must not be clobbered by update_joints (FK transforms)."""
        vis = MeshcatVisualizer()
        try:
            robot = six_dof_articulated()
            vis.set_robot(robot)
            stl = make_binary_stl(tmp_path / "link0.stl")
            vis.load_link_meshes({0: stl})
            vis.update_joints(np.zeros(6))
            vis.update_joints(np.array([0.1, -0.2, 0.3, 0.5, -0.4, 0.2]))
            assert vis._link_mesh_paths == {0: str(stl)}
            # transform still applied to the mesh-carrying node
            assert len(vis._last_link_transforms) == 6
        finally:
            vis.stop_realtime_stream()

    def test_mesh_scale_option(self, tmp_path):
        """Optional scale keyword resizes the mesh geometry."""
        vis = MeshcatVisualizer()
        try:
            robot = six_dof_articulated()
            vis.set_robot(robot)
            stl = make_binary_stl(tmp_path / "link0.stl")
            vis.load_link_meshes({0: stl}, scale=2.0)
            assert vis._link_mesh_scales == {0: 2.0}
        finally:
            vis.stop_realtime_stream()


class TestLoadEnvironmentMesh:
    """RED phase: static environment/obstacle mesh loading."""

    def test_load_env_mesh_stl(self, tmp_path):
        vis = MeshcatVisualizer()
        try:
            stl = make_binary_stl(tmp_path / "table.stl")
            vis.load_environment_mesh("table", stl, position=np.array([0.5, 0.0, -0.1]))
            assert vis._env_meshes == {"table": str(stl)}
            assert np.allclose(vis._env_positions["table"], [0.5, 0.0, -0.1])
        finally:
            vis.stop_realtime_stream()

    def test_load_env_mesh_obj(self, tmp_path):
        vis = MeshcatVisualizer()
        try:
            obj = make_obj(tmp_path / "floor.obj")
            vis.load_environment_mesh("floor", obj)
            assert vis._env_meshes["floor"] == str(obj)
            # default position = origin
            assert np.allclose(vis._env_positions["floor"], [0, 0, 0])
        finally:
            vis.stop_realtime_stream()

    def test_load_env_mesh_missing_file_raises(self, tmp_path):
        vis = MeshcatVisualizer()
        try:
            with pytest.raises(FileNotFoundError):
                vis.load_environment_mesh("ghost", tmp_path / "nope.stl")
        finally:
            vis.stop_realtime_stream()

    def test_load_env_mesh_unsupported_ext_raises(self, tmp_path):
        vis = MeshcatVisualizer()
        try:
            bad = tmp_path / "ghost.ply"
            bad.write_text("ply\n")
            with pytest.raises(ValueError, match="[Uu]nsupported"):
                vis.load_environment_mesh("ghost", bad)
        finally:
            vis.stop_realtime_stream()

    def test_load_env_mesh_accepts_transform_matrix(self, tmp_path):
        vis = MeshcatVisualizer()
        try:
            stl = make_binary_stl(tmp_path / "part.stl")
            T = np.eye(4)
            T[:3, 3] = [0.2, 0.3, 0.4]
            vis.load_environment_mesh("part", stl, transform=T)
            assert np.allclose(vis._env_transforms["part"], T)
        finally:
            vis.stop_realtime_stream()
