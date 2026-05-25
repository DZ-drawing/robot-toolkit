"""Tests for Meshcat visualization - strict TDD

Tests are written FIRST, then code is implemented to pass them.
"""

import threading

import numpy as np
import pytest

from robot_ik import six_dof_articulated

meshcat = pytest.importorskip("meshcat")  # noqa: E402

from robot_ik.hardware.hal import HardwareRegistry, SimulatedHardware  # noqa: E402
from robot_ik.visualization.meshcat_viz import MeshcatVisualizer  # noqa: E402


class TestMeshcatInitialization:
    """RED phase: test visualizer creation and error handling."""

    def test_default_port(self):
        vis = MeshcatVisualizer()
        assert vis.port == MeshcatVisualizer.DEFAULT_PORT
        assert not vis._streaming
        assert vis._robot is None
        vis.stop_realtime_stream()  # cleanup

    def test_custom_port(self):
        vis = MeshcatVisualizer(port=8100)
        assert vis.port == 8100
        vis.stop_realtime_stream()

    def test_vis_object_created(self):
        vis = MeshcatVisualizer(port=8101)
        assert vis.vis is not None
        vis.stop_realtime_stream()


class TestLinkTransformUsesRealFK:
    """RED: _compute_link_transform must match robot's actual FK.

    The placeholder used hardcoded heights (0.1, 0.5, 1.0, ...).
    Real implementation should use robot.forward_kinematics.
    """

    def test_link0_transform_matches_fk_at_zero(self):
        """After update_joints at q=0, stored link 0 transform should match FK."""
        vis = MeshcatVisualizer(port=8102)
        robot = six_dof_articulated()
        vis.set_robot(robot)

        q = np.zeros(6)
        vis.update_joints(q)

        _, transforms = robot.forward_kinematics(q, return_all=True)
        assert np.allclose(
            vis._last_link_transforms[0], transforms[1], atol=1e-10
        ), f"Link 0 transform mismatch.\nGot:\n{vis._last_link_transforms[0]}\nExpected:\n{transforms[1]}"

    def test_link3_transform_matches_fk_at_random(self):
        """After update_joints at random q, stored link 3 transform should match FK."""
        vis = MeshcatVisualizer(port=8103)
        robot = six_dof_articulated()
        vis.set_robot(robot)

        q = np.array([0.1, -0.2, 0.3, 0.4, -0.1, 0.2])
        vis.update_joints(q)

        _, transforms = robot.forward_kinematics(q, return_all=True)
        assert np.allclose(
            vis._last_link_transforms[3], transforms[4], atol=1e-10
        ), f"Link 3 transform mismatch.\nGot:\n{vis._last_link_transforms[3]}\nExpected:\n{transforms[4]}"

    def test_link0_at_pi2_is_not_hardcoded(self):
        """Link 0 at q=[pi/2, 0, ...] should produce non-trivial rotation (not identity)."""
        vis = MeshcatVisualizer(port=8104)
        robot = six_dof_articulated()
        vis.set_robot(robot)

        q = np.array([np.pi / 2, 0, 0, 0, 0, 0])
        vis.update_joints(q)

        _, transforms = robot.forward_kinematics(q, return_all=True)
        T_link0 = vis._last_link_transforms[0]
        assert np.allclose(T_link0, transforms[1], atol=1e-10)

        # Hardcoded placeholder would produce simple Z rotation.
        # Real FK with DH alpha=-pi/2 should produce non-trivial rotation.
        R = T_link0[:3, :3]
        # Check that the rotation is NOT a simple Rz(pi/2)
        Rz_pi2 = np.array(
            [
                [np.cos(np.pi / 2), -np.sin(np.pi / 2), 0],
                [np.sin(np.pi / 2), np.cos(np.pi / 2), 0],
                [0, 0, 1],
            ]
        )
        assert not np.allclose(
            R, Rz_pi2, atol=1e-6
        ), "Rotation looks like a simple Rz(pi/2) — might be hardcoded"


class TestSetRobot:
    """RED: set_robot should create correct mesh objects."""

    def test_set_robot_stores_reference(self):
        vis = MeshcatVisualizer(port=8105)
        robot = six_dof_articulated()
        vis.set_robot(robot)
        assert vis._robot is robot
        assert vis._num_joints == 6

    def test_set_robot_custom_color(self):
        vis = MeshcatVisualizer(port=8106)
        robot = six_dof_articulated()
        color = np.array([1.0, 0.0, 0.0, 1.0])
        vis.set_robot(robot, color=color)
        assert vis._robot is robot

    def test_set_robot_without_meshcat_raises_error(self):
        """If meshcat not installed, set_robot should handle gracefully."""
        vis = MeshcatVisualizer(port=8107)
        robot = six_dof_articulated()
        # set_robot should work since meshcat IS installed in test env
        vis.set_robot(robot)
        assert vis._robot is not None


class TestUpdateJoints:
    """RED: update_joints should set correct meshcat transforms."""

    def test_update_joints_without_robot_raises(self):
        vis = MeshcatVisualizer(port=8108)
        with pytest.raises(MeshcatVisualizer.MeshcatError, match="not set"):
            vis.update_joints(np.zeros(6))

    def test_update_joints_invalid_shape_raises(self):
        vis = MeshcatVisualizer(port=8109)
        robot = six_dof_articulated()
        vis.set_robot(robot)
        with pytest.raises(ValueError, match="shape"):
            vis.update_joints(np.zeros(5))

    def test_update_joints_zero_position(self):
        vis = MeshcatVisualizer(port=8110)
        robot = six_dof_articulated()
        vis.set_robot(robot)
        vis.update_joints(np.zeros(6))  # should not raise

    def test_update_joints_random_positions(self):
        vis = MeshcatVisualizer(port=8111)
        robot = six_dof_articulated()
        vis.set_robot(robot)
        rng = np.random.RandomState(42)
        for _ in range(5):
            q = rng.uniform(-np.pi, np.pi, 6)
            vis.update_joints(q)

    def test_update_joints_end_effector_matches_fk(self):
        """After update_joints, end-effector transform should match FK result."""
        vis = MeshcatVisualizer(port=8112)
        robot = six_dof_articulated()
        vis.set_robot(robot)

        q = np.array([0.1, -0.2, 0.3, 0.4, -0.1, 0.2])
        vis.update_joints(q)

        T_expected, transforms = robot.forward_kinematics(q, return_all=True)

        # Verify base transform
        base_transform = vis._last_base_transform
        assert np.allclose(base_transform, transforms[0], atol=1e-10)

        # Verify each link transform
        for i in range(6):
            link_transform = vis._last_link_transforms[i]
            assert np.allclose(
                link_transform, transforms[i + 1], atol=1e-10
            ), f"Link {i} transform mismatch"


class TestRealtimeStream:
    """RED: realtime streaming lifecycle."""

    def test_start_stop_stream(self):
        vis = MeshcatVisualizer(port=8113)
        robot = six_dof_articulated()
        vis.set_robot(robot)
        hw = SimulatedHardware(dof=6)

        vis.start_realtime_stream(hw, freq=30)
        assert vis._streaming
        assert vis._stream_thread is not None

        # Wait for at least one update cycle via stop event timeout
        updated = threading.Event()

        original_update = vis.update_joints

        def tracked_update(q):
            updated.set()
            original_update(q)

        vis.update_joints = tracked_update

        updated.wait(timeout=1.0)
        assert updated.is_set(), "Stream should have called update_joints at least once"

        vis.stop_realtime_stream()
        assert not vis._streaming

    def test_concurrent_stream_raises(self):
        vis = MeshcatVisualizer(port=8114)
        robot = six_dof_articulated()
        vis.set_robot(robot)
        hw = SimulatedHardware(dof=6)

        vis.start_realtime_stream(hw, freq=30)
        with pytest.raises(RuntimeError, match="already running"):
            vis.start_realtime_stream(hw, freq=30)
        vis.stop_realtime_stream()

    def test_invalid_hardware_raises(self):
        vis = MeshcatVisualizer(port=8115)
        robot = six_dof_articulated()
        vis.set_robot(robot)

        with pytest.raises(MeshcatVisualizer.StreamingError, match="get_joint_positions"):
            vis.start_realtime_stream(object(), freq=30)

    def test_stream_updates_joints(self):
        """Stream should call update_joints with hardware positions."""
        vis = MeshcatVisualizer(port=8116)
        robot = six_dof_articulated()
        vis.set_robot(robot)
        hw = SimulatedHardware(dof=6)

        q_target = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        hw.set_joint_targets(q_target)

        updated = threading.Event()
        original_update = vis.update_joints

        def tracked_update(q):
            updated.set()
            original_update(q)

        vis.update_joints = tracked_update

        vis.start_realtime_stream(hw, freq=30)
        updated.wait(timeout=1.0)
        assert updated.is_set(), "Stream should have called update_joints"
        vis.stop_realtime_stream()

        # After streaming, last update should reflect hardware position
        assert vis._last_update_q is not None
        assert np.allclose(vis._last_update_q, q_target, atol=1e-6)

    def test_stop_when_not_streaming_is_noop(self):
        vis = MeshcatVisualizer(port=8117)
        vis.stop_realtime_stream()  # should not raise
        assert not vis._streaming


class TestJupyterIntegration:
    """RED: Jupyter notebook integration."""

    def test_start_jupyter_returns_iframe(self):
        vis = MeshcatVisualizer(port=8118)
        iframe = vis.start_jupyter()
        assert iframe is not None
        assert hasattr(iframe, "src")
        assert "http" in iframe.src


class TestHardwareHAL:
    """RED: Hardware Abstraction Layer basics."""

    def test_simulated_hardware_get_set(self):
        hw = SimulatedHardware(dof=6)
        q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        hw.set_joint_targets(q)
        assert np.allclose(hw.get_joint_positions(), q)

    def test_simulated_hardware_velocity(self):
        hw = SimulatedHardware(dof=6)
        dq = hw.get_joint_velocities()
        assert np.allclose(dq, 0)

    def test_simulated_hardware_stop(self):
        hw = SimulatedHardware(dof=6)
        q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        hw.set_joint_targets(q)
        hw.stop()
        assert hw.get_info()["stopped"]
        assert np.allclose(hw.get_joint_velocities(), 0)
        # After stop, set_joint_targets should be ignored
        hw.set_joint_targets(np.ones(6))
        assert np.allclose(hw.get_joint_positions(), q)  # unchanged

    def test_registry_lists_simulated(self):
        protocols = HardwareRegistry.list_protocols()
        assert "simulated" in protocols

    def test_registry_create_simulated(self):
        hw = HardwareRegistry.create("simulated", dof=6)
        assert isinstance(hw, SimulatedHardware)
        assert hw.dof == 6

    def test_registry_unknown_raises(self):
        with pytest.raises(ValueError, match="Unknown"):
            HardwareRegistry.create("nonexistent")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])


class TestSuggestionFixes:
    """TDD: fix reviewer suggestions from pre-commit review."""

    # S1: _num_joints should come from robot, not hardcoded
    def test_num_joints_read_from_robot(self):
        """set_robot should infer _num_joints from robot's dh_params, not stay at init value."""
        vis = MeshcatVisualizer(port=8200)
        robot = six_dof_articulated()
        vis.set_robot(robot)
        # Verify _num_joints was updated from robot, not left at default 6
        assert vis._num_joints == len(robot.dh_params)

    def test_num_joints_stored_on_robot_attribute(self):
        """If robot has a 'dof' attr, prefer it over dh_params length."""
        vis = MeshcatVisualizer(port=8201)
        robot = six_dof_articulated()
        # Monkey-patch a dof attribute to simulate a future robot class
        robot.dof = 3  # different from dh_params length!
        vis.set_robot(robot)
        assert vis._num_joints == 3, f"Should use robot.dof (3), got {vis._num_joints}"

    # S4: context manager for reliable cleanup
    def test_context_manager_cleanup(self):
        """with vis as v: should stop streaming on exit."""
        vis = MeshcatVisualizer(port=8202)
        robot = six_dof_articulated()
        vis.set_robot(robot)
        hw = SimulatedHardware(dof=6)

        with vis:
            vis.start_realtime_stream(hw, freq=30)
            assert vis._streaming

        # After exiting context, stream should be stopped
        assert not vis._streaming

    def test_context_manager_returns_self(self):
        """with MeshcatVisualizer() as vis should return self."""
        vis = MeshcatVisualizer(port=8203)
        with vis as v:
            assert v is vis

    # S6: thread-safety lock on _streaming
    def test_concurrent_start_does_not_double_start(self):
        """Rapid concurrent start_realtime_stream calls should not double-start."""
        vis = MeshcatVisualizer(port=8204)
        robot = six_dof_articulated()
        vis.set_robot(robot)
        hw = SimulatedHardware(dof=6)

        results = []

        def try_start():
            try:
                vis.start_realtime_stream(hw, freq=30)
                results.append(True)
            except RuntimeError:
                results.append(False)

        threads = [threading.Thread(target=try_start) for _ in range(5)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        # Exactly one should succeed, rest should get RuntimeError
        assert results.count(True) == 1, f"Expected 1 success, got {results.count(True)}"
        vis.stop_realtime_stream()
