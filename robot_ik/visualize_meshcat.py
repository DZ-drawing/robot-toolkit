"""
Meshcat 3D Visualization for robot-toolkit

This module provides web-based 3D visualization using Meshcat, supporting:
- Interactive development in Jupyter notebooks
- Real-time monitoring (30Hz+ streaming)
- Procedural robot model generation (simple geometries)
- Optional real mesh loading (STL/OBJ)
"""

import logging
import threading

import numpy as np

logger = logging.getLogger(__name__)


class MeshcatVisualizer:
    """Meshcat 3D visualizer for robot manipulators

    Features:
    - Web-based visualization (browser access)
    - Jupyter notebook native support
    - Real-time streaming (30Hz+)
    - Procedural 3D model generation (default)
    - Optional real mesh loading

    Example:
        >>> from robot_ik import MeshcatVisualizer, six_dof_articulated
        >>> vis = MeshcatVisualizer()
        >>> robot = six_dof_articulated()
        >>> vis.set_robot(robot)
        >>> vis.update_joints(np.zeros(6))
    """

    # Configuration
    DEFAULT_PORT = 7000
    DEFAULT_FREQ = 30

    # Robot model config (procedural generation)
    BASE_SIZE = [0.1, 0.1, 0.1]
    LINK_RADIUS = 0.05
    JOINT_RADIUS = 0.06
    DEFAULT_COLOR = [0.3, 0.6, 0.9, 1.0]  # RGBA

    # Exception classes
    class MeshcatError(Exception):
        """Base exception for Meshcat visualization errors"""
        pass

    class StreamingError(MeshcatError):
        """Real-time streaming error"""
        pass

    class InitializationError(MeshcatError):
        """Initialization error"""
        pass

    def __init__(self, port: int = DEFAULT_PORT, zmq_url: str | None = None):
        """Initialize Meshcat visualizer

        Args:
            port: Meshcat server port (default 7000)
            zmq_url: ZeroMQ URL (optional, auto-generated if None)

        Raises:
            InitializationError: If Meshcat initialization fails
        """
        self.port = port
        self._streaming = False
        self._stop_event = None
        self._stream_thread = None

        # Tracking attributes for testability
        self._last_base_transform = np.eye(4)
        self._last_link_transforms: list[np.ndarray] = []
        self._last_update_q: np.ndarray | None = None

        try:
            import meshcat

            if zmq_url:
                self.vis = meshcat.Visualizer(zmq_url=zmq_url)
            else:
                self.vis = meshcat.Visualizer()
            logger.info(f"Meshcat visualizer initialized on port {port}")
        except ImportError:
            raise self.InitializationError(
                "meshcat package not installed. "
                "Install with: pip install robot-ik[meshcat]"
            ) from None
        except Exception as e:
            raise self.InitializationError(f"Meshcat initialization failed: {e}") from e

        # Robot reference
        self._robot = None
        self._num_joints = 6

    def set_robot(self, robot, color: np.ndarray | None = None):
        """Set robot model (procedural generation)

        Creates 3D meshes using simple geometries:
        - Base: Box
        - Links 1-6: Cylinders
        - Joints 1-6: Spheres
        - End-effector: Coordinate frame (Triad)

        Args:
            robot: six_dof_articulated instance
            color: Link color [R, G, B, A] (optional, default blue)
        """
        import meshcat.geometry as mg

        self._robot = robot
        if color is None:
            color = self.DEFAULT_COLOR
        else:
            color = list(np.asarray(color).flatten())

        try:
            # Base
            self.vis["base"].set_object(
                mg.Box(self.BASE_SIZE),
                mg.MeshLambertMaterial(color=color),
            )

            # Links and joints
            for i in range(self._num_joints):
                # Link (cylinder)
                self.vis[f"link{i}"].set_object(
                    mg.Cylinder(0.5, self.LINK_RADIUS),
                    mg.MeshLambertMaterial(color=color),
                )

                # Joint (sphere)
                self.vis[f"joint{i}"].set_object(
                    mg.Sphere(self.JOINT_RADIUS),
                    mg.MeshLambertMaterial(color=[0.4, 0.4, 0.4, 1.0]),
                )

                # Coordinate frame
                self.vis[f"frame{i}"].set_object(mg.triad())

            # End-effector frame
            self.vis["end_effector"].set_object(mg.triad())

            logger.info("Robot model created successfully")

        except Exception as e:
            raise self.MeshcatError(f"Failed to create robot model: {e}") from e

    def update_joints(self, q: np.ndarray):
        """Update joint angles using robot's forward kinematics.

        Args:
            q: Joint angles (6,)

        Raises:
            MeshcatError: If robot not set or invalid q
        """
        if self._robot is None:
            raise self.MeshcatError("Robot not set. Call set_robot() first.")

        q = np.asarray(q)
        if q.shape != (self._num_joints,):
            raise ValueError(f"Expected q shape ({self._num_joints},), got {q.shape}")

        try:
            # Compute all transforms via FK
            _, transforms = self._robot.forward_kinematics(q, return_all=True)

            # Store for testability
            self._last_base_transform = transforms[0].copy()
            self._last_link_transforms = [t.copy() for t in transforms[1:]]
            self._last_update_q = q.copy()

            # Update base
            self.vis["base"].set_transform(transforms[0])

            # Update each link, joint, and frame
            for i in range(self._num_joints):
                T_i = transforms[i + 1]
                self.vis[f"link{i}"].set_transform(T_i)
                self.vis[f"joint{i}"].set_transform(T_i)
                self.vis[f"frame{i}"].set_transform(T_i)

            # Update end-effector (last transform)
            self.vis["end_effector"].set_transform(transforms[-1])

        except Exception as e:
            raise self.MeshcatError(f"Failed to update joints: {e}") from e

    def start_jupyter(self):
        """Display in Jupyter Notebook

        Returns:
            IPython.display.IFrame for inline display

        Raises:
            MeshcatError: If not in Jupyter environment
        """
        try:
            from IPython.display import IFrame
        except ImportError:
            raise self.MeshcatError(
                "IPython not available. Install with: pip install ipython"
            ) from None

        # Try newer API first, fall back to URL construction
        try:
            url = self.vis.viewer_url()
        except AttributeError:
            # Older meshcat versions use .url()
            url = getattr(self.vis, "url", lambda: f"http://127.0.0.1:{self.port}/static/")()
        logger.info(f"Jupyter display URL: {url}")

        return IFrame(src=url, width=800, height=600)

    def start_realtime_stream(self, hardware, freq: int = DEFAULT_FREQ):
        """Start real-time monitoring stream (background thread)

        Args:
            hardware: HardwareInterface instance
            freq: Update frequency in Hz (default 30)

        Raises:
            RuntimeError: If stream already running
            StreamingError: If hardware interface invalid
        """
        if self._streaming:
            raise RuntimeError("Real-time stream already running")

        if not hasattr(hardware, "get_joint_positions"):
            raise self.StreamingError(
                "Invalid hardware interface. "
                "Must have get_joint_positions() method."
            )

        self._streaming = True
        self._stop_event = threading.Event()

        def update_loop():
            """Background update loop"""
            while not self._stop_event.is_set():
                try:
                    q = hardware.get_joint_positions()
                    self.update_joints(q)
                except Exception as e:
                    logger.error(f"Real-time update error: {e}")

                self._stop_event.wait(1.0 / freq)

        self._stream_thread = threading.Thread(
            target=update_loop, daemon=True
        )
        self._stream_thread.start()
        logger.info(f"Real-time stream started at {freq} Hz")

    def stop_realtime_stream(self):
        """Stop real-time monitoring stream"""
        if not self._streaming:
            return

        self._stop_event.set()

        if self._stream_thread:
            self._stream_thread.join(timeout=5.0)

        self._streaming = False
        logger.info("Real-time stream stopped")

    def __del__(self):
        """Cleanup on deletion"""
        if self._streaming:
            self.stop_realtime_stream()
