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
from pathlib import Path

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
    - Context manager for reliable cleanup

    Example:
        >>> from robot_ik import MeshcatVisualizer, six_dof_articulated
        >>> with MeshcatVisualizer() as vis:
        ...     robot = six_dof_articulated()
        ...     vis.set_robot(robot)
        ...     vis.update_joints(np.zeros(6))
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
        self._stream_lock = threading.Lock()

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
                "meshcat package not installed. " "Install with: pip install robot-ik[meshcat]"
            ) from None
        except Exception as e:
            raise self.InitializationError(f"Meshcat initialization failed: {e}") from e

        # Robot reference
        self._robot = None
        self._num_joints = 6

        # Real-mesh tracking (populated by load_link_meshes/load_environment_mesh)
        self._link_mesh_paths: dict[int, str] = {}
        self._link_mesh_scales: dict[int, float] = {}
        self._link_mesh_geometries: dict[int, object] = {}
        self._env_meshes: dict[str, str] = {}
        self._env_positions: dict[str, np.ndarray] = {}
        self._env_transforms: dict[str, np.ndarray] = {}

    # Mesh geometry factory (STL/OBJ are passed as raw file contents;
    # scale is applied via the material-independent geometry wrapper)
    _MESH_EXTS = (".stl", ".obj")

    def _mesh_geometry(self, path, scale: float | None = None):
        """Load a mesh file into a meshcat geometry.

        Uses meshcat's own from_file loaders (the StlMeshGeometry/ObjMeshGeometry
        constructors are broken in meshcat 0.3.2 — they pass too many args to
        super().__init__). Scale, when != 1.0, is returned separately so the
        caller can bake it into a child-node transform.

        Returns:
            (geometry, scale_factor) tuple

        Raises:
            FileNotFoundError: If path does not exist
            ValueError: If extension is not .stl or .obj
        """
        import meshcat.geometry as mg

        p = Path(path)
        if not p.is_file():
            raise FileNotFoundError(f"Mesh file not found: {p}")
        ext = p.suffix.lower()
        if ext == ".stl":
            geom = mg.StlMeshGeometry.from_file(str(p))
        elif ext == ".obj":
            geom = mg.ObjMeshGeometry.from_file(str(p))
        else:
            raise ValueError(f"Unsupported mesh format '{ext}'. Use .stl or .obj")
        return geom, (1.0 if scale is None else float(scale))

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_realtime_stream()
        return False

    def set_robot(self, robot, color: np.ndarray | None = None):
        """Set robot model (procedural generation)

        Creates 3D meshes using simple geometries:
        - Base: Box
        - Links 1-N: Cylinders
        - Joints 1-N: Spheres
        - End-effector: Coordinate frame (Triad)

        Args:
            robot: Robot model instance with forward_kinematics and dh_params
            color: Link color [R, G, B, A] (optional, default blue)
        """
        import meshcat.geometry as mg

        self._robot = robot

        # Infer _num_joints from robot (prefer explicit dof attr, else dh_params)
        if hasattr(robot, "dof"):
            self._num_joints = robot.dof
        elif hasattr(robot, "dh_params"):
            self._num_joints = len(robot.dh_params)
        else:
            self._num_joints = 6  # fallback default

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

    def load_link_meshes(
        self, meshes: dict[int, object], scale: float | dict | None = None
    ) -> None:
        """Replace procedural link geometry with real meshes (STL/OBJ).

        Each mesh is attached to its link's scene node, so subsequent
        update_joints() calls keep animating it via forward kinematics.

        Args:
            meshes: Mapping {link_index: path} of mesh files to load
            scale: Optional uniform scale factor, or a {link_index: factor}
                dict (default 1.0 for unspecified links)

        Raises:
            MeshcatError: If robot not set
            IndexError: If a link index is out of range
            FileNotFoundError: If a mesh file does not exist
            ValueError: If the mesh extension is unsupported
        """
        import meshcat.geometry as mg

        if self._robot is None:
            raise self.MeshcatError("Robot not set. Call set_robot() first.")

        scale_map: dict[int, float] = {}
        if isinstance(scale, dict):
            scale_map = {int(k): float(v) for k, v in scale.items()}
        elif scale is not None:
            scale_map = {i: float(scale) for i in range(self._num_joints)}

        for link_idx, path in meshes.items():
            link_idx = int(link_idx)
            if not 0 <= link_idx < self._num_joints:
                raise IndexError(
                    f"Link index {link_idx} out of range for " f"{self._num_joints}-joint robot"
                )
            s = scale_map.get(link_idx, 1.0)
            geom, s = self._mesh_geometry(path, s)
            # Geometry lives on a child node; update_joints() only writes the
            # parent link node's transform, so the scale matrix is preserved.
            self.vis[f"link{link_idx}/mesh"].set_object(
                geom, mg.MeshLambertMaterial(color=self.DEFAULT_COLOR)
            )
            if s != 1.0:
                S = np.eye(4) * s
                S[3, 3] = 1.0
                self.vis[f"link{link_idx}/mesh"].set_transform(S)
            self._link_mesh_paths[link_idx] = str(path)
            self._link_mesh_scales[link_idx] = s
            self._link_mesh_geometries[link_idx] = geom
            logger.info(f"Link {link_idx} mesh loaded from {path} (scale={s})")

    def load_environment_mesh(
        self,
        name: str,
        path,
        position: np.ndarray | None = None,
        transform: np.ndarray | None = None,
        color=None,
    ) -> None:
        """Add a static environment mesh (table, floor, obstacles) to the scene.

        Args:
            name: Scene name for the mesh (e.g. "table", "floor")
            path: Path to .stl or .obj file
            position: [x, y, z] placement (default origin)
            transform: 4x4 pose matrix; overrides position if given
            color: Optional [R, G, B, A] material color

        Raises:
            FileNotFoundError: If the mesh file does not exist
            ValueError: If the mesh extension is unsupported
        """
        import meshcat.geometry as mg

        geom = self._mesh_geometry(path)[0]
        material = mg.MeshLambertMaterial(
            color=list(np.asarray(color).flatten()) if color is not None else None
        )
        self.vis[f"environment/{name}"].set_object(geom, material)

        if transform is not None:
            T = np.asarray(transform, dtype=float)
            if T.shape != (4, 4):
                raise ValueError(f"transform must be 4x4, got {T.shape}")
        else:
            T = np.eye(4)
            if position is not None:
                T[:3, 3] = np.asarray(position, dtype=float).flatten()
        self.vis[f"environment/{name}"].set_transform(T)

        self._env_meshes[name] = str(path)
        self._env_positions[name] = T[:3, 3].copy()
        self._env_transforms[name] = T.copy()
        logger.info(f"Environment mesh '{name}' loaded from {path}")

    def update_joints(self, q: np.ndarray):
        """Update joint angles using robot's forward kinematics.

        Args:
            q: Joint angles (num_joints,)

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

        Thread-safe: uses a lock to prevent concurrent starts.

        Args:
            hardware: HardwareInterface instance
            freq: Update frequency in Hz (default 30)

        Raises:
            RuntimeError: If stream already running
            StreamingError: If hardware interface invalid
        """
        with self._stream_lock:
            if self._streaming:
                raise RuntimeError("Real-time stream already running")

            if not hasattr(hardware, "get_joint_positions"):
                raise self.StreamingError(
                    "Invalid hardware interface. " "Must have get_joint_positions() method."
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

        self._stream_thread = threading.Thread(target=update_loop, daemon=True)
        self._stream_thread.start()
        logger.info(f"Real-time stream started at {freq} Hz")

    def stop_realtime_stream(self):
        """Stop real-time monitoring stream"""
        with self._stream_lock:
            if not self._streaming:
                return

            self._streaming = False
            self._stop_event.set()

        if self._stream_thread:
            self._stream_thread.join(timeout=5.0)

        logger.info("Real-time stream stopped")

    def __del__(self):
        """Cleanup on deletion"""
        if self._streaming:
            self.stop_realtime_stream()
