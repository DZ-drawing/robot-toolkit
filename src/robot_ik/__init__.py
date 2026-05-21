"""robot-ik — Fast 6-DOF Inverse Kinematics and Rigid Body Dynamics

C++ accelerated robotics toolkit with Python API.
"""

from robot_ik.collision import (
    Box,
    Capsule,
    CollisionChecker,
    CollisionResult,
    Sphere,
)
from robot_ik.dynamics import (
    DynamicsSolver,
    LinkInertia,
    RobotDynamicsModel,
    six_dof_articulated_dyn,
)
from robot_ik.ik import (
    DHParam,
    RobotModel,
    dh_transform,
    six_dof_articulated,
    spherical_wrist_6dof,
)
from robot_ik.path_planning import (
    PathPlanningResult,
    RRTStar,
    plan_path_rrt_star,
)
from robot_ik.trajectory import (
    TrajectoryResult,
    cartesian_straight_line,
    joint_cubic_interpolation,
    joint_linear_interpolation,
    joint_quintic_interpolation,
    s_curve_profile,
    trapezoidal_velocity_profile,
    waypoint_trajectory,
)
from robot_ik.urdf import (
    quick_urdf,
    urdf_to_dynamics_model,
)

# Try to import C++ extensions (optional)
try:
    import importlib.util as _iu

    _iu.find_spec("robot_ik.ik_fast")  # noqa: F841
    HAS_IK_FAST = True
except (ImportError, ModuleNotFoundError):
    HAS_IK_FAST = False

try:
    import importlib.util as _iu

    _iu.find_spec("robot_ik.robot_dyn_fast")  # noqa: F841
    HAS_DYN_FAST = True
except (ImportError, ModuleNotFoundError):
    HAS_DYN_FAST = False


__version__ = "0.3.0"
__all__ = [
    "DHParam",
    "RobotModel",
    "dh_transform",
    "six_dof_articulated",
    "spherical_wrist_6dof",
    "LinkInertia",
    "RobotDynamicsModel",
    "DynamicsSolver",
    "six_dof_articulated_dyn",
    "urdf_to_dynamics_model",
    "quick_urdf",
    "TrajectoryResult",
    "joint_linear_interpolation",
    "joint_cubic_interpolation",
    "joint_quintic_interpolation",
    "cartesian_straight_line",
    "trapezoidal_velocity_profile",
    "s_curve_profile",
    "waypoint_trajectory",
    "Sphere",
    "Capsule",
    "Box",
    "CollisionChecker",
    "CollisionResult",
    "RRTStar",
    "PathPlanningResult",
    "plan_path_rrt_star",
    "HAS_IK_FAST",
    "HAS_DYN_FAST",
]
