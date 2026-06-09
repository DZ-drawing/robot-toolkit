from robot_ik.collision.mesh import TriangleMesh
from robot_ik.collision.module import (
    Box,
    Capsule,
    CollisionChecker,
    CollisionResult,
    GeometryType,
    Sphere,
    distance_box_to_box,
    distance_capsule_to_capsule,
    distance_point_to_box,
    distance_point_to_sphere,
    distance_sphere_to_box,
    distance_sphere_to_capsule,
    distance_sphere_to_sphere,
)

try:
    from robot_ik.collision.epa import epa_penetration
    from robot_ik.collision.gjk import gjk_distance, gjk_intersect
except ImportError:
    gjk_intersect = None  # type: ignore[assignment]
    gjk_distance = None  # type: ignore[assignment]
    epa_penetration = None  # type: ignore[assignment]

__all__ = [
    "Box",
    "Capsule",
    "CollisionChecker",
    "CollisionResult",
    "GeometryType",
    "Sphere",
    "TriangleMesh",
    "distance_box_to_box",
    "distance_capsule_to_capsule",
    "distance_point_to_box",
    "distance_point_to_sphere",
    "distance_sphere_to_box",
    "distance_sphere_to_capsule",
    "distance_sphere_to_sphere",
    "gjk_intersect",
    "gjk_distance",
    "epa_penetration",
]
