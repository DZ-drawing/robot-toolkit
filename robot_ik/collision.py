"""Collision Detection for Robot Manipulators.

Simple geometry-based collision detection using primitive shapes:
- Spheres (for joints)
- Capsules (for links)
- Boxes (for base, environment obstacles)

Uses efficient distance computations and bounding volume hierarchies.

Author: Danny Zeng
License: MIT
"""

from dataclasses import dataclass, field
from enum import Enum

import numpy as np


class GeometryType(Enum):
    """Primitive geometry types for collision shapes."""

    SPHERE = "sphere"
    CAPSULE = "capsule"
    BOX = "box"


@dataclass
class Sphere:
    """Sphere defined by center and radius."""

    radius: float = 0.05
    pose: np.ndarray = field(default_factory=lambda: np.eye(4))


@dataclass
class Capsule:
    """Capsule defined by line segment (p1, p2) and radius."""

    p1: np.ndarray = field(default_factory=lambda: np.zeros(3))
    p2: np.ndarray = field(default_factory=lambda: np.array([0, 0, 0.1]))
    radius: float = 0.04
    pose: np.ndarray = field(default_factory=lambda: np.eye(4))


@dataclass
class Box:
    """Box defined by center and dimensions (x, y, z)."""

    size: np.ndarray = field(default_factory=lambda: np.array([0.1, 0.1, 0.1]))
    pose: np.ndarray = field(default_factory=lambda: np.eye(4))


@dataclass
class CollisionResult:
    """Result of collision check."""

    is_colliding: bool
    distance: float  # Minimum distance (negative if penetrating)
    contact_point: np.ndarray | None  # Contact point if colliding
    pair: tuple[str, str]  # Names of colliding pair


def distance_point_to_sphere(point: np.ndarray, sphere: Sphere) -> float:
    """Distance from point to sphere surface."""
    center = sphere.pose[:3, 3]
    return np.linalg.norm(point - center) - sphere.radius


def distance_point_to_box(point: np.ndarray, box: Box) -> float:
    """Distance from point to box surface (signed)."""
    # Transform point to box local frame
    R = box.pose[:3, :3]
    t = box.pose[:3, 3]
    point_local = R.T @ (point - t)

    half_size = box.size / 2

    # Find closest point on box
    closest = np.zeros(3)
    for i in range(3):
        closest[i] = np.clip(point_local[i], -half_size[i], half_size[i])

    # Distance to closest point
    dist_local = np.linalg.norm(point_local - closest)

    # Check if point is inside
    inside = np.all(np.abs(point_local) <= half_size)

    # Return signed distance (negative if inside)
    if inside:
        # Find penetration depth
        penetration = np.min(half_size - np.abs(point_local))
        return -penetration
    else:
        return dist_local


def distance_sphere_to_sphere(s1: Sphere, s2: Sphere) -> float:
    """Distance between two spheres."""
    c1 = s1.pose[:3, 3]
    c2 = s2.pose[:3, 3]
    center_dist = np.linalg.norm(c1 - c2)
    return center_dist - s1.radius - s2.radius


def distance_sphere_to_capsule(sphere: Sphere, capsule: Capsule) -> float:
    """Distance between sphere and capsule."""
    # Closest point on capsule line segment to sphere center
    sphere_center = sphere.pose[:3, 3]
    p1 = capsule.p1
    p2 = capsule.p2

    # Project sphere center onto line
    v = p2 - p1
    if np.linalg.norm(v) < 1e-10:
        # Capsule is a point
        closest = p1
    else:
        t = np.dot(sphere_center - p1, v) / np.dot(v, v)
        t = np.clip(t, 0, 1)
        closest = p1 + t * v

    dist = np.linalg.norm(sphere_center - closest)
    return dist - sphere.radius - capsule.radius


def distance_capsule_to_capsule(c1: Capsule, c2: Capsule) -> float:
    """Distance between two capsules."""
    # Closest points between two line segments
    p1, p2 = c1.p1, c1.p2
    p3, p4 = c2.p1, c2.p2

    v1 = p2 - p1
    v2 = p4 - p3

    # Handle degenerate cases
    if np.linalg.norm(v1) < 1e-10 and np.linalg.norm(v2) < 1e-10:
        dist = np.linalg.norm(p1 - p3)
    elif np.linalg.norm(v1) < 1e-10:
        t = np.dot(p1 - p3, v2) / np.dot(v2, v2)
        t = np.clip(t, 0, 1)
        closest2 = p3 + t * v2
        dist = np.linalg.norm(p1 - closest2)
    elif np.linalg.norm(v2) < 1e-10:
        t = np.dot(p3 - p1, v1) / np.dot(v1, v1)
        t = np.clip(t, 0, 1)
        closest1 = p1 + t * v1
        dist = np.linalg.norm(closest1 - p3)
    else:
        # Segment-segment distance
        # Using analytical solution for closest points on two line segments
        w = p1 - p3
        a = np.dot(v1, v1)
        b = np.dot(v1, v2)
        c = np.dot(v2, v2)
        d = np.dot(v1, w)
        e = np.dot(v2, w)
        denom = a * c - b * b

        if abs(denom) < 1e-10:
            # Parallel segments
            t = 0
        else:
            t = (b * e - c * d) / denom
            t = np.clip(t, 0, 1)

        s = (b * t + e) / c if c > 1e-10 else 0
        s = np.clip(s, 0, 1)

        # Recompute t with clipped s
        if abs(denom) < 1e-10:
            t = 0
        else:
            t = (b * s + d) / a if a > 1e-10 else 0
            t = np.clip(t, 0, 1)

        closest1 = p1 + t * v1
        closest2 = p3 + s * v2
        dist = np.linalg.norm(closest1 - closest2)

    return dist - c1.radius - c2.radius


def distance_sphere_to_box(sphere: Sphere, box: Box) -> float:
    """Distance between sphere and box."""
    sphere_center = sphere.pose[:3, 3]

    # Transform sphere center to box local frame
    R = box.pose[:3, :3]
    t = box.pose[:3, 3]
    point_local = R.T @ (sphere_center - t)

    half_size = box.size / 2

    # Find closest point on box
    closest = np.zeros(3)
    for i in range(3):
        closest[i] = np.clip(point_local[i], -half_size[i], half_size[i])

    # Distance in local frame
    dist_local = np.linalg.norm(point_local - closest)

    # Check if sphere center is inside box
    inside = np.all(np.abs(point_local) <= half_size + sphere.radius)

    if inside:
        # Penetration depth
        penetration = sphere.radius - np.linalg.norm(point_local - closest)
        return -penetration
    else:
        return dist_local - sphere.radius


def distance_box_to_box(b1: Box, b2: Box) -> float:
    """Distance between two boxes (OBB)."""
    # Simplified: using axis-aligned bounding boxes in world frame
    # For full OBB-OBB distance, use separating axis theorem
    c1 = b1.pose[:3, 3]
    c2 = b2.pose[:3, 3]

    # Use radius approximation (conservative)
    r1 = np.linalg.norm(b1.size) / 2
    r2 = np.linalg.norm(b2.size) / 2

    return np.linalg.norm(c1 - c2) - r1 - r2


class CollisionChecker:
    """Collision detection for robot manipulators."""

    def __init__(self):
        self.link_geometries: dict[str, list] = {}
        self.obstacles: list = []

    def add_link_geometry(self, link_name: str, geometry):
        """Add collision geometry for a robot link."""
        if link_name not in self.link_geometries:
            self.link_geometries[link_name] = []
        self.link_geometries[link_name].append(geometry)

    def add_obstacle(self, obstacle):
        """Add environment obstacle."""
        self.obstacles.append(obstacle)

    def check_self_collision(
        self,
        link_transforms: dict[str, np.ndarray],
        ignore_adjacent: bool = True,
    ) -> CollisionResult | None:
        """Check for self-collision between robot links.

        Args:
            link_transforms: Dictionary mapping link names to 4x4 transforms.
            ignore_adjacent: If True, ignore collision between adjacent links.

        Returns:
            CollisionResult if collision detected, None otherwise.
        """
        # Check all pairs
        link_names = list(self.link_geometries.keys())
        for i, name1 in enumerate(link_names):
            for j, name2 in enumerate(link_names[i + 1 :], start=i + 1):
                # Skip adjacent links (by index in the kinematic chain)
                if ignore_adjacent and j - i <= 1:
                    continue

                for geom1 in self.link_geometries[name1]:
                    for geom2 in self.link_geometries[name2]:
                        # Apply transforms temporarily
                        T1 = link_transforms.get(name1, np.eye(4))
                        T2 = link_transforms.get(name2, np.eye(4))

                        # Create temporary geometries with applied transforms
                        import copy

                        g1 = copy.deepcopy(geom1)
                        g2 = copy.deepcopy(geom2)
                        g1.pose = T1 @ g1.pose
                        g2.pose = T2 @ g2.pose

                        result = self._check_geometry_collision(g1, g2)
                        if result.is_colliding:
                            result.pair = (name1, name2)
                            return result

        return None

    def check_environment_collision(
        self,
        link_transforms: dict[str, np.ndarray],
    ) -> CollisionResult | None:
        """Check for collision between robot and environment obstacles.

        Args:
            link_transforms: Dictionary mapping link names to 4x4 transforms.

        Returns:
            CollisionResult if collision detected, None otherwise.
        """
        # Check each link against each obstacle
        for link_name, geometries in self.link_geometries.items():
            if link_name not in link_transforms:
                continue

            for geom in geometries:
                import copy

                g = copy.deepcopy(geom)
                g.pose = link_transforms[link_name] @ g.pose

                for obstacle in self.obstacles:
                    result = self._check_geometry_collision(g, obstacle)
                    if result.is_colliding:
                        result.pair = (link_name, "obstacle")
                        return result

        return None

    def _check_geometry_collision(
        self,
        g1,
        g2,
        collision_threshold: float = 0.0,
    ) -> CollisionResult:
        """Check collision between two geometries."""
        # Compute distance based on geometry types
        if isinstance(g1, Sphere) and isinstance(g2, Sphere):
            dist = distance_sphere_to_sphere(g1, g2)
        elif isinstance(g1, Sphere) and isinstance(g2, Capsule):
            dist = distance_sphere_to_capsule(g1, g2)
        elif isinstance(g1, Capsule) and isinstance(g2, Sphere):
            dist = distance_sphere_to_capsule(g2, g1)
        elif isinstance(g1, Capsule) and isinstance(g2, Capsule):
            dist = distance_capsule_to_capsule(g1, g2)
        elif isinstance(g1, Sphere) and isinstance(g2, Box):
            dist = distance_sphere_to_box(g1, g2)
        elif isinstance(g1, Box) and isinstance(g2, Sphere):
            dist = distance_sphere_to_box(g2, g1)
        elif isinstance(g1, Box) and isinstance(g2, Box):
            dist = distance_box_to_box(g1, g2)
        else:
            # Default: treat as spheres at origin
            dist = np.linalg.norm(g1.pose[:3, 3] - g2.pose[:3, 3])

        is_colliding = dist <= collision_threshold

        # Approximate contact point
        contact_point = None
        if is_colliding:
            contact_point = (g1.pose[:3, 3] + g2.pose[:3, 3]) / 2

        return CollisionResult(
            is_colliding=is_colliding,
            distance=dist,
            contact_point=contact_point,
            pair=("", ""),
        )
