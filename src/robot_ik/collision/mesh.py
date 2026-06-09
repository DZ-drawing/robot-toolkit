"""TriangleMesh for GJK collision detection.

Provides a TriangleMesh dataclass representing convex triangle meshes with
a support function suitable for use in the GJK (Gilbert-Johnson-Keerthi)
collision detection algorithm.

Factory methods construct common primitive shapes as convex meshes:
- from_convex_hull: generic convex hull from point cloud
- from_box: axis-aligned box (12 triangles)
- from_sphere: icosphere approximation
- from_capsule: capsule approximation via convex hull

Author: Danny Zeng
License: MIT
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class TriangleMesh:
    """Convex triangle mesh with support function for GJK.

    Attributes:
        vertices: Nx3 array of vertex positions in local frame.
        faces: Mx3 array of triangle face indices (into vertices).
        pose: 4x4 homogeneous transform (local -> world).
        name: Optional identifier for the mesh.
    """

    vertices: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    faces: np.ndarray = field(default_factory=lambda: np.zeros((0, 3), dtype=int))
    pose: np.ndarray = field(default_factory=lambda: np.eye(4))
    name: str = "unnamed"

    # ------------------------------------------------------------------
    # Core methods
    # ------------------------------------------------------------------

    def support(self, direction: np.ndarray) -> np.ndarray:
        """Return the farthest vertex in *direction* (world frame).

        Applies the mesh rotation so that the search is performed correctly
        when the mesh is oriented.

        Args:
            direction: 3-element direction vector (world frame).

        Returns:
            3-element vertex position in world frame.
        """
        direction = np.asarray(direction, dtype=float)
        # Transform direction into local frame for vertex search
        R = self.pose[:3, :3]
        local_dir = R.T @ direction

        # Find vertex that maximises dot product with local direction
        dots = self.vertices @ local_dir
        idx = np.argmax(dots)
        local_vertex = self.vertices[idx]

        # Transform to world frame
        return R @ local_vertex + self.pose[:3, 3]

    def update_pose(self, pose: np.ndarray) -> None:
        """Replace the current pose transform.

        Args:
            pose: 4x4 homogeneous transform matrix.
        """
        self.pose = np.asarray(pose, dtype=float)

    # ------------------------------------------------------------------
    # Factory methods
    # ------------------------------------------------------------------

    @classmethod
    def from_convex_hull(
        cls,
        points: np.ndarray,
        pose: np.ndarray | None = None,
        name: str = "convex_hull",
    ) -> TriangleMesh:
        """Create a mesh from the convex hull of a point cloud.

        Uses :class:`scipy.spatial.ConvexHull` to compute the hull.

        Args:
            points: Nx3 array of input points.
            pose: Optional 4x4 transform (defaults to identity).
            name: Identifier string.

        Returns:
            TriangleMesh representing the convex hull.
        """
        from scipy.spatial import ConvexHull  # noqa: WPS433

        points = np.asarray(points, dtype=float)
        hull = ConvexHull(points)
        vertices = hull.points[hull.vertices].copy()
        faces = hull.simplices.copy()

        if pose is None:
            pose = np.eye(4)
        return cls(vertices=vertices, faces=faces, pose=pose, name=name)

    @classmethod
    def from_box(
        cls,
        size: np.ndarray | list[float],
        pose: np.ndarray | None = None,
        name: str = "box",
    ) -> TriangleMesh:
        """Create a 12-triangle box mesh.

        Args:
            size: [sx, sy, sz] full extents of the box.
            pose: Optional 4x4 transform (defaults to identity).
            name: Identifier string.

        Returns:
            TriangleMesh with 8 vertices and 12 triangular faces.
        """
        size = np.asarray(size, dtype=float)
        hx, hy, hz = size / 2.0

        # 8 corners of the box
        vertices = np.array(
            [
                [-hx, -hy, -hz],
                [+hx, -hy, -hz],
                [+hx, +hy, -hz],
                [-hx, +hy, -hz],
                [-hx, -hy, +hz],
                [+hx, -hy, +hz],
                [+hx, +hy, +hz],
                [-hx, +hy, +hz],
            ],
            dtype=float,
        )

        # 12 triangles (2 per face, 6 faces)
        faces = np.array(
            [
                # -Z face
                [0, 2, 1],
                [0, 3, 2],
                # +Z face
                [4, 5, 6],
                [4, 6, 7],
                # -X face
                [0, 4, 7],
                [0, 7, 3],
                # +X face
                [1, 2, 6],
                [1, 6, 5],
                # -Y face
                [0, 1, 5],
                [0, 5, 4],
                # +Y face
                [3, 7, 6],
                [3, 6, 2],
            ],
            dtype=int,
        )

        if pose is None:
            pose = np.eye(4)
        return cls(vertices=vertices, faces=faces, pose=pose, name=name)

    @classmethod
    def from_sphere(
        cls,
        radius: float = 1.0,
        subdivisions: int = 1,
        pose: np.ndarray | None = None,
        name: str = "sphere",
    ) -> TriangleMesh:
        """Create an icosphere approximation.

        Starts with a regular icosahedron and subdivides each triangle
        *subdivisions* times, projecting new vertices onto the sphere.

        Args:
            radius: Sphere radius.
            subdivisions: Number of subdivision iterations (0 = 20 faces).
            pose: Optional 4x4 transform (defaults to identity).
            name: Identifier string.

        Returns:
            TriangleMesh approximating a sphere.
        """
        # Golden ratio
        phi = (1.0 + np.sqrt(5.0)) / 2.0

        # 12 vertices of a regular icosahedron
        vertices = np.array(
            [
                [-1, phi, 0],
                [1, phi, 0],
                [-1, -phi, 0],
                [1, -phi, 0],
                [0, -1, phi],
                [0, 1, phi],
                [0, -1, -phi],
                [0, 1, -phi],
                [phi, 0, -1],
                [phi, 0, 1],
                [-phi, 0, -1],
                [-phi, 0, 1],
            ],
            dtype=float,
        )

        # Normalise to unit sphere then scale by radius
        norms = np.linalg.norm(vertices, axis=1, keepdims=True)
        vertices = vertices / norms * radius

        # 20 triangular faces of the icosahedron
        faces = np.array(
            [
                [0, 11, 5],
                [0, 5, 1],
                [0, 1, 7],
                [0, 7, 10],
                [0, 10, 11],
                [1, 5, 9],
                [5, 11, 4],
                [11, 10, 2],
                [10, 7, 6],
                [7, 1, 8],
                [3, 9, 4],
                [3, 4, 2],
                [3, 2, 6],
                [3, 6, 8],
                [3, 8, 9],
                [4, 9, 5],
                [2, 4, 11],
                [6, 2, 10],
                [8, 6, 7],
                [9, 8, 1],
            ],
            dtype=int,
        )

        for _ in range(subdivisions):
            vertices, faces = cls._subdivide(vertices, faces, radius)

        if pose is None:
            pose = np.eye(4)
        return cls(vertices=vertices, faces=faces, pose=pose, name=name)

    @classmethod
    def from_capsule(
        cls,
        p1: np.ndarray,
        p2: np.ndarray,
        radius: float = 0.05,
        subdivisions: int = 1,
        pose: np.ndarray | None = None,
        name: str = "capsule",
    ) -> TriangleMesh:
        """Create a capsule mesh via convex hull of generated points.

        Generates hemispheres at each endpoint and rings along the body,
        then takes the convex hull.

        Args:
            p1: Start point of capsule axis (3-element).
            p2: End point of capsule axis (3-element).
            radius: Capsule radius.
            subdivisions: Refinement level (more rings + subdivisions).
            pose: Optional 4x4 transform (defaults to identity).
            name: Identifier string.

        Returns:
            TriangleMesh approximating a capsule.
        """
        p1 = np.asarray(p1, dtype=float)
        p2 = np.asarray(p2, dtype=float)
        axis = p2 - p1
        length = np.linalg.norm(axis)

        if length < 1e-10:
            # Degenerate capsule → sphere
            return cls.from_sphere(radius=radius, subdivisions=subdivisions, pose=pose, name=name)

        axis_dir = axis / length

        # Build an orthonormal basis perpendicular to the axis
        if abs(axis_dir[0]) < 0.9:
            perp1 = np.cross(axis_dir, np.array([1.0, 0.0, 0.0]))
        else:
            perp1 = np.cross(axis_dir, np.array([0.0, 1.0, 0.0]))
        perp1 /= np.linalg.norm(perp1)
        perp2 = np.cross(axis_dir, perp1)

        n_rings = max(4, 4 * subdivisions)
        n_radial = max(6, 6 * subdivisions)
        points: list[np.ndarray] = []

        for i in range(n_rings + 1):
            t = i / n_rings  # 0..1 along axis
            center = p1 + t * axis

            # At the ends, points are on the hemisphere so the radius along
            # the axis shrinks: r_axis = sqrt(max(0, radius^2 - (t*L)^2))
            # For t in [0,1], distance from nearest end:
            d_end = min(t, 1.0 - t) * length
            r_lat = np.sqrt(max(0.0, radius**2 - d_end**2))

            if r_lat < 1e-12:
                # Pole point
                points.append(center.copy())
            else:
                for j in range(n_radial):
                    angle = 2.0 * np.pi * j / n_radial
                    pt = center + r_lat * (np.cos(angle) * perp1 + np.sin(angle) * perp2)
                    points.append(pt)

        pts_array = np.array(points, dtype=float)
        return cls.from_convex_hull(pts_array, pose=pose, name=name)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _subdivide(
        vertices: np.ndarray, faces: np.ndarray, radius: float
    ) -> tuple[np.ndarray, np.ndarray]:
        """Subdivide each triangle into 4 children and project onto sphere.

        Args:
            vertices: Current Nx3 vertex array.
            faces: Current Mx3 face array.
            radius: Target sphere radius for new vertices.

        Returns:
            Tuple of (new_vertices, new_faces).
        """
        # Build a lookup for edge midpoints so shared edges produce one vertex
        edge_midpoint: dict[tuple[int, int], int] = {}

        new_vertices = list(vertices)

        def get_midpoint(i: int, j: int) -> int:
            key = (min(i, j), max(i, j))
            if key in edge_midpoint:
                return edge_midpoint[key]
            mid = (new_vertices[i] + new_vertices[j]) / 2.0
            mid = mid / np.linalg.norm(mid) * radius
            idx = len(new_vertices)
            new_vertices.append(mid)
            edge_midpoint[key] = idx
            return idx

        new_faces: list[list[int]] = []
        for f in faces:
            a, b, c = int(f[0]), int(f[1]), int(f[2])
            ab = get_midpoint(a, b)
            bc = get_midpoint(b, c)
            ca = get_midpoint(c, a)
            new_faces.append([a, ab, ca])
            new_faces.append([b, bc, ab])
            new_faces.append([c, ca, bc])
            new_faces.append([ab, bc, ca])

        return np.array(new_vertices, dtype=float), np.array(new_faces, dtype=int)
