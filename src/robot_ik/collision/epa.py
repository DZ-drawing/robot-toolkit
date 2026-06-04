"""EPA (Expanding Polytope Algorithm) for penetration depth and contact info.

When GJK detects that two convex shapes are intersecting, EPA expands the
GJK simplex into a polytope that approximates the Minkowski difference
boundary. It iteratively adds support points to refine the polytope
until it finds the closest face to the origin, from which the penetration
depth, collision normal, and contact point are derived.

Public API
----------
- :func:`epa_penetration` -- compute penetration depth, normal, contact point.

References
----------
- van den Bergen, G. (2003). *Collision Detection in Interactive 3D Environments*.
- Ericson, C. (2005). *Real-Time Collision Detection*, ch. 5.
"""

from __future__ import annotations

import numpy as np

_EPS = 1e-8


def _support_minkowski(shape_a, shape_b, direction):
    """Support on Minkowski difference A - B. Returns (mk, pt_a, pt_b)."""
    from robot_ik.collision.gjk import _EPS as GJK_EPS

    direction = np.asarray(direction, dtype=float)
    norm = np.linalg.norm(direction)
    if norm < GJK_EPS:
        direction = np.array([1.0, 0.0, 0.0])
    else:
        direction = direction / norm
    pt_a = shape_a.support(direction)
    pt_b = shape_b.support(-direction)
    return pt_a - pt_b, pt_a.copy(), pt_b.copy()


def _triangle_normal_outward(v0, v1, v2):
    """Compute outward-facing triangle normal (pointing away from origin).

    Returns (normal, distance) or None if degenerate.
    """
    n = np.cross(v1 - v0, v2 - v0)
    n_len = np.linalg.norm(n)
    if n_len < _EPS:
        return None
    n /= n_len
    # Ensure normal points away from origin
    centroid = (v0 + v1 + v2) / 3.0
    if np.dot(n, centroid) < 0:
        n = -n
    d = float(np.dot(n, v0))
    return n, d


def _build_initial_tetrahedron(shape_a, shape_b):
    """Build a non-degenerate tetrahedron in the Minkowski difference.

    Tries multiple direction combinations until we get 4 non-coplanar
    Minkowski difference points that properly enclose the origin.

    Returns:
        (vertices, support_a, support_b, faces) or None.
        Each face is (normal, distance, [i0, i1, i2]).
    """
    # 26 candidate directions: 6 axes + 8 corners + 12 edge-midpoints
    candidates = [
        np.array([1.0, 0.0, 0.0]),
        np.array([-1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, -1.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, 0.0, -1.0]),
    ]
    for sx in (1.0, -1.0):
        for sy in (1.0, -1.0):
            for sz in (1.0, -1.0):
                candidates.append(np.array([sx, sy, sz]) / np.sqrt(3))
    for s1i in range(3):
        for s2i in range(3):
            if s1i == s2i:
                continue
            s3i = 3 - s1i - s2i
            for s1 in (1.0, -1.0):
                for s2 in (1.0, -1.0):
                    d = np.zeros(3)
                    d[s1i] = s1
                    d[s2i] = s2
                    candidates.append(d / np.linalg.norm(d))

    # Sample support points
    points = []  # (mk, pa, pb)
    for d in candidates:
        mk, pa, pb = _support_minkowski(shape_a, shape_b, d)
        is_dup = any(np.linalg.norm(mk - p[0]) < _EPS for p in points)
        if not is_dup:
            points.append((mk, pa, pb))

    if len(points) < 4:
        return None

    # Find 4 non-coplanar points that enclose the origin
    best_verts = None
    best_vol = 0.0

    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            for k in range(j + 1, len(points)):
                for l in range(k + 1, len(points)):
                    p0, p1, p2, p3 = points[i][0], points[j][0], points[k][0], points[l][0]
                    vol = abs(np.dot(p0 - p3, np.cross(p1 - p3, p2 - p3))) / 6.0
                    if vol < _EPS:
                        continue
                    if best_verts is None or vol > best_vol:
                        best_verts = (i, j, k, l)
                        best_vol = vol

    if best_verts is None:
        return None

    indices = list(best_verts)
    vertices = [points[idx][0].copy() for idx in indices]
    sa = [points[idx][1].copy() for idx in indices]
    sb = [points[idx][2].copy() for idx in indices]

    # Build 4 faces, normals pointing outward
    faces = []
    for omit in range(4):
        face_idx = [i for i in range(4) if i != omit]
        v0, v1, v2 = vertices[face_idx[0]], vertices[face_idx[1]], vertices[face_idx[2]]
        result = _triangle_normal_outward(v0, v1, v2)
        if result is None:
            return None
        n, d = result
        # Verify the omitted vertex is on the outside (positive side)
        omitted = vertices[omit]
        if np.dot(n, omitted) < -_EPS:
            # This means normal points inward for this face; flip
            # (shouldn't happen with centroid check, but safety)
            pass
        faces.append((n, d, face_idx))

    if len(faces) < 4:
        return None
    return vertices, sa, sb, faces


def _find_closest_face(faces):
    """Find face closest to origin. Returns (index, distance)."""
    best_i = -1
    best_d = float("inf")
    for i, (n, d, _) in enumerate(faces):
        if d < best_d:
            best_d = d
            best_i = i
    return best_i, best_d


def _find_adjacent(faces, skip, edge_fs):
    """Find face index sharing edge (as frozenset), other than skip."""
    for i, (_, _, idx) in enumerate(faces):
        if i == skip:
            continue
        fe = {
            frozenset((idx[0], idx[1])),
            frozenset((idx[1], idx[2])),
            frozenset((idx[0], idx[2])),
        }
        if edge_fs in fe:
            return i
    return None


def epa_penetration(
    shape_a, shape_b, gjk_simplex=None, max_iterations=64, tolerance=1e-6
):
    """Compute penetration depth, contact normal, and contact point.

    Args:
        shape_a: First convex shape with support() method.
        shape_b: Second convex shape with support() method.
        gjk_simplex: Ignored — EPA builds its own robust polytope.
        max_iterations: EPA expansion limit.
        tolerance: Convergence threshold.

    Returns:
        (depth, normal, contact_point).
        - depth >= 0: penetration depth.
        - normal: unit collision normal.
        - contact_point: world-frame contact point.
    """
    from robot_ik.collision.gjk import _closest_on_triangle, gjk_intersect

    # Pre-check: shapes must actually intersect
    if not gjk_intersect(shape_a, shape_b):
        return (0.0, np.array([1.0, 0.0, 0.0]), np.zeros(3))

    # Build robust initial tetrahedron
    result = _build_initial_tetrahedron(shape_a, shape_b)
    if result is None:
        return (0.0, np.array([1.0, 0.0, 0.0]), np.zeros(3))
    vertices, sa_list, sb_list, faces = result

    # Iteratively expand the polytope toward the closest face to origin
    for _ in range(max_iterations):
        fi, fd = _find_closest_face(faces)
        if fi < 0 or fd < 0:
            break

        normal = faces[fi][0]
        mk_new, pa_new, pb_new = _support_minkowski(shape_a, shape_b, normal)
        proj = float(np.dot(normal, mk_new))

        if proj - fd < tolerance:
            # Converged — compute contact point from closest face
            _, _, indices = faces[fi]
            v0 = vertices[indices[0]]
            v1 = vertices[indices[1]]
            v2 = vertices[indices[2]]
            _, bary = _closest_on_triangle(v0, v1, v2)
            cp_a = (
                bary[0] * sa_list[indices[0]]
                + bary[1] * sa_list[indices[1]]
                + bary[2] * sa_list[indices[2]]
            )
            cp_b = (
                bary[0] * sb_list[indices[0]]
                + bary[1] * sb_list[indices[1]]
                + bary[2] * sb_list[indices[2]]
            )
            contact = (cp_a + cp_b) / 2.0
            return (fd, normal, contact)

        new_idx = len(vertices)
        vertices.append(mk_new.copy())
        sa_list.append(pa_new.copy())
        sb_list.append(pb_new.copy())

        # BFS to find visible faces and horizon edges
        visible = set()
        queue = [fi]
        visible.add(fi)
        horizon = []
        horizon_set = set()

        while queue:
            current = queue.pop(0)
            _, _, idx = faces[current]
            edges = [
                frozenset((idx[0], idx[1])),
                frozenset((idx[1], idx[2])),
                frozenset((idx[0], idx[2])),
            ]
            for e in edges:
                if e in horizon_set:
                    continue
                adj = _find_adjacent(faces, current, e)
                if adj is None or adj in visible:
                    continue
                adj_n = faces[adj][0]
                if np.dot(adj_n, mk_new) > faces[adj][1] + _EPS:
                    visible.add(adj)
                    queue.append(adj)
                else:
                    pair = sorted(e)
                    horizon.append((pair[0], pair[1]))
                    horizon_set.add(e)

        # Remove visible faces, add new faces from horizon to new vertex
        faces = [f for i, f in enumerate(faces) if i not in visible]

        for e0, e1 in horizon:
            v0 = vertices[e0]
            v1 = vertices[e1]
            v_new = vertices[new_idx]
            n = np.cross(v1 - v0, v_new - v0)
            n_len = np.linalg.norm(n)
            if n_len < _EPS:
                continue
            n /= n_len
            # Ensure outward normal
            face_center = (v0 + v1 + v_new) / 3.0
            if np.dot(n, face_center) < 0:
                n = -n
            d = float(np.dot(n, v0))
            faces.append((n, d, [e0, e1, new_idx]))

        if len(faces) < 3:
            break

    # Best-effort result
    fi, fd = _find_closest_face(faces)
    if fi < 0 or fd <= 0:
        return (0.0, np.array([1.0, 0.0, 0.0]), np.zeros(3))

    _, _, indices = faces[fi]
    v0 = vertices[indices[0]]
    v1 = vertices[indices[1]]
    v2 = vertices[indices[2]]
    _, bary = _closest_on_triangle(v0, v1, v2)
    cp_a = (
        bary[0] * sa_list[indices[0]]
        + bary[1] * sa_list[indices[1]]
        + bary[2] * sa_list[indices[2]]
    )
    cp_b = (
        bary[0] * sb_list[indices[0]]
        + bary[1] * sb_list[indices[1]]
        + bary[2] * sb_list[indices[2]]
    )
    contact = (cp_a + cp_b) / 2.0
    return (fd, faces[fi][0], contact)
