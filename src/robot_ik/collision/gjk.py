"""GJK (Gilbert-Johnson-Keerthi) collision detection algorithm.

Provides intersection and distance queries between convex shapes represented
as :class:`TriangleMesh` objects.  The algorithm operates on the Minkowski
difference of two convex shapes: if the origin is enclosed by the Minkowski
difference, the shapes intersect.

Public API
----------
- :func:`gjk_intersect` — boolean intersection test.
- :func:`gjk_distance` — signed distance with closest-point witnesses.

References
----------
- Gilbert, E. G., Johnson, D. W., Keerthi, S. S. (1988).
  "A fast procedure for computing the distance between complex objects
  in three-dimensional space."
- Ericson, C. (2005). *Real-Time Collision Detection*, ch. 5.

Author: Danny Zeng
License: MIT
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from robot_ik.collision.mesh import TriangleMesh


# ======================================================================
# Numerical helpers
# ======================================================================

_EPS = 1e-12


def _triple_product(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> np.ndarray:
    """Compute the vector triple product (A x B) x C.

    Uses the identity (A x B) x C = B(A . C) - A(B . C).
    """
    return np.cross(np.cross(a, b), c)


def _safe_normalize(v: np.ndarray) -> np.ndarray:
    """Return *v* normalized, or a zero vector if |v| is too small."""
    n = np.linalg.norm(v)
    return v / n if n > _EPS else np.zeros(3)


# ======================================================================
# Geometry helpers
# ======================================================================


def _closest_on_line(
    origin: np.ndarray,
    direction: np.ndarray,
    point: np.ndarray,
) -> np.ndarray:
    """Closest point on an infinite line to a query point.

    Args:
        origin: Line origin.
        direction: Line direction (need not be unit length).
        point: Query point.

    Returns:
        Closest point on the line.
    """
    d = point - origin
    denom = np.dot(direction, direction)
    if denom < _EPS * _EPS:
        return origin.copy()
    t = np.dot(d, direction) / denom
    return origin + t * direction


def _closest_on_triangle(
    a: np.ndarray,
    b: np.ndarray,
    c: np.ndarray,
    origin: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Closest point on triangle ABC to the origin.

    Uses the algorithm from Ericson, *Real-Time Collision Detection*,
    Section 5.1.5 (Voronoi-region barycentric test).

    Args:
        a, b, c: Triangle vertices.
        origin: Query point (defaults to the origin).

    Returns:
        (closest_point, barycentric_weights) where barycentric weights
        sum to 1 and correspond to vertices a, b, c respectively.
    """
    if origin is None:
        origin = np.zeros(3)

    ab = b - a
    ac = c - a
    ao = origin - a

    d1 = np.dot(ab, ao)
    d2 = np.dot(ac, ao)
    if d1 <= 0.0 and d2 <= 0.0:
        # Vertex A region
        return a.copy(), np.array([1.0, 0.0, 0.0])

    bo = origin - b
    d3 = np.dot(ab, bo)
    d4 = np.dot(ac, bo)
    if d3 >= 0.0 and d4 <= d3:
        # Vertex B region
        return b.copy(), np.array([0.0, 1.0, 0.0])

    co = origin - c
    d5 = np.dot(ab, co)
    d6 = np.dot(ac, co)
    if d6 >= 0.0 and d5 <= d6:
        # Vertex C region
        return c.copy(), np.array([0.0, 0.0, 1.0])

    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        # Edge AB region
        v = d1 / (d1 - d3)
        pt = a + v * ab
        return pt, np.array([1.0 - v, v, 0.0])

    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        # Edge AC region
        w = d2 / (d2 - d6)
        pt = a + w * ac
        return pt, np.array([1.0 - w, 0.0, w])

    va = d3 * d6 - d5 * d4
    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        # Edge BC region
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        pt = b + w * (c - b)
        return pt, np.array([0.0, 1.0 - w, w])

    # Inside triangle face
    denom = 1.0 / (va + vb + vc)
    v = vb * denom
    w = vc * denom
    pt = a + v * ab + w * ac
    return pt, np.array([1.0 - v - w, v, w])


# ======================================================================
# Minkowski difference support
# ======================================================================


def _support_minkowski(
    shape_a: TriangleMesh,
    shape_b: TriangleMesh,
    direction: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Support function for the Minkowski difference A ⊖ B.

    Returns ``(a − b, a, b)`` where *a* and *b* are the support points of
    shape A and shape B respectively.
    """
    direction = np.asarray(direction, dtype=float)
    d_norm = np.linalg.norm(direction)
    if d_norm < _EPS:
        direction = np.array([1.0, 0.0, 0.0])
    pt_a = shape_a.support(direction)
    pt_b = shape_b.support(-direction)
    return pt_a - pt_b, pt_a, pt_b


# ======================================================================
# Simplex evolution (Voronoi-region checks)
# ======================================================================
#
# The simplex is stored as a list of tuples ``(mk_point, pt_a, pt_b)`` where
# *mk_point* is the Minkowski-difference vertex and *pt_a*, *pt_b* are the
# individual shape support points used to compute it.
#
# Each evolution function receives the simplex, extracts the Minkowski
# points for geometric computation, and may shrink the list in-place when
# a sub-feature (edge or vertex) is closest to the origin.
#
# Returns ``(contains_origin: bool, direction: ndarray)``.


def _evolve_line(simplex: list) -> tuple[bool, np.ndarray]:
    """Evolve a 2-point simplex (line segment) toward the origin.

    Simplex convention: ``[B, A]`` where A is the newest point.
    """
    B_mk = simplex[0][0]
    A_mk = simplex[1][0]
    AB = B_mk - A_mk
    AO = -A_mk

    if np.dot(AB, AO) > 0.0:
        # Origin projects onto the edge AB (not past A).
        # Direction: perpendicular to AB toward origin.
        # Triple product: (AB x AO) x AB = |AB|² * AO_perp
        direction = _triple_product(AB, AO, AB)
    else:
        # Origin is closest to vertex A.
        simplex[:] = [simplex[1]]
        direction = AO

    # Normalize for numerical stability (direction is used for next support)
    return False, _safe_normalize(direction)


def _evolve_triangle(simplex: list) -> tuple[bool, np.ndarray]:
    """Evolve a 3-point simplex (triangle) toward the origin.

    Simplex convention: ``[C, B, A]`` where A is the newest point.

    Checks Voronoi regions of edges AB, AC and the face interior.
    """
    A_mk = simplex[2][0]
    B_mk = simplex[1][0]
    C_mk = simplex[0][0]

    AB = B_mk - A_mk
    AC = C_mk - A_mk
    AO = -A_mk

    # Triangle normal (right-hand rule AB → AC)
    n = np.cross(AB, AC)

    # ---- Check which side of the triangle the origin is on ----
    dot_n_ao = np.dot(n, AO)

    # Perpendicular to AB in the triangle plane, pointing *away* from C.
    # cross(AB, n) = cross(AB, cross(AB, AC))
    perp_ab = np.cross(AB, n)

    # Perpendicular to AC in the triangle plane, pointing *away* from B.
    # cross(n, AC) = cross(cross(AB, AC), AC)
    perp_ac = np.cross(n, AC)

    # ---- Voronoi-region checks ----

    if dot_n_ao > 0.0:
        # Origin is on the normal side of the triangle.
        # Check edge AB (outside = away from C).
        if np.dot(perp_ab, AO) > 0.0:
            if np.dot(AB, AO) > 0.0:
                # Edge AB region.
                simplex[:] = [simplex[1], simplex[2]]  # [B, A]
                direction = _triple_product(AB, AO, AB)
            else:
                # Vertex A region.
                simplex[:] = [simplex[2]]
                direction = AO
            return False, _safe_normalize(direction)

        # Check edge AC (outside = away from B).
        if np.dot(perp_ac, AO) > 0.0:
            if np.dot(AC, AO) > 0.0:
                # Edge AC region.
                simplex[:] = [simplex[0], simplex[2]]  # [C, A]
                direction = _triple_product(AC, AO, AC)
            else:
                # Vertex A region.
                simplex[:] = [simplex[2]]
                direction = AO
            return False, _safe_normalize(direction)

        # Face region — origin projects inside the triangle.
        direction = n
    else:
        # Origin is on the *opposite* side of the triangle.
        # We still check the same edge conditions but with the
        # understanding that the perp vectors flip relative to AO.
        if np.dot(perp_ab, AO) > 0.0:
            if np.dot(AB, AO) > 0.0:
                simplex[:] = [simplex[1], simplex[2]]
                direction = _triple_product(AB, AO, AB)
            else:
                simplex[:] = [simplex[2]]
                direction = AO
            return False, _safe_normalize(direction)

        if np.dot(perp_ac, AO) > 0.0:
            if np.dot(AC, AO) > 0.0:
                simplex[:] = [simplex[0], simplex[2]]
                direction = _triple_product(AC, AO, AC)
            else:
                simplex[:] = [simplex[2]]
                direction = AO
            return False, _safe_normalize(direction)

        # Face region on opposite side.
        direction = -n

    return False, _safe_normalize(direction)


def _evolve_tetrahedron(simplex: list) -> tuple[bool, np.ndarray]:
    """Evolve a 4-point simplex (tetrahedron) toward the origin.

    Simplex convention: ``[D, C, B, A]`` where A is the newest point.

    Tests the three faces that contain vertex A.  If the origin is
    outside any face, the simplex is reduced to that face and
    :func:`_evolve_triangle` is invoked.  If the origin is inside all
    three faces, the shapes intersect.
    """
    A_mk = simplex[3][0]
    B_mk = simplex[2][0]
    C_mk = simplex[1][0]
    D_mk = simplex[0][0]

    AB = B_mk - A_mk
    AC = C_mk - A_mk
    AD = D_mk - A_mk
    AO = -A_mk

    # --- Face ABC (opposite vertex: D) ---
    ABC = np.cross(AB, AC)
    # Origin is outside ABC iff dot(n, AO) and dot(n, AD) have
    # opposite signs (origin on the opposite side from D).
    if np.dot(ABC, AO) * np.dot(ABC, AD) < 0.0:
        simplex[:] = [simplex[1], simplex[2], simplex[3]]  # [C, B, A]
        return _evolve_triangle(simplex)

    # --- Face ABD (opposite vertex: C) ---
    ABD = np.cross(AB, AD)
    if np.dot(ABD, AO) * np.dot(ABD, AC) < 0.0:
        simplex[:] = [simplex[0], simplex[2], simplex[3]]  # [D, B, A]
        return _evolve_triangle(simplex)

    # --- Face ACD (opposite vertex: B) ---
    ACD = np.cross(AC, AD)
    if np.dot(ACD, AO) * np.dot(ACD, AB) < 0.0:
        simplex[:] = [simplex[0], simplex[1], simplex[3]]  # [D, C, A]
        return _evolve_triangle(simplex)

    # Origin is inside the tetrahedron.
    return True, np.zeros(3)


def _evolve_simplex(simplex: list) -> tuple[bool, np.ndarray]:
    """Dispatch to the appropriate simplex evolution handler."""
    n = len(simplex)
    if n == 2:
        return _evolve_line(simplex)
    elif n == 3:
        return _evolve_triangle(simplex)
    elif n == 4:
        return _evolve_tetrahedron(simplex)
    # Should never reach here.
    raise RuntimeError(f"Unexpected simplex size {n}")


# ======================================================================
# Simplex-to-origin distance helpers
# ======================================================================


def _segment_distance(
    simplex: list,
) -> tuple[np.ndarray, np.ndarray]:
    """Closest point on a 2-point simplex (segment) to the origin.

    Returns ``(closest_point, barycentric_weights)``.
    """
    A_mk = simplex[0][0]
    B_mk = simplex[1][0]
    AB = B_mk - A_mk
    AO = -A_mk

    ab_sq = np.dot(AB, AB)
    if ab_sq < _EPS * _EPS:
        return A_mk.copy(), np.array([1.0, 0.0])

    t = np.dot(AO, AB) / ab_sq
    t = float(np.clip(t, 0.0, 1.0))

    closest = A_mk + t * AB
    return closest, np.array([1.0 - t, t])


def _triangle_distance(
    simplex: list,
) -> tuple[np.ndarray, np.ndarray]:
    """Closest point on a 3-point simplex (triangle) to the origin.

    Returns ``(closest_point, barycentric_weights)``.
    """
    A_mk = simplex[0][0]
    B_mk = simplex[1][0]
    C_mk = simplex[2][0]
    return _closest_on_triangle(A_mk, B_mk, C_mk)


def _tetrahedron_distance(
    simplex: list,
) -> tuple[np.ndarray, np.ndarray]:
    """Closest point on a 4-point simplex (tetrahedron) to the origin.

    Brute-force check of all 4 faces — each face's closest point is
    computed via :func:`_triangle_distance`, and the overall closest is
    returned along with full 4-element barycentric weights.
    """
    best_dist_sq = float("inf")
    best_pt: np.ndarray | None = None
    best_bary: np.ndarray | None = None

    for i0, i1, i2 in [(0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3)]:
        face = [simplex[i0], simplex[i1], simplex[i2]]
        pt, face_bary = _triangle_distance(face)
        d_sq = float(np.dot(pt, pt))
        if d_sq < best_dist_sq:
            best_dist_sq = d_sq
            best_pt = pt
            full = np.zeros(4)
            full[i0] = face_bary[0]
            full[i1] = face_bary[1]
            full[i2] = face_bary[2]
            best_bary = full

    return best_pt, best_bary  # type: ignore[return-value]


def _simplex_distance(
    simplex: list,
) -> tuple[np.ndarray, np.ndarray | None]:
    """Closest point on the current simplex to the origin.

    Returns ``(closest_point, barycentric_weights)``.
    Weights are ``None`` when the origin is inside the simplex.
    """
    n = len(simplex)
    if n == 1:
        return simplex[0][0].copy(), np.array([1.0])
    elif n == 2:
        return _segment_distance(simplex)
    elif n == 3:
        return _triangle_distance(simplex)
    elif n == 4:
        # For 4-point simplex, check all faces.
        # If best distance ≈ 0 the origin is inside.
        pt, bary = _tetrahedron_distance(simplex)
        if float(np.dot(pt, pt)) < _EPS * _EPS:
            return np.zeros(3), None
        return pt, bary
    else:
        # Should not happen.
        return np.zeros(3), None


# ======================================================================
# Public API
# ======================================================================


def gjk_intersect(
    shape_a: TriangleMesh,
    shape_b: TriangleMesh,
    max_iterations: int = 64,
) -> bool:
    """Test whether two convex shapes intersect.

    Uses the GJK algorithm on the Minkowski difference A ⊖ B.  Returns
    ``True`` if the origin lies inside (or on the boundary of) the
    Minkowski difference.

    Args:
        shape_a: First convex shape (must expose a ``support`` method).
        shape_b: Second convex shape.
        max_iterations: Safety cap on the number of iterations.

    Returns:
        ``True`` if the shapes intersect.
    """
    # Pick an arbitrary initial direction.
    direction = np.array([1.0, 0.0, 0.0])

    mk, _, _ = _support_minkowski(shape_a, shape_b, direction)
    # If the first Minkowski support point is very close to origin
    # the shapes are almost certainly overlapping.
    if np.linalg.norm(mk) < _EPS:
        return True

    simplex: list = [(mk, None, None)]
    direction = -mk
    direction = _safe_normalize(direction)
    if np.linalg.norm(direction) < _EPS:
        return True

    for _ in range(max_iterations):
        mk, _, _ = _support_minkowski(shape_a, shape_b, direction)

        # If the new support point did not pass the origin, the shapes
        # are separated.
        if np.dot(mk, direction) < 0.0:
            return False

        simplex.append((mk, None, None))

        contains, direction = _evolve_simplex(simplex)
        if contains:
            return True

    # Failed to converge — treat as non-intersecting.
    return False


def _reduce_simplex(simplex: list) -> None:
    """Remove simplex vertices whose barycentric weight is ≈ 0.

    After finding the closest point on the current simplex, any vertex
    that does not contribute (weight ≈ 0) can be dropped without
    changing the closest point.  This keeps the simplex bounded at 3
    points for the distance loop.
    """
    n = len(simplex)
    if n <= 2:
        return
    _, bary = _simplex_distance(simplex)
    if bary is None:
        return  # origin inside — let the caller handle it
    threshold = 1e-10
    simplex[:] = [simplex[i] for i in range(n) if bary[i] > threshold]


def gjk_distance(
    shape_a: TriangleMesh,
    shape_b: TriangleMesh,
    max_iterations: int = 64,
) -> tuple[float, np.ndarray | None, np.ndarray | None]:
    """Compute the distance between two convex shapes.

    Uses the Gilbert-Johnson-Keerthi iterative refinement algorithm.
    When the shapes are separated the distance is the Euclidean distance
    between the closest pair of surface points.  When the shapes overlap,
    returns ``(0.0, None, None)``.

    The algorithm maintains a small simplex in the Minkowski difference
    A ⊖ B and iteratively refines it toward the closest feature to the
    origin.  Convergence is detected when a new support point no longer
    improves the estimate.

    Args:
        shape_a: First convex shape.
        shape_b: Second convex shape.
        max_iterations: Safety cap on iterations.

    Returns:
        ``(distance, closest_on_a, closest_on_b)`` where the two
        closest-point arrays are 3-element vectors (world frame) or
        ``None`` when the shapes overlap.
    """
    # --- Gilbert's distance algorithm (1988) --------------------------

    # Step 1: initial support point in an arbitrary direction.
    direction = np.array([1.0, 0.0, 0.0])
    mk, pa, pb = _support_minkowski(shape_a, shape_b, direction)
    if np.linalg.norm(mk) < _EPS:
        return 0.0, None, None

    simplex: list = [(mk, pa, pb)]

    for _ in range(max_iterations):
        # Step 2: find closest point *y* on current simplex to origin.
        _reduce_simplex(simplex)
        closest, bary = _simplex_distance(simplex)
        dist_sq = float(np.dot(closest, closest))

        if dist_sq < _EPS * _EPS:
            return 0.0, None, None  # overlapping

        # Step 3: search direction = −y / |y| (toward origin).
        direction = -closest / np.sqrt(dist_sq)

        # Step 4: new support point.
        mk_new, pa_new, pb_new = _support_minkowski(shape_a, shape_b, direction)

        # Step 5: convergence check.
        # |w − y|² < ε² (1 + |y|²)  ⟹  distance has converged.
        diff = mk_new - closest
        if float(np.dot(diff, diff)) < _EPS * _EPS * (1.0 + dist_sq):
            distance = np.sqrt(dist_sq)
            # Compute witness points from barycentric weights.
            closest_a = np.zeros(3)
            closest_b = np.zeros(3)
            for i, w in enumerate(bary):
                closest_a += w * simplex[i][1]
                closest_b += w * simplex[i][2]
            return float(distance), closest_a, closest_b

        # Step 6: add new point to the simplex.
        simplex.append((mk_new, pa_new, pb_new))

    # Did not converge — return best-effort distance.
    _reduce_simplex(simplex)
    closest, bary = _simplex_distance(simplex)
    dist = float(np.linalg.norm(closest))
    if bary is not None and dist > _EPS:
        closest_a = np.zeros(3)
        closest_b = np.zeros(3)
        for i, w in enumerate(bary):
            closest_a += w * simplex[i][1]
            closest_b += w * simplex[i][2]
        return dist, closest_a, closest_b
    return dist, None, None
