import numpy as np

__all__ = [
    "segment_intersects_aabb",
    "segment_intersects_any_aabb",
    "path_is_collision_free",
    "point_in_aabb",
]


def _prepare_aabb(bounds):
    """Convert *bounds* to (min_corner, max_corner) np.ndarray tuples.

    Parameters
    ----------
    bounds : Sequence[float] | np.ndarray
        Either a 6-tuple ``(xmin, ymin, zmin, xmax, ymax, zmax)`` or a
        2x3 array ``[[xmin, ymin, zmin], [xmax, ymax, zmax]]``.
    """
    b = np.asarray(bounds, dtype=float)
    if b.shape == (6,):
        return b[:3], b[3:]
    if b.shape == (2, 3):
        return b[0], b[1]
    raise ValueError("AABB bounds must be shape (6,) or (2,3)")


def point_in_aabb(p, bounds, *, inclusive=True):
    """Return ``True`` iff *p* lies inside *bounds*.

    Parameters
    ----------
    p : Sequence[float] | np.ndarray
        3-D point.
    bounds : Sequence[float] | np.ndarray
        AABB bounds.
    inclusive : bool, optional
        If ``True`` (default) points on the boundary count as inside.
    """
    p = np.asarray(p, dtype=float)
    min_c, max_c = _prepare_aabb(bounds)
    if inclusive:
        return np.all(p >= min_c) and np.all(p <= max_c)
    return np.all(p > min_c) and np.all(p < max_c)


def segment_intersects_aabb(p0, p1, bounds, *, epsilon=1e-9):
    """Check whether the closed line segment *p0-p1* intersects a 3-D AABB.

    The test uses the *slab* (parametric) method and runs in O(1) time.

    Parameters
    ----------
    p0, p1 : Sequence[float] | np.ndarray
        End points of the segment.
    bounds : Sequence[float] | np.ndarray
        AABB bounds.
    epsilon : float, optional
        Threshold for detecting zero directional components.

    Returns
    -------
    bool
        ``True`` if the segment intersects (or touches) the box, ``False`` otherwise.
    """
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)

    # Early‑out: endpoint inside the box.
    if point_in_aabb(p0, bounds) or point_in_aabb(p1, bounds):
        return True

    d = p1 - p0  # direction vector
    min_c, max_c = _prepare_aabb(bounds)

    t_min = 0.0  # entering parametric value
    t_max = 1.0  # exiting parametric value

    for i in range(3):
        if abs(d[i]) < epsilon:
            # Segment is parallel to the slab. Reject if outside.
            if p0[i] < min_c[i] or p0[i] > max_c[i]:
                return False
            continue  # otherwise the segment lies within the slab; no constraint.

        inv_d = 1.0 / d[i]
        t1 = (min_c[i] - p0[i]) * inv_d
        t2 = (max_c[i] - p0[i]) * inv_d

        t_low, t_high = (t1, t2) if t1 <= t2 else (t2, t1)
        t_min = max(t_min, t_low)
        t_max = min(t_max, t_high)
        if t_min > t_max:
            return False  # No intersection across all slabs.

    return True  # Overlaps for all axes.


def segment_intersects_any_aabb(p0, p1, boxes):
    """Return ``True`` if *p0-p1* intersects *any* box in *boxes*.

    Parameters
    ----------
    p0, p1 : Sequence[float] | np.ndarray
        Segment endpoints.
    boxes : Iterable[Sequence[float] | np.ndarray]
        An iterable of AABB bounds.
    """
    for b in boxes:
        if segment_intersects_aabb(p0, p1, b):
            return True
    return False


def path_is_collision_free(waypoints, boxes):
    """Check whether a *piecewise-linear* path is collision-free.

    Parameters
    ----------
    waypoints : Sequence[Sequence[float]]
        Ordered 3-D points describing the path.
    boxes : Iterable[Sequence[float] | np.ndarray]
        Obstacles (AABBs).

    Returns
    -------
    bool
        ``True`` if *every* segment of the path avoids all boxes.
    """
    it = iter(waypoints)
    try:
        p_prev = next(it)
    except StopIteration:
        return True  # Empty path.
    for p_curr in it:
        if segment_intersects_any_aabb(p_prev, p_curr, boxes):
            return False
        p_prev = p_curr
    return True


if __name__ == "__main__":
    # Simple self‑test.
    cube = (0, 0, 0, 1, 1, 1)
    assert segment_intersects_aabb([0.5, 0.5, -1], [0.5, 0.5, 2], cube)
    assert not segment_intersects_aabb([2, 2, 2], [3, 3, 3], cube)
    assert path_is_collision_free([[1.5, 0, 0], [1.5, 1, 1]], [cube])
    assert not path_is_collision_free([[0.5, -1, 0.5], [0.5, 2, 0.5]], [cube])
    print("All tests passed.")
