"""Shared geometry for the band family (Elastic Bands, TEB).

Both members represent their state as a polyline (bubble centers / poses) that
gets deformed against the obstacle field each tick, so both need the same two
primitives: the closest occupied cell to a point (bubble clearance / obstacle
gradient) and arc-length resampling of a polyline (band initialization / TEB's
progress projection). Kept as free functions, mirroring the
`local_planning/_geometry.py` precedent for machinery shared across a family's
planners rather than owned by one.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from navigation.core.capabilities import ObstacleQuery
from navigation.core.types import Point

from .._geometry import sq_dist


def nearest_occupied(space: ObstacleQuery, p: Point, radius: float) -> tuple[Point | None, float]:
    """Closest occupied cell center to ``p`` within ``radius``, and the
    continuous (non-quantized) distance to it -- (None, inf) if none.

    Strict ``<`` keeps the first tie in ``occupied_within``'s row/col-ascending
    list, so a symmetric cluster of equidistant cells resolves the same way in
    every language rather than depending on iteration/comparison order.
    """
    best: Point | None = None
    best_sq = float("inf")
    for o in space.occupied_within(p, radius):
        d = sq_dist(p, o)
        if d < best_sq:
            best_sq = d
            best = o
    if best is None:
        return None, float("inf")
    return best, math.sqrt(best_sq)


def point_at_arclength(points: Sequence[Point], s: float) -> Point:
    """Point on polyline ``points`` at arc-length ``s`` from its start;
    clamped to the last point once ``s`` reaches or exceeds the total length."""
    if len(points) == 1 or s <= 0.0:
        return points[0]
    remaining = s
    for i in range(len(points) - 1):
        a, b = points[i], points[i + 1]
        seg_len = math.sqrt(sq_dist(a, b))
        if remaining <= seg_len:
            if seg_len < 1e-12:
                return a
            t = remaining / seg_len
            return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        remaining -= seg_len
    return points[-1]


def resample_polyline(points: Sequence[Point], spacing: float) -> list[Point]:
    """Re-samples ``points`` at even ``spacing`` arc-length intervals, exactly
    preserving the start and end points. A degenerate (near-zero-length) input
    collapses to just those two endpoints.

    Segment count follows the ``max(1, round(total / spacing))`` convention
    already used for distance-to-step-count conversion elsewhere in the
    codebase (`global_planning/sampling/eit_star.py`), so this stays on the
    same rounding contract Python/C++ parity already relies on.
    """
    if len(points) < 2:
        return list(points)
    total = 0.0
    for i in range(len(points) - 1):
        total += math.sqrt(sq_dist(points[i], points[i + 1]))
    if total < 1e-12:
        return [points[0], points[-1]]
    n_segments = max(1, round(total / spacing))
    step = total / n_segments
    out = [points[0]]
    for k in range(1, n_segments):
        out.append(point_at_arclength(points, k * step))
    out.append(points[-1])
    return out
