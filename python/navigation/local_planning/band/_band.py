"""Arc-length polyline geometry for the band family (Elastic Bands, TEB).

Both members represent their state as a polyline (bubble centers / poses) that
gets deformed against the obstacle field each tick, so both need arc-length
sampling/resampling of a polyline (band initialization / TEB's progress
projection). Kept as free functions, mirroring the `local_planning/_geometry.py`
precedent for machinery shared across a family's planners rather than owned by
one. (Nearest-occupied lookup, once band-only, now lives in the category-shared
`_geometry.py` since the predictive family reuses it too.)
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from navigation.core.types import Point

from .._geometry import sq_dist


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
