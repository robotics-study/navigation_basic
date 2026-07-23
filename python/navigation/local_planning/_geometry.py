"""Shared geometry for the local_planning category.

Category-internal `_`-prefixed module, mirroring the `global_planning/sampling/
_sampling.py` precedent for shared machinery that spans more than one family.
Holds the angle-wrap helper, the polyline primitives (nearest point on a
segment, squared distance, monotonic progress-index advance) needed by both
the tracking family (lookahead-circle path following) and the band family
(arc-length progress projection), and the nearest-occupied obstacle lookup
shared by the band family (bubble/pose clearance) and the predictive family
(the MPC/MPPI obstacle cost). Each was promoted out of a single family's
private module once a second family needed it, so a new family can reuse it
without importing across families.
"""

from __future__ import annotations

import math

from navigation.core.capabilities import ObstacleQuery
from navigation.core.types import Point


def wrap_to_pi(angle: float) -> float:
    """Normalize ``angle`` (radians) to (-pi, pi]."""
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def closest_point_on_segment(p: Point, a: Point, b: Point) -> Point:
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-12:
        return a
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / seg_len_sq))
    return (ax + t * dx, ay + t * dy)


def sq_dist(a: Point, b: Point) -> float:
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


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


def advance_progress_index(path: tuple[Point, ...], probe: Point, start_index: int) -> int:
    """Nearest-segment index at or after ``start_index`` -- monotonic forward-only
    so a self-crossing path never snaps tracking backward to an earlier,
    geometrically-closer crossing."""
    if len(path) < 2:
        return start_index
    best_index = start_index
    best_sq_dist = float("inf")
    for i in range(start_index, len(path) - 1):
        closest = closest_point_on_segment(probe, path[i], path[i + 1])
        d = sq_dist(probe, closest)
        # <=, not <: consecutive segments share their joint endpoint, so a
        # probe sitting exactly at a corner ties every segment ending/starting
        # there. Preferring the later (more forward) segment on a tie keeps
        # progress advancing through the corner instead of latching onto the
        # segment just traveled.
        if d <= best_sq_dist:
            best_sq_dist = d
            best_index = i
    return best_index
