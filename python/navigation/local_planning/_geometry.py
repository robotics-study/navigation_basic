"""Shared geometry for the local_planning category.

Category-internal `_`-prefixed module, mirroring the `global_planning/sampling/
_sampling.py` precedent for shared machinery that spans more than one family.
Holds the angle-wrap helper plus the polyline primitives (nearest point on a
segment, squared distance, monotonic progress-index advance) needed by both
the tracking family (lookahead-circle path following) and the band family
(arc-length progress projection) -- promoted out of `tracking/_path.py` so a
new family can reuse them without importing across families.
"""

from __future__ import annotations

import math

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
