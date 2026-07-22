"""Reference-path lookahead geometry for tracking planners.

Path trackers (Pure Pursuit and its descendants) additionally need to
intersect a lookahead circle with the polyline -- kept here as a tracking-only
free function. The point-projection/progress-index primitives this family
shares with the band family live in `local_planning/_geometry.py` and are
re-exported below so existing `from ._path import ...` call sites keep
working unchanged. Mirrors the C++ `local_planning/tracking/path.hpp`.
"""

from __future__ import annotations

import math

from navigation.core.types import Point
from navigation.local_planning._geometry import (
    advance_progress_index,
    closest_point_on_segment,
    sq_dist,
)

__all__ = [
    "advance_progress_index",
    "closest_point_on_segment",
    "lookahead_point",
    "segment_circle_forward_t",
    "sq_dist",
]


def segment_circle_forward_t(p: Point, a: Point, b: Point, radius: float) -> float | None:
    """Forward-most intersection of the probe-centered lookahead circle with
    segment a->b, as a parameter t in [0, 1], or None if the segment stays
    entirely inside/outside the circle.

    Solves |a + t*(b-a) - p|^2 = radius^2 (Coulter 1992 sec. 3: circle-line
    intersection) and keeps the larger root in range -- the exit point, i.e.
    further along the path -- so the chosen point always leads the robot
    forward rather than back toward where it entered the circle.
    """
    dx, dy = b[0] - a[0], b[1] - a[1]
    fx, fy = a[0] - p[0], a[1] - p[1]
    aa = dx * dx + dy * dy
    if aa < 1e-12:
        return None
    bb = 2.0 * (fx * dx + fy * dy)
    cc = fx * fx + fy * fy - radius * radius
    disc = bb * bb - 4.0 * aa * cc
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    for t in ((-bb + sq) / (2.0 * aa), (-bb - sq) / (2.0 * aa)):
        if 0.0 <= t <= 1.0:
            return t
    return None


def lookahead_point(
    path: tuple[Point, ...],
    start_index: int,
    robot_xy: Point,
    lookahead_distance: float,
) -> Point:
    """First forward intersection of the lookahead circle with the path at or
    after ``start_index``; falls back to the path end when the remaining path
    is shorter than the lookahead distance."""
    for i in range(start_index, len(path) - 1):
        t = segment_circle_forward_t(robot_xy, path[i], path[i + 1], lookahead_distance)
        if t is not None:
            a, b = path[i], path[i + 1]
            return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
    # No segment crosses the lookahead circle -- the remaining path is
    # shorter than L_d, so aim at the path's end (the goal).
    return path[-1]
