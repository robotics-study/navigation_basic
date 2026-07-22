"""Shared reference-path geometry for tracking planners.

Path trackers (Pure Pursuit and its descendants) all need the same primitives:
project a probe point onto the path, advance a monotonic progress index along
it, and intersect a lookahead circle with the polyline. Kept as free functions
so each planner owns its progress state while sharing the geometry. Mirrors the
C++ `local_planning/tracking/path.hpp`.
"""

from __future__ import annotations

import math

from navigation.core.types import Point


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
