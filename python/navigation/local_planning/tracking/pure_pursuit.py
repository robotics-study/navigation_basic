"""Pure Pursuit path tracker (Coulter 1992, CMU-RI-TR-92-01).

Chases the point where a lookahead circle centered on the robot meets the
reference path, steering along the single constant-curvature arc that passes
through it. Purely geometric: no obstacle awareness by design (see the class
docstring for why `required_capabilities()` still binds `ObstacleQuery`).
"""

from __future__ import annotations

import math

from navigation.core.capabilities import Capability, ObstacleQuery
from navigation.core.params import ParamSet
from navigation.core.planner import ObstacleLocalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import LocalTask, Point, RobotState, VelocityCommand

from .._geometry import wrap_to_pi

# Below this curvature the clamp-recompute step (v = omega / kappa) would divide
# by a near-zero denominator. At |kappa| this small, kappa*v is already far under
# any reasonable max_omega so the clamp branch never actually fires in practice --
# this guards the division defensively rather than tuning any real behavior.
_KAPPA_EPS = 1e-9


def _closest_point_on_segment(p: Point, a: Point, b: Point) -> Point:
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-12:
        return a
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / seg_len_sq))
    return (ax + t * dx, ay + t * dy)


def _sq_dist(a: Point, b: Point) -> float:
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def _segment_circle_forward_t(p: Point, a: Point, b: Point, radius: float) -> float | None:
    """Forward-most intersection of the robot-centered lookahead circle with
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


class PurePursuit(ObstacleLocalPlanner):
    """`required_capabilities()` still declares `ObstacleQuery` even though this
    planner never queries obstacles: the closed-loop execution contract
    (collision / clearance) is defined over that capability and the simulator
    always requires it, so binding the same view here keeps the simulator
    single-path instead of branching on "does this planner see obstacles"."""

    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        # Index of the reference-path segment the robot is currently tracking.
        # Monotonic forward-only so a self-crossing path never snaps the pursuit
        # point backward to an earlier, geometrically-closer crossing.
        self._progress_index = 0

    @property
    def name(self) -> str:
        return "pure_pursuit"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.OBSTACLE_QUERY}

    def requires_reference_path(self) -> bool:
        return True

    def reset(self) -> None:
        self._progress_index = 0

    def compute_command(
        self,
        space: ObstacleQuery,
        state: RobotState,
        task: LocalTask,
        dt: float,
        recorder: TraceRecorder | None = None,
    ) -> VelocityCommand:
        path = task.reference_path
        x, y, theta = state.pose
        robot_xy: Point = (x, y)

        self._progress_index = self._advance_progress_index(path, robot_xy)
        lookahead_distance = self.params.get_float("lookahead_distance")
        target = self._lookahead_point(path, self._progress_index, robot_xy, lookahead_distance)

        alpha = wrap_to_pi(math.atan2(target[1] - y, target[0] - x) - theta)
        kappa = 2.0 * math.sin(alpha) / lookahead_distance

        max_speed = self.params.get_float("max_speed")
        slow_radius = self.params.get_float("slow_radius")
        remaining = math.hypot(task.goal[0] - x, task.goal[1] - y)
        v = max_speed * min(1.0, remaining / slow_radius)

        max_omega = self.params.get_float("max_omega")
        omega_raw = kappa * v
        omega = max(-max_omega, min(max_omega, omega_raw))
        if omega != omega_raw and abs(kappa) > _KAPPA_EPS:
            # Clamp changed the turn rate -- recompute v so the executed (v, omega)
            # still traces the commanded curvature kappa instead of silently
            # understeering relative to the geometric lookahead arc.
            v = omega / kappa

        if recorder is not None:
            recorder.candidate_evaluated([target[0], target[1]], kappa, data={"alpha": alpha})
        return VelocityCommand(v, omega)

    def _advance_progress_index(self, path: tuple[Point, ...], robot_xy: Point) -> int:
        if len(path) < 2:
            return self._progress_index
        best_index = self._progress_index
        best_sq_dist = float("inf")
        for i in range(self._progress_index, len(path) - 1):
            closest = _closest_point_on_segment(robot_xy, path[i], path[i + 1])
            sq_dist = _sq_dist(robot_xy, closest)
            # <=, not <: consecutive segments share their joint endpoint, so a
            # robot sitting exactly at a corner ties every segment ending/starting
            # there. Preferring the later (more forward) segment on a tie keeps
            # progress advancing through the corner instead of latching onto the
            # segment just traveled.
            if sq_dist <= best_sq_dist:
                best_sq_dist = sq_dist
                best_index = i
        return best_index

    def _lookahead_point(
        self,
        path: tuple[Point, ...],
        start_index: int,
        robot_xy: Point,
        lookahead_distance: float,
    ) -> Point:
        for i in range(start_index, len(path) - 1):
            t = _segment_circle_forward_t(robot_xy, path[i], path[i + 1], lookahead_distance)
            if t is not None:
                a, b = path[i], path[i + 1]
                return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        # No segment crosses the lookahead circle -- the remaining path is
        # shorter than L_d, so aim at the path's end (the goal).
        return path[-1]
