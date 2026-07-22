"""Regulated Pure Pursuit (Macenski, Singh, Martin & Gines 2023, "Regulated
Pure Pursuit for Robot Path Tracking"; Nav2's default local controller).

Builds on the plain lookahead-arc law (Coulter 1992) but adds three
regulations the original never had: an adaptive lookahead distance
(Campbell 2007), a curvature-proportional speed cap so tight corners are
taken slower, and a proximity-proportional speed cap so the robot slows near
obstacles. It also predicts the command arc a short distance ahead and stops
outright if that prediction intersects an obstacle. Geometry (progress index
+ lookahead point) is shared with `pure_pursuit.py` via `_path`; the
regulation and command-assembly logic below is deliberately NOT shared with
`pure_pursuit.py` even though the curvature formula and clamp-recompute step
look similar -- factoring that out would couple this planner's regulation
logic to Pure Pursuit's plain command, blocking either one from evolving
independently."""

from __future__ import annotations

import math

from navigation.core.capabilities import Capability, ObstacleQuery
from navigation.core.params import ParamSet
from navigation.core.planner import ObstacleLocalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import Footprint, LocalTask, Point, RobotState, VelocityCommand

from .._geometry import wrap_to_pi
from ._path import advance_progress_index, lookahead_point

# Below this curvature, both the "no curvature regulation" branch and the
# clamp-recompute step (v = omega / kappa) would be operating on a near-zero
# denominator. At |kappa| this small the arc is effectively straight, so
# skipping regulation and guarding the division are the same threshold.
# Identical value to pure_pursuit.py's _KAPPA_EPS -- same reasoning, defined
# independently per this module's own-computation design.
_KAPPA_EPS = 1e-9

# Guard band (radians) around alpha = 0 and alpha = +-pi for the lookahead
# collision-check arc length (step 5 below): both neighborhoods make
# L_d*alpha/sin(alpha) degenerate, so both fall back to L_d directly. Wide
# enough (~3 degrees) to cover real near-antipodal targets, not just exact
# floating-point equality -- see the WHY comment at the call site for why
# alpha actually reaches +-pi in practice.
_ARC_ALPHA_MARGIN = 0.05


class RegulatedPurePursuit(ObstacleLocalPlanner):
    """`required_capabilities()` declares `ObstacleQuery` for real use here
    (unlike plain Pure Pursuit): proximity regulation and the lookahead
    collision check both query it every tick."""

    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        # Index of the reference-path segment currently tracked. Monotonic
        # forward-only, same rationale as PurePursuit/Stanley.
        self._progress_index = 0

    @property
    def name(self) -> str:
        return "regulated_pure_pursuit"

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

        # 1. Adaptive lookahead (Macenski 2023 sec. 3.1; lineage Campbell 2007):
        # scales with current speed so a fast robot looks further ahead than a
        # slow one, instead of chasing a fixed-radius point regardless of pace.
        lookahead_time = self.params.get_float("lookahead_time")
        min_lookahead = self.params.get_float("min_lookahead")
        max_lookahead = self.params.get_float("max_lookahead")
        # Clamp order is min(max(.), max_lookahead): if min_lookahead is
        # declared larger than max_lookahead, max_lookahead wins.
        lookahead_distance = min(max(lookahead_time * state.v, min_lookahead), max_lookahead)

        # 2. Target point: same geometry as Pure Pursuit, shared via _path.
        self._progress_index = advance_progress_index(path, robot_xy, self._progress_index)
        target = lookahead_point(path, self._progress_index, robot_xy, lookahead_distance)

        # 3. Commanded curvature (Coulter 1992 geometry, independently
        # recomputed here rather than imported -- see module docstring).
        alpha = wrap_to_pi(math.atan2(target[1] - y, target[0] - x) - theta)
        kappa = 2.0 * math.sin(alpha) / lookahead_distance

        # 4. Speed regulation: v is the minimum of three independent caps.
        max_speed = self.params.get_float("max_speed")
        slow_radius = self.params.get_float("slow_radius")
        remaining = math.hypot(task.goal[0] - x, task.goal[1] - y)
        v_goal = max_speed * min(1.0, remaining / slow_radius)

        regulated_min_radius = self.params.get_float("regulated_min_radius")
        if abs(kappa) <= _KAPPA_EPS:
            v_curv = max_speed
        else:
            radius = 1.0 / abs(kappa)
            v_curv = (
                max_speed * (radius / regulated_min_radius)
                if radius < regulated_min_radius
                else max_speed
            )

        # Proximity heuristic (Macenski 2023 sec. 3.2): the paper scales by
        # costmap cost, this implementation scales by EDT distance to the
        # nearest obstacle instead -- a simplification (no costmap layer here).
        proximity_distance = self.params.get_float("proximity_distance")
        clearance = space.distance_to_nearest((x, y)) - self.params.get_float("footprint_radius")
        v_prox = (
            max_speed * max(clearance, 0.0) / proximity_distance
            if clearance < proximity_distance
            else max_speed
        )

        min_regulated_speed = self.params.get_float("min_regulated_speed")
        # The min_regulated_speed floor only applies to the three caps above --
        # never to the collision stop in step 5, which must be able to reach 0.
        v = max(min(v_goal, v_curv, v_prox), min_regulated_speed)

        # 5. Lookahead collision check (Macenski 2023 sec. 3.3): walk the
        # commanded constant-curvature arc out to the target's arc length and
        # stop outright if it runs into an obstacle, instead of discovering
        # the collision only after already committing to the command.
        # sin(alpha) -> 0 both as alpha -> 0 and as alpha -> +-pi, so the arc
        # length L_d*alpha/sin(alpha) is degenerate at both ends: near alpha=0
        # it correctly limits to L_d, but near alpha=+-pi the same formula
        # blows up (alpha stays ~pi while sin(alpha) shrinks toward 0),
        # predicting an absurdly long arc instead of a short one. The shared
        # `_path.lookahead_point` can hand back a target *behind* the robot
        # (alpha ~= +-pi) whenever the robot is tracking a straight segment
        # almost exactly on-line and comes within lookahead_distance of that
        # segment's end -- its forward-circle intersection then falls just
        # past the segment's t=1 endpoint, so the search returns the
        # backward intersection instead of continuing on to the next
        # segment. That is a real, observable case (not a hypothetical one),
        # so both degenerate neighborhoods use the same L_d fallback here --
        # a wide-enough guard band on alpha (not just a division-by-zero
        # guard) rather than only reacting exactly at alpha == 0.
        arc_length = (
            lookahead_distance
            if abs(alpha) < _ARC_ALPHA_MARGIN or abs(alpha) > math.pi - _ARC_ALPHA_MARGIN
            else lookahead_distance * alpha / math.sin(alpha)
        )

        collision_check_step = self.params.get_float("collision_check_step")
        footprint = Footprint(self.params.get_float("footprint_radius"))
        blocked = False
        s = collision_check_step
        while s <= arc_length:
            pose = _propagate_arc((x, y, theta), kappa, s)
            if space.is_collision(footprint, pose):
                blocked = True
                break
            s += collision_check_step

        # Trace dict is only ever built inside this closure, which itself only
        # runs when recorder is not None -- keeps the hot path allocation-free
        # when tracing is off, same as every other planner's recorder guard.
        def emit(blocked_value: float) -> None:
            if recorder is not None:
                recorder.candidate_evaluated(
                    [target[0], target[1]],
                    kappa,
                    data={
                        "alpha": alpha,
                        "lookahead": lookahead_distance,
                        "curvature_scale": v_curv / max_speed,
                        "proximity_scale": v_prox / max_speed,
                        "blocked": blocked_value,
                    },
                )

        if blocked:
            emit(1.0)
            return VelocityCommand(0.0, 0.0)

        # 6. Angular velocity, clamped with curvature-preserving recompute
        # (same pattern as pure_pursuit.py's _KAPPA_EPS clamp).
        max_omega = self.params.get_float("max_omega")
        omega_raw = kappa * v
        omega = max(-max_omega, min(max_omega, omega_raw))
        if omega != omega_raw and abs(kappa) > _KAPPA_EPS:
            v = omega / kappa

        emit(0.0)
        return VelocityCommand(v, omega)


def _propagate_arc(
    pose: tuple[float, float, float], kappa: float, s: float
) -> tuple[float, float, float]:
    """Pose reached after arc length ``s`` along the constant-curvature arc
    ``kappa`` starting at ``pose`` -- closed-form circular-arc propagation
    (same family as the simulator's `integrate_unicycle`, parametrized by arc
    length instead of time since the collision check walks distance, not
    ticks)."""
    x, y, theta = pose
    if abs(kappa) < _KAPPA_EPS:
        return (x + s * math.cos(theta), y + s * math.sin(theta), theta)
    new_theta = theta + kappa * s
    x2 = x + (math.sin(new_theta) - math.sin(theta)) / kappa
    y2 = y - (math.cos(new_theta) - math.cos(theta)) / kappa
    return (x2, y2, wrap_to_pi(new_theta))
