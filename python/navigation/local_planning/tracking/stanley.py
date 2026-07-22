"""Stanley path tracker (Thrun et al. 2006, DARPA Grand Challenge, sec. 9.2),
with the k_soft low-speed softening term from Hoffmann et al. 2007.

Unlike Pure Pursuit's single lookahead arc, Stanley steers on two errors at
once: heading misalignment with the path tangent and lateral (crosstrack)
offset, both measured at the front axle."""

from __future__ import annotations

import math

from navigation.core.capabilities import Capability, ObstacleQuery
from navigation.core.params import ParamSet
from navigation.core.planner import ObstacleLocalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import LocalTask, Point, RobotState, VelocityCommand

from .._geometry import wrap_to_pi
from ._path import advance_progress_index, closest_point_on_segment, sq_dist

# Below this |tan(delta)| the clamp-recompute step (v = omega*L/tan(delta)) would
# divide by a near-zero denominator. max_steer is declared < pi/2 (see
# stanley.yaml), so tan(delta) itself never diverges; this only guards the
# division defensively, mirroring pure_pursuit.py's _KAPPA_EPS.
_TAN_EPS = 1e-9

# A segment shorter than this (squared) has no defined tangent direction --
# same degenerate-segment threshold as _path.closest_point_on_segment, so the
# two guards agree on what counts as a zero-length segment.
_SEG_EPS_SQ = 1e-12


class Stanley(ObstacleLocalPlanner):
    """`required_capabilities()` still declares `ObstacleQuery` for the same
    reason as PurePursuit: the simulator's closed-loop contract is defined over
    that capability regardless of whether a given tracker queries obstacles."""

    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        # Index of the reference-path segment currently tracked at the front
        # axle. Monotonic forward-only, same rationale as PurePursuit.
        self._progress_index = 0

    @property
    def name(self) -> str:
        return "stanley"

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

        # Speed profile first: the steering law below divides by (k_soft + v),
        # so v must be settled before delta is computed.
        max_speed = self.params.get_float("max_speed")
        slow_radius = self.params.get_float("slow_radius")
        remaining = math.hypot(task.goal[0] - x, task.goal[1] - y)
        v = max_speed * min(1.0, remaining / slow_radius)

        # Front-axle point: the original paper defines both errors here
        # (Thrun 2006 sec. 9.2), not at the robot's rear-axle/center pose.
        wheelbase = self.params.get_float("wheelbase")
        front: Point = (x + wheelbase * math.cos(theta), y + wheelbase * math.sin(theta))

        self._progress_index = advance_progress_index(path, front, self._progress_index)
        # The tangent divides by the segment length, so a duplicated waypoint
        # (zero-length segment) or a single-point path has no defined tangent:
        # use the first non-degenerate segment at or after the progress index,
        # and fall back to aiming at the path's end otherwise -- a finite
        # command instead of a NaN silently propagating into the simulator
        # (the path-end fallback mirrors Pure Pursuit's lookahead convention).
        segment: tuple[Point, Point] | None = None
        for j in range(self._progress_index, len(path) - 1):
            if sq_dist(path[j], path[j + 1]) >= _SEG_EPS_SQ:
                segment = (path[j], path[j + 1])
                break
        if segment is not None:
            a, b = segment
            seg_len = math.hypot(b[0] - a[0], b[1] - a[1])
            tx, ty = (b[0] - a[0]) / seg_len, (b[1] - a[1]) / seg_len
            theta_path = math.atan2(ty, tx)
            foot = closest_point_on_segment(front, a, b)
            # Cross product of the tangent with (front - foot): positive when the
            # front axle sits to the path's left. This is a mirror-image sign
            # convention from the paper's e (right-positive) -- same steering law,
            # only the sign folds differently below.
            e = tx * (front[1] - foot[1]) - ty * (front[0] - foot[0])
        else:
            end = path[-1]
            dx, dy = end[0] - front[0], end[1] - front[1]
            # If the front axle sits on the path's end there is no direction
            # left to align with either -- hold the current heading.
            theta_path = math.atan2(dy, dx) if dx * dx + dy * dy >= _SEG_EPS_SQ else theta
            foot = end
            e = 0.0
        psi = wrap_to_pi(theta_path - theta)

        k_gain = self.params.get_float("k_gain")
        k_soft = self.params.get_float("k_soft")
        delta_raw = psi - math.atan(k_gain * e / (k_soft + v))
        max_steer = self.params.get_float("max_steer")
        delta = max(-max_steer, min(max_steer, delta_raw))

        # Rear-axle kinematic bicycle theta_dot = (v/L)*tan(delta) is exactly
        # the unicycle equation with omega = (v/L)*tan(delta) -- not an
        # approximation, just a different parametrization of the same motion
        # (tire slip / vehicle dynamics beyond this kinematic model are not
        # captured).
        max_omega = self.params.get_float("max_omega")
        omega_raw = v * math.tan(delta) / wheelbase
        omega = max(-max_omega, min(max_omega, omega_raw))
        if omega != omega_raw and abs(math.tan(delta)) > _TAN_EPS:
            # Clamp changed the turn rate -- recompute v so the executed
            # (v, omega) still traces the commanded curvature instead of
            # silently understeering relative to delta (PP's clamp-recompute
            # pattern mirrored here).
            v = omega * wheelbase / math.tan(delta)

        if recorder is not None:
            recorder.candidate_evaluated(
                [foot[0], foot[1]], delta, data={"e": e, "psi": psi, "v": v}
            )
        return VelocityCommand(v, omega)
