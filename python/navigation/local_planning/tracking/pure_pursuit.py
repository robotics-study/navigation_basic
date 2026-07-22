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
from ._path import advance_progress_index, lookahead_point

# Below this curvature the clamp-recompute step (v = omega / kappa) would divide
# by a near-zero denominator. At |kappa| this small, kappa*v is already far under
# any reasonable max_omega so the clamp branch never actually fires in practice --
# this guards the division defensively rather than tuning any real behavior.
_KAPPA_EPS = 1e-9


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

        self._progress_index = advance_progress_index(path, robot_xy, self._progress_index)
        lookahead_distance = self.params.get_float("lookahead_distance")
        target = lookahead_point(path, self._progress_index, robot_xy, lookahead_distance)

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
