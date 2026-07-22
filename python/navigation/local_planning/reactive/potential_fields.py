"""Potential Fields (Khatib 1986): sum an attractive pull toward the goal with
a FIRAS repulsive push from every obstacle within range, and steer toward the
resultant force every control tick. Stateless -- each tick is a fresh vector
sum with no memory of past ticks, so `reset()` stays the base no-op.
"""

from __future__ import annotations

import math

from navigation.core.capabilities import Capability, ObstacleQuery
from navigation.core.params import ParamSet
from navigation.core.planner import ObstacleLocalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import LocalTask, RobotState, VelocityCommand

from .._geometry import wrap_to_pi
from ._steering import heading_command


class PotentialFields(ObstacleLocalPlanner):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._k_att = params.get_float("k_att")
        self._k_rep = params.get_float("k_rep")
        self._influence_radius = params.get_float("influence_radius")
        self._k_v = params.get_float("k_v")
        self._k_omega = params.get_float("k_omega")
        self._max_speed = params.get_float("max_speed")
        self._max_omega = params.get_float("max_omega")
        # Contact clamp for the repulsive 1/d term: the footprint radius is the
        # closest the robot center can approach an obstacle center before
        # collision, so it doubles as the smallest physically meaningful d.
        self._d_min = params.get_float("footprint_radius")

    @property
    def name(self) -> str:
        return "potential_fields"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.OBSTACLE_QUERY}

    def compute_command(
        self,
        space: ObstacleQuery,
        state: RobotState,
        task: LocalTask,
        dt: float,
        recorder: TraceRecorder | None = None,
    ) -> VelocityCommand:
        x, y, theta = state.pose
        gx, gy = task.goal[0], task.goal[1]
        fx_att = self._k_att * (gx - x)
        fy_att = self._k_att * (gy - y)

        fx_rep = 0.0
        fy_rep = 0.0
        for ox, oy in space.occupied_within((x, y), self._influence_radius):
            dx, dy = x - ox, y - oy
            d = max(math.hypot(dx, dy), self._d_min)
            if d >= self._influence_radius:
                continue
            magnitude = self._k_rep * (1.0 / d - 1.0 / self._influence_radius) * (1.0 / (d * d))
            fx_rep += magnitude * dx / d
            fy_rep += magnitude * dy / d

        fx = fx_att + fx_rep
        fy = fy_att + fy_rep

        if recorder is not None:
            recorder.force_computed(
                state.pose,
                data={
                    "fx_att": fx_att,
                    "fy_att": fy_att,
                    "fx_rep": fx_rep,
                    "fy_rep": fy_rep,
                    "fx": fx,
                    "fy": fy,
                },
            )

        theta_d = math.atan2(fy, fx)
        v_eff = min(self._max_speed, self._k_v * math.hypot(fx, fy))
        return heading_command(wrap_to_pi(theta_d - theta), self._k_omega, v_eff, self._max_omega)
