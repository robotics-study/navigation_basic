"""Reciprocal Velocity Obstacle (van den Berg, Lin & Manocha 2008,
DOI 10.1109/ROBOT.2008.4543489): identical to VO except each cone's apex is
shifted from the obstacle's velocity toward the midpoint of both agents'
velocities (`reciprocity`, 0.5 by default), so each side of a symmetric
encounter absorbs half the avoidance effort instead of both assuming the
other holds course -- the fix for VO's reciprocal-dance oscillation.
"""

from __future__ import annotations

import math

from navigation.core.params import ParamSet
from navigation.core.types import Point, RobotState

from ._velocity_obstacle import (
    DynamicObstacle,
    VelocityObstaclePlanner,
    rvo_apex,
    select_sampled_velocity,
)


class Rvo(VelocityObstaclePlanner):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._speed_samples = params.get_int("speed_samples")
        self._angle_samples = params.get_int("angle_samples")
        self._reciprocity = params.get_float("reciprocity")

    @property
    def name(self) -> str:
        return "rvo"

    def _select_velocity(
        self,
        v_pref: Point,
        neighbors: tuple[DynamicObstacle, ...],
        statics: tuple[DynamicObstacle, ...],
        state: RobotState,
        dt: float,
    ) -> tuple[Point, tuple[tuple[float, ...], ...]]:
        x, y, theta = state.pose
        v_self = (state.v * math.cos(theta), state.v * math.sin(theta))
        reciprocity = self._reciprocity
        return select_sampled_velocity(
            v_pref,
            neighbors + statics,
            (x, y),
            self._agent_radius,
            self._neighbor_dist,
            self._time_horizon,
            self._max_speed,
            self._speed_samples,
            self._angle_samples,
            apex_of=lambda o: rvo_apex(v_self, o.velocity, reciprocity),
        )
