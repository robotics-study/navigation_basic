"""Velocity Obstacle (Fiorini & Shiller 1998, DOI 10.1177/027836499801700706):
every tick, build one truncated cone per nearby obstacle (apex = the
obstacle's own velocity) and pick the candidate velocity closest to the
goal-seeking preferred velocity that lies outside every cone.
"""

from __future__ import annotations

from navigation.core.params import ParamSet
from navigation.core.types import Point, RobotState

from ._velocity_obstacle import DynamicObstacle, VelocityObstaclePlanner, select_sampled_velocity


class Vo(VelocityObstaclePlanner):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._speed_samples = params.get_int("speed_samples")
        self._angle_samples = params.get_int("angle_samples")

    @property
    def name(self) -> str:
        return "vo"

    def _select_velocity(
        self,
        v_pref: Point,
        neighbors: tuple[DynamicObstacle, ...],
        statics: tuple[DynamicObstacle, ...],
        state: RobotState,
        dt: float,
    ) -> tuple[Point, tuple[tuple[float, ...], ...]]:
        x, y, _theta = state.pose
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
            apex_of=lambda o: o.velocity,
        )
