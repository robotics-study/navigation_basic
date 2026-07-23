"""Optimal Reciprocal Collision Avoidance (van den Berg, Guy, Lin & Manocha
2011, DOI 10.1007/978-3-642-19457-3_1): replaces VO/RVO's sampled candidate
grid with an exact half-plane per obstacle plus a deterministic 2D linear
program (RVO2's linearProgram1/2), falling back to a penetration-minimizing
3D solve when the constraints are jointly infeasible. Static obstacles use a
shorter, separately-configured time horizon than moving neighbors, since a
wall's "collision" urgency isn't governed by the same lookahead as another
agent's.
"""

from __future__ import annotations

import math

from navigation.core.params import ParamSet
from navigation.core.types import Point, RobotState

from ._velocity_obstacle import (
    DynamicObstacle,
    HalfPlane,
    VelocityObstaclePlanner,
    halfplane_to_constraint,
    linear_program_2d,
    linear_program_3d,
    orca_half_plane,
)


class Orca(VelocityObstaclePlanner):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._time_horizon_obst = params.get_float("time_horizon_obst")

    @property
    def name(self) -> str:
        return "orca"

    def _half_planes_for(
        self,
        obstacles: tuple[DynamicObstacle, ...],
        pos: Point,
        v_self: Point,
        dt: float,
        tau: float,
    ) -> list[HalfPlane]:
        planes: list[HalfPlane] = []
        for o in obstacles:
            rel_pos = (o.position[0] - pos[0], o.position[1] - pos[1])
            if math.hypot(*rel_pos) >= self._neighbor_dist + o.radius:
                continue
            rel_vel = (v_self[0] - o.velocity[0], v_self[1] - o.velocity[1])
            planes.append(
                orca_half_plane(rel_pos, rel_vel, v_self, self._agent_radius + o.radius, tau, dt)
            )
        return planes

    def _select_velocity(
        self,
        v_pref: Point,
        neighbors: tuple[DynamicObstacle, ...],
        statics: tuple[DynamicObstacle, ...],
        state: RobotState,
        dt: float,
    ) -> tuple[Point, tuple[tuple[float, ...], ...]]:
        x, y, theta = state.pose
        pos = (x, y)
        v_self = (state.v * math.cos(theta), state.v * math.sin(theta))
        planes = self._half_planes_for(neighbors, pos, v_self, dt, self._time_horizon)
        planes += self._half_planes_for(statics, pos, v_self, dt, self._time_horizon_obst)
        ok, v_new, fail = linear_program_2d(planes, v_pref, self._max_speed)
        if not ok:
            v_new = linear_program_3d(planes, fail, v_pref, self._max_speed)
        constraints = tuple(halfplane_to_constraint(p) for p in planes)
        return v_new, constraints
