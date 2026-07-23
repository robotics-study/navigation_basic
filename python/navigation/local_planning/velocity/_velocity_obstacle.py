"""Shared geometry/LP machinery for the velocity-obstacle family: VO/RVO/ORCA
choose the agent's *next absolute velocity* directly in velocity space instead
of DWA's command-space rollout scoring. `_`-prefixed family helper module,
mirroring the `reactive/_steering.py` (now promoted into `.._geometry`) and
`band/_band.py` precedent of one shared module per family.

Fiorini & Shiller 1998 (VO, truncated cone) / van den Berg, Lin & Manocha 2008
(RVO, reciprocal apex) / van den Berg, Guy, Lin & Manocha 2011 (ORCA,
half-plane + linear program) all reduce to: build one forbidden region per
nearby obstacle, then pick the admissible velocity closest to a goal-seeking
preferred velocity.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from collections.abc import Callable, Sequence
from dataclasses import dataclass

from navigation.core.capabilities import Capability, ObstacleQuery
from navigation.core.params import ParamSet
from navigation.core.planner import ObstacleLocalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import LocalTask, Point, Pose, RobotState, VelocityCommand

from .._geometry import heading_command, wrap_to_pi

# Numerical guard for divisions by a near-zero vector length / distance -- not a
# tunable parameter (no meaningful physical unit), so a module constant rather
# than a config value.
_EPS = 1e-9
# Added to an infeasible (VO/RVO-cone-violating) candidate's cost so any
# feasible candidate always outranks it, however close the infeasible one sits
# to v_pref. Not tunable (only its relative dominance over real costs matters).
_PENALTY = 1e6


@dataclass(frozen=True)
class DynamicObstacle:
    position: Point
    velocity: Point
    radius: float


@dataclass(frozen=True)
class Cone:
    """Truncated velocity obstacle (Fiorini & Shiller 1998): the set of
    (absolute) velocities that, held for `tau` seconds, put the agent inside
    `radius` of the obstacle. `left`/`right` are unit boundary rays from
    `apex` along the two tangents; `full` marks an already-overlapping pair
    (radius >= dist), whose VO is the entire velocity plane."""

    apex: Point
    axis: Point
    cos_half: float
    dist: float
    radius: float
    tau: float
    full: bool
    left: Point
    right: Point


# ORCA half-plane: feasible region is {v : dot(v - point, normal) >= 0}, normal unit.
HalfPlane = tuple[Point, Point]


def preferred_velocity(pose: Pose, goal: Pose, max_speed: float) -> Point:
    """Goal-seeking velocity: full speed toward the goal, tapering to a stop
    once within max_speed meters of it so the episode settles into REACHED
    instead of orbiting/overshooting (RVO2's goal heuristic)."""
    dx, dy = goal[0] - pose[0], goal[1] - pose[1]
    dist = math.hypot(dx, dy)
    if dist < _EPS:
        return (0.0, 0.0)
    speed = min(max_speed, dist)
    return (dx / dist * speed, dy / dist * speed)


def velocity_to_command(
    v_new: Point, theta: float, max_omega: float, heading_gain: float
) -> VelocityCommand:
    """Differential-drive projection of a velocity-space target onto (v, omega),
    reusing the reactive family's turn-in-place-then-drive law: the chosen
    speed itself becomes the law's speed cap, cos-gated by the heading error."""
    speed = math.hypot(v_new[0], v_new[1])
    if speed < _EPS:
        return VelocityCommand(0.0, 0.0)
    desired = math.atan2(v_new[1], v_new[0])
    theta_err = wrap_to_pi(desired - theta)
    return heading_command(theta_err, heading_gain, speed, max_omega)


def truncated_vo_cone(rel_pos: Point, combined_radius: float, apex_vel: Point, tau: float) -> Cone:
    px, py = rel_pos
    dist = math.hypot(px, py)
    if dist <= combined_radius + _EPS:
        # Already overlapping: every relative velocity leads to (deeper)
        # penetration, so the forbidden region is the whole plane.
        return Cone(
            apex=apex_vel,
            axis=(1.0, 0.0),
            cos_half=-1.0,
            dist=dist,
            radius=combined_radius,
            tau=tau,
            full=True,
            left=(1.0, 0.0),
            right=(1.0, 0.0),
        )
    ux, uy = px / dist, py / dist
    sin_half = combined_radius / dist
    cos_half = math.sqrt(max(0.0, 1.0 - sin_half * sin_half))
    left = (ux * cos_half - uy * sin_half, ux * sin_half + uy * cos_half)
    right = (ux * cos_half + uy * sin_half, -ux * sin_half + uy * cos_half)
    return Cone(
        apex=apex_vel,
        axis=(ux, uy),
        cos_half=cos_half,
        dist=dist,
        radius=combined_radius,
        tau=tau,
        full=False,
        left=left,
        right=right,
    )


def in_velocity_obstacle(v: Point, cone: Cone) -> bool:
    if cone.full:
        return True
    wx, wy = v[0] - cone.apex[0], v[1] - cone.apex[1]
    wlen = math.hypot(wx, wy)
    if wlen < _EPS:
        return False  # relative rest never collides
    wproj = wx * cone.axis[0] + wy * cone.axis[1]
    if wproj <= 0.0:
        return False  # moving away from the obstacle
    cos_ang = wproj / wlen
    if cos_ang < cone.cos_half:
        return False  # outside the cone's angular span
    # tau-truncation (near-plane approximation): the gap must close within tau.
    if wproj < (cone.dist - cone.radius) / cone.tau:
        return False
    return True


def cone_to_constraint(cone: Cone) -> tuple[float, ...]:
    return (cone.apex[0], cone.apex[1], cone.left[0], cone.left[1], cone.right[0], cone.right[1])


def halfplane_to_constraint(plane: HalfPlane) -> tuple[float, ...]:
    point, normal = plane
    return (point[0], point[1], normal[0], normal[1])


def rvo_apex(v_self: Point, v_other: Point, reciprocity: float) -> Point:
    """RVO apex (van den Berg et al. 2008): shift the VO's apex from the other
    agent's velocity toward the midpoint of both velocities so each side
    absorbs half the avoidance effort. reciprocity=0 recovers plain VO
    (other bears all responsibility), 1 collapses the cone onto self."""
    return (
        (1.0 - reciprocity) * v_other[0] + reciprocity * v_self[0],
        (1.0 - reciprocity) * v_other[1] + reciprocity * v_self[1],
    )


def sample_reachable_velocities(
    v_pref: Point, max_speed: float, speed_samples: int, angle_samples: int
) -> list[Point]:
    """Deterministic polar candidate grid for VO/RVO (no RNG): speed-outer /
    angle-inner traversal so py/cpp/TS scoring and tie-breaking stay
    bit-identical. Candidate 0 is v_pref itself (clamped to max_speed) so a
    fully unobstructed tick costs exactly 0 and wins every tie."""
    speed = math.hypot(v_pref[0], v_pref[1])
    if speed > max_speed:
        v0 = (v_pref[0] / speed * max_speed, v_pref[1] / speed * max_speed)
    else:
        v0 = v_pref
    out = [v0]
    for si in range(speed_samples + 1):
        s = max_speed * si / speed_samples
        for ai in range(angle_samples):
            ang = 2.0 * math.pi * ai / angle_samples
            out.append((s * math.cos(ang), s * math.sin(ang)))
    return out


def static_obstacles(
    space: ObstacleQuery, center: Point, sensor_radius: float, obstacle_radius: float
) -> tuple[DynamicObstacle, ...]:
    """Occupied cells within `sensor_radius` folded into velocity-0 obstacles so
    a single VO/RVO/ORCA code path handles static walls and moving agents
    alike. `obstacle_radius` (a disc approximation of one occupied cell) is a
    declared config value rather than derived from grid resolution: ObstacleQuery
    does not expose resolution to algorithm modules (capability abstraction),
    so the disc size is an explicit tunable instead of an implicit map detail.
    `occupied_within` is already row/col ascending (deterministic)."""
    return tuple(
        DynamicObstacle(p, (0.0, 0.0), obstacle_radius)
        for p in space.occupied_within(center, sensor_radius)
    )


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def select_sampled_velocity(
    v_pref: Point,
    obstacles: tuple[DynamicObstacle, ...],
    pos: Point,
    agent_radius: float,
    neighbor_dist: float,
    time_horizon: float,
    max_speed: float,
    speed_samples: int,
    angle_samples: int,
    apex_of: Callable[[DynamicObstacle], Point],
) -> tuple[Point, tuple[tuple[float, ...], ...]]:
    """Candidate-grid velocity selection shared by VO and RVO: the two differ
    only in how each obstacle's cone apex is placed (`apex_of`), so this owns
    the common cone-building / scan / cost-with-penalty loop (Fiorini &
    Shiller 1998 eq. for VO; van den Berg et al. 2008 for the reciprocal
    apex), avoiding a duplicated scan between vo.py and rvo.py."""
    cones = [
        truncated_vo_cone(
            (o.position[0] - pos[0], o.position[1] - pos[1]),
            agent_radius + o.radius,
            apex_of(o),
            time_horizon,
        )
        for o in obstacles
        if _dist(o.position, pos) < neighbor_dist + o.radius
    ]
    candidates = sample_reachable_velocities(v_pref, max_speed, speed_samples, angle_samples)
    best: Point = candidates[0]
    best_cost = math.inf
    for v in candidates:  # fixed traversal order -- see sample_reachable_velocities
        violated = any(in_velocity_obstacle(v, c) for c in cones)
        cost = _dist(v, v_pref) + (_PENALTY if violated else 0.0)
        if cost < best_cost:  # strict <: first candidate (v_pref) wins ties
            best_cost, best = cost, v
    constraints = tuple(cone_to_constraint(c) for c in cones)
    return best, constraints


def orca_half_plane(
    rel_pos: Point, rel_vel: Point, v_self: Point, combined_radius: float, tau: float, dt: float
) -> HalfPlane:
    """ORCA line for one obstacle (van den Berg et al. 2011; RVO2 reference
    implementation's Agent::computeNewVelocity). `rel_pos` = other - self,
    `rel_vel` = self - other (both absolute). Returns (point, normal) with
    feasible region `dot(v - point, normal) >= 0`."""
    px, py = rel_pos
    vx, vy = rel_vel
    dist_sq = px * px + py * py
    r = combined_radius
    r_sq = r * r
    if dist_sq > r_sq:  # not currently colliding
        inv_tau = 1.0 / tau
        wx, wy = vx - inv_tau * px, vy - inv_tau * py
        w_len_sq = wx * wx + wy * wy
        dot1 = wx * px + wy * py
        if dot1 < 0.0 and dot1 * dot1 > r_sq * w_len_sq:
            # In front of the truncated cutoff circle: project onto it.
            w_len = math.sqrt(w_len_sq)
            unit_wx, unit_wy = wx / w_len, wy / w_len
            normal = (unit_wx, unit_wy)
            u = ((r * inv_tau - w_len) * unit_wx, (r * inv_tau - w_len) * unit_wy)
        else:
            # Project onto one of the two tangent legs.
            leg = math.sqrt(dist_sq - r_sq)
            if px * wy - py * wx > 0.0:
                dirx = (px * leg - py * r) / dist_sq
                diry = (px * r + py * leg) / dist_sq
            else:
                # RVO2's true right-leg tangent is the negation of the naive
                # mirror of the left-leg formula (Agent.cpp computeNewVelocity):
                # without this flip the derived half-plane normal points INTO
                # the collision cone instead of away from it (verified by the
                # ORCA linear-program unit test).
                dirx = -(px * leg + py * r) / dist_sq
                diry = (px * r - py * leg) / dist_sq
            dot2 = vx * dirx + vy * diry
            u = (dot2 * dirx - vx, dot2 * diry - vy)
            normal = (-diry, dirx)
    else:  # already colliding: project onto the cutoff circle of the control tick
        inv_dt = 1.0 / dt
        wx, wy = vx - inv_dt * px, vy - inv_dt * py
        w_len = math.hypot(wx, wy)
        if w_len < _EPS:
            unit_wx, unit_wy = 1.0, 0.0  # degenerate exact-alignment fallback
        else:
            unit_wx, unit_wy = wx / w_len, wy / w_len
        normal = (unit_wx, unit_wy)
        u = ((r * inv_dt - w_len) * unit_wx, (r * inv_dt - w_len) * unit_wy)
    point = (v_self[0] + 0.5 * u[0], v_self[1] + 0.5 * u[1])
    return point, normal


def _det(a: Point, b: Point) -> float:
    return a[0] * b[1] - a[1] * b[0]


def _dot(a: Point, b: Point) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _sub(a: Point, b: Point) -> Point:
    return (a[0] - b[0], a[1] - b[1])


def _unit(v: Point) -> Point:
    n = math.hypot(v[0], v[1])
    if n < _EPS:
        return (0.0, 0.0)
    return (v[0] / n, v[1] / n)


def _direction_of(normal: Point) -> Point:
    # direction (RVO2's Line.direction) is normal rotated -90 degrees, so that
    # feasibility det(direction, v - point) >= 0 matches dot(v - point, normal) >= 0.
    return (normal[1], -normal[0])


def _normal_of(direction: Point) -> Point:
    return (-direction[1], direction[0])


def _lp1(
    half_planes: Sequence[HalfPlane],
    line_no: int,
    opt_velocity: Point,
    max_speed: float,
    direction_opt: bool,
) -> tuple[bool, Point]:
    """RVO2 linearProgram1, ported to the (point, normal) representation: the
    1D sub-problem of optimizing along line `line_no` subject to the max-speed
    circle and every earlier line. Returns (False, point) on an infeasible
    (empty) interval -- never raises."""
    point, normal = half_planes[line_no]
    direction = _direction_of(normal)
    dot_product = _dot(point, direction)
    discriminant = dot_product * dot_product + max_speed * max_speed - _dot(point, point)
    if discriminant < 0.0:
        return False, point
    sqrt_discriminant = math.sqrt(discriminant)
    t_left = -dot_product - sqrt_discriminant
    t_right = -dot_product + sqrt_discriminant
    for i in range(line_no):
        p_i, n_i = half_planes[i]
        d_i = _direction_of(n_i)
        denominator = _det(direction, d_i)
        numerator = _det(d_i, _sub(point, p_i))
        if abs(denominator) <= _EPS:
            if numerator < 0.0:
                return False, point
            continue
        t = numerator / denominator
        if denominator >= 0.0:
            t_right = min(t_right, t)
        else:
            t_left = max(t_left, t)
        if t_left > t_right:
            return False, point
    if direction_opt:
        t = t_right if _dot(opt_velocity, direction) > 0.0 else t_left
    else:
        t = max(t_left, min(t_right, _dot(direction, _sub(opt_velocity, point))))
    return True, (point[0] + t * direction[0], point[1] + t * direction[1])


def _lp2(
    half_planes: Sequence[HalfPlane], opt_velocity: Point, max_speed: float, direction_opt: bool
) -> tuple[Point, int]:
    """RVO2 linearProgram2: incrementally re-optimize onto each violated line's
    1D sub-problem. Returns (result, fail_index); fail_index == len(half_planes)
    means every line was satisfied."""
    if direction_opt:
        result = (opt_velocity[0] * max_speed, opt_velocity[1] * max_speed)
    elif math.hypot(opt_velocity[0], opt_velocity[1]) > max_speed:
        u = _unit(opt_velocity)
        result = (u[0] * max_speed, u[1] * max_speed)
    else:
        result = opt_velocity
    for i, (point, normal) in enumerate(half_planes):
        if _dot(_sub(result, point), normal) < 0.0:
            saved = result
            ok, candidate = _lp1(half_planes, i, opt_velocity, max_speed, direction_opt)
            if not ok:
                return saved, i
            result = candidate
    return result, len(half_planes)


def linear_program_2d(
    half_planes: Sequence[HalfPlane], v_pref: Point, max_speed: float
) -> tuple[bool, Point, int]:
    result, fail_index = _lp2(half_planes, v_pref, max_speed, False)
    return fail_index == len(half_planes), result, fail_index


def linear_program_3d(
    half_planes: Sequence[HalfPlane], begin_line: int, v_pref: Point, max_speed: float
) -> Point:
    """RVO2 linearProgram3: over-constrained fallback, minimizing total
    penetration across every line from `begin_line` on. Always returns a
    Point -- never raises (hot path). Re-derives (rather than receives) the
    prefix result already satisfied before `begin_line`, since that prefix
    computation is deterministic and bit-identical to what linear_program_2d
    already performed, keeping this function's public signature exactly
    (half_planes, begin_line, v_pref, max_speed)."""
    result, _ = _lp2(half_planes[:begin_line], v_pref, max_speed, False)
    distance = 0.0
    for i in range(begin_line, len(half_planes)):
        point, normal = half_planes[i]
        direction = _direction_of(normal)
        if _dot(_sub(result, point), normal) < -distance:
            proj_lines: list[HalfPlane] = []
            for j in range(i):
                p_j, n_j = half_planes[j]
                d_j = _direction_of(n_j)
                denominator = _det(direction, d_j)
                if abs(denominator) <= _EPS:
                    if _dot(direction, d_j) > 0.0:
                        continue
                    new_point = (0.5 * (point[0] + p_j[0]), 0.5 * (point[1] + p_j[1]))
                else:
                    t = _det(d_j, _sub(point, p_j)) / denominator
                    new_point = (point[0] + t * direction[0], point[1] + t * direction[1])
                new_dir = _unit((d_j[0] - direction[0], d_j[1] - direction[1]))
                proj_lines.append((new_point, _normal_of(new_dir)))
            candidate, fail_j = _lp2(proj_lines, normal, max_speed, True)
            # A failure here means even the direction-optimizing 1D sub-problem
            # is over-constrained -- RVO2 treats this as float noise around an
            # already-feasible point and keeps the prior `result` rather than
            # raising (this function's hot-path contract).
            if fail_j == len(proj_lines):
                result = candidate
            distance = -_dot(_sub(result, point), normal)
    return result


class VelocityObstaclePlanner(ObstacleLocalPlanner, ABC):
    """Template Method base for VO/RVO/ORCA: every tick, gather neighbor +
    static obstacles, compute a goal-seeking preferred velocity, and delegate
    the actual avoidance strategy to `_select_velocity`. The three algorithms
    differ only in that one method.
    """

    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._max_speed = params.get_float("max_speed")
        self._max_omega = params.get_float("max_omega")
        self._heading_gain = params.get_float("heading_gain")
        self._agent_radius = params.get_float("agent_radius")
        self._neighbor_dist = params.get_float("neighbor_dist")
        self._time_horizon = params.get_float("time_horizon")
        self._obstacle_radius = params.get_float("obstacle_radius")

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
        # ABC entry point: static-avoidance-only mode (no neighbors) so this
        # planner remains a drop-in ObstacleLocalPlanner for the single-robot
        # simulator; the multi-agent harness calls command_with_neighbors directly.
        return self.command_with_neighbors(space, state, task, (), dt, recorder)

    def command_with_neighbors(
        self,
        space: ObstacleQuery,
        state: RobotState,
        task: LocalTask,
        neighbors: Sequence[DynamicObstacle],
        dt: float,
        recorder: TraceRecorder | None = None,
    ) -> VelocityCommand:
        x, y, theta = state.pose
        statics = static_obstacles(space, (x, y), self._neighbor_dist, self._obstacle_radius)
        v_pref = preferred_velocity(state.pose, task.goal, self._max_speed)
        v_new, constraints = self._select_velocity(v_pref, tuple(neighbors), statics, state, dt)
        if recorder is not None:
            recorder.velocity_obstacle(
                (x, y, theta),
                constraints,
                data={
                    "pref_vx": v_pref[0],
                    "pref_vy": v_pref[1],
                    "new_vx": v_new[0],
                    "new_vy": v_new[1],
                },
            )
        return velocity_to_command(v_new, theta, self._max_omega, self._heading_gain)

    @abstractmethod
    def _select_velocity(
        self,
        v_pref: Point,
        neighbors: tuple[DynamicObstacle, ...],
        statics: tuple[DynamicObstacle, ...],
        state: RobotState,
        dt: float,
    ) -> tuple[Point, tuple[tuple[float, ...], ...]]:
        """Choose the next absolute velocity plus the trace constraints (cones
        or half-planes) that produced it. `neighbors` and `statics` are kept
        separate (rather than pre-merged) because ORCA needs a different time
        horizon for each (moving agents vs. static-cell discs); VO/RVO treat
        both uniformly and simply concatenate them."""
        ...
