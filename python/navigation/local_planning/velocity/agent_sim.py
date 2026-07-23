"""Multi-agent closed-loop harness for the velocity-obstacle family.

N bodies share one map; each is either driven by its own VelocityObstaclePlanner
or moves at a fixed scripted velocity (a non-cooperative mover for single-planner
demos). Every tick is read-all-then-write: every body's command is computed
against the SAME pre-tick snapshot of every other body, then all bodies
integrate together -- order-independent and reproducible, which reciprocal
avoidance (van den Berg et al. 2008) depends on (a sequential update would make
agent 0 react to agent 1's *old* position while agent 1 reacts to agent 0's
*new* one, breaking the reciprocity the algorithm assumes).

Reuses `local_planning/simulation.py`'s `integrate_unicycle`/`SimConfig`/
`SimStatus` (family -> category-shared root, the same direction `_geometry.py`
is imported from) rather than re-deriving single-body integration or status
enums here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from navigation.core.capabilities import ObstacleQuery
from navigation.core.trace import TraceRecorder
from navigation.core.types import Footprint, LocalTask, Point, Pose, RobotState, VelocityCommand
from navigation.local_planning.simulation import SimConfig, SimStatus, integrate_unicycle

from ._velocity_obstacle import DynamicObstacle, VelocityObstaclePlanner


@dataclass(frozen=True)
class AgentSpec:
    start: RobotState
    goal: Pose
    radius: float
    # None -> driven by the matching planner; a value -> a non-cooperative
    # constant-velocity mover (the VO/RVO/ORCA demos' scripted crossing traffic).
    scripted_velocity: Point | None = None


@dataclass
class AgentResult:
    status: SimStatus
    steps: int
    trajectory: list[Pose] = field(default_factory=list)
    min_pair_clearance: float = math.inf


def _xy_dist(a: tuple[float, float] | Pose, b: tuple[float, float] | Pose) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _pairwise_min_clearance(states: list[RobotState], specs: list[AgentSpec]) -> float:
    best = math.inf
    n = len(states)
    for i in range(n):
        for j in range(i + 1, n):
            gap = _xy_dist(states[i].pose, states[j].pose) - specs[i].radius - specs[j].radius
            if gap < best:
                best = gap
    return best


def simulate_agents(
    planners: list[VelocityObstaclePlanner | None],
    specs: list[AgentSpec],
    space: ObstacleQuery,
    config: SimConfig,
    recorder: TraceRecorder | None = None,
) -> list[AgentResult]:
    """Run every agent in closed loop until every planner-driven agent reaches
    its goal, a collision (pairwise or static) occurs, the group stalls, or
    the step budget runs out. `planners[k] is None` iff `specs[k]` is scripted.

    Reached agents are not special-cased out of planning/integration: once an
    agent is near its own goal, `preferred_velocity` alone drives it toward
    (0, 0), so it settles in place naturally and keeps contributing an
    (approximately stationary) DynamicObstacle snapshot to its neighbors.
    """
    n = len(specs)
    footprint = Footprint(config.footprint_radius)
    states = [spec.start for spec in specs]
    world_vel: list[Point] = [(0.0, 0.0) for _ in range(n)]
    trajectories: list[list[Pose]] = [[spec.start.pose] for spec in specs]
    reached: list[bool] = [False] * n
    planner_indices = [k for k in range(n) if specs[k].scripted_velocity is None]

    min_pair_clearance = _pairwise_min_clearance(states, specs)
    terminal = SimStatus.TIMEOUT
    steps = config.max_steps

    for step in range(1, config.max_steps + 1):
        # 1) snapshot every body before this tick's motion.
        snapshot = [
            DynamicObstacle(
                position=(states[k].pose[0], states[k].pose[1]),
                velocity=world_vel[k],
                radius=specs[k].radius,
            )
            for k in range(n)
        ]
        # 2) compute every command against that fixed snapshot, index order.
        commands: list[VelocityCommand | None] = [None] * n
        for k in planner_indices:
            neighbors = tuple(snapshot[j] for j in range(n) if j != k)
            task = LocalTask(goal=specs[k].goal)
            planner = planners[k]
            assert planner is not None, f"agent {k} has no planner but is not scripted"
            rec = recorder if k == 0 else None
            commands[k] = planner.command_with_neighbors(
                space, states[k], task, neighbors, config.control_dt, rec
            )
        # 3) integrate every body only after every command is known.
        new_states = list(states)
        for k in range(n):
            scripted = specs[k].scripted_velocity
            if scripted is not None:
                vx, vy = scripted
                x, y, _theta = states[k].pose
                new_pose = (
                    x + vx * config.control_dt,
                    y + vy * config.control_dt,
                    math.atan2(vy, vx),
                )
                world_vel[k] = (vx, vy)
                new_states[k] = RobotState(new_pose, math.hypot(vx, vy), 0.0)
            else:
                cmd = commands[k]
                assert cmd is not None
                new_pose = integrate_unicycle(states[k].pose, cmd, config.control_dt)
                world_vel[k] = (cmd.v * math.cos(new_pose[2]), cmd.v * math.sin(new_pose[2]))
                new_states[k] = RobotState(new_pose, cmd.v, cmd.omega)
        states = new_states
        # 4) trace + trajectory bookkeeping for every body.
        for k in range(n):
            trajectories[k].append(states[k].pose)
            if recorder is not None:
                recorder.robot_moved(
                    states[k].pose, data={"v": states[k].v, "omega": states[k].omega}, agent=k
                )
        # 5) termination judgement.
        tick_clearance = _pairwise_min_clearance(states, specs)
        min_pair_clearance = min(min_pair_clearance, tick_clearance)
        collided = tick_clearance < 0.0 or any(
            space.is_collision(footprint, states[k].pose) for k in range(n)
        )
        if collided:
            terminal, steps = SimStatus.COLLISION, step
            break
        for k in planner_indices:
            if not reached[k] and _xy_dist(states[k].pose, specs[k].goal) <= config.goal_tolerance:
                reached[k] = True
        if all(reached[k] for k in planner_indices):
            terminal, steps = SimStatus.REACHED, step
            break
        still_active = [k for k in planner_indices if not reached[k]]
        if (
            still_active
            and step >= config.stall_window
            and all(
                _xy_dist(states[k].pose, trajectories[k][step - config.stall_window])
                < config.stall_distance
                for k in still_active
            )
        ):
            terminal, steps = SimStatus.STALLED, step
            break
        steps = step

    return [
        AgentResult(
            status=SimStatus.REACHED if reached[k] else terminal,
            steps=steps,
            trajectory=list(trajectories[k]),
            min_pair_clearance=min_pair_clearance,
        )
        for k in range(n)
    ]
