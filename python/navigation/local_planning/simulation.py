"""Closed-loop simulator: the execution harness every local planner runs under.

Depends on `core` abstractions only (LocalPlanner, ObstacleQuery, types, trace) --
no algorithm module import, no concrete map class. Shared by demos, tests, and
bench so the tick order (and therefore trace event stream) is identical no
matter which caller drives a planner.
"""

from __future__ import annotations

import enum
import math
import time
from dataclasses import dataclass, field

from navigation.core.capabilities import ObstacleQuery
from navigation.core.planner import LocalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import Footprint, LocalTask, Pose, RobotState, VelocityCommand

from ._geometry import wrap_to_pi

# Below this angular rate the closed-form arc (division by omega) is numerically
# unstable, so integration falls back to the straight-line limit instead --
# not a tolerance the caller can tune, so it is not a config parameter.
_OMEGA_EPS = 1e-9


class SimStatus(enum.Enum):
    REACHED = "reached"
    COLLISION = "collision"
    # Net displacement over a sliding window fell below tolerance: a distinct
    # failure mode from TIMEOUT (e.g. Potential Fields local-minimum lock-up),
    # worth reporting separately rather than lumping into "ran out of steps".
    STALLED = "stalled"
    TIMEOUT = "timeout"


@dataclass(frozen=True)
class SimConfig:
    control_dt: float
    max_steps: int
    goal_tolerance: float
    footprint_radius: float
    stall_window: int
    stall_distance: float


@dataclass
class SimResult:
    status: SimStatus
    success: bool
    steps: int
    time_to_goal: float  # steps * control_dt
    distance_traveled: float
    min_clearance: float
    trajectory: list[Pose] = field(default_factory=list)  # includes the start pose


def integrate_unicycle(pose: Pose, cmd: VelocityCommand, dt: float) -> Pose:
    """Exact constant-(v, omega) unicycle arc over one tick (closed form).

    A constant-velocity-and-turn-rate segment has a closed-form circular-arc
    solution, so this is exact for any dt -- Euler discretization error never
    mixes into the tick loop, keeping observed behavior (PF oscillation, PP
    tracking error) attributable to the algorithm rather than the integrator.
    """
    x, y, theta = pose
    v, omega = cmd.v, cmd.omega
    if abs(omega) < _OMEGA_EPS:
        return (x + v * dt * math.cos(theta), y + v * dt * math.sin(theta), theta)
    new_theta = theta + omega * dt
    x2 = x + (v / omega) * (math.sin(new_theta) - math.sin(theta))
    y2 = y - (v / omega) * (math.cos(new_theta) - math.cos(theta))
    return (x2, y2, wrap_to_pi(new_theta))


def _xy_dist(a: Pose, b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _finish(
    recorder: TraceRecorder | None,
    status: SimStatus,
    steps: int,
    control_dt: float,
    distance_traveled: float,
    min_clearance: float,
    trajectory: list[Pose],
    t_start: float,
) -> SimResult:
    success = status is SimStatus.REACHED
    time_to_goal = steps * control_dt
    metrics = {
        "time_to_goal": time_to_goal,
        "distance_traveled": distance_traveled,
        "min_clearance": min_clearance,
        "steps": float(steps),
        "runtime_sec": time.monotonic() - t_start,
        # additionalProperties on the trace `metrics` field is numeric-only, so
        # the outcome is carried as 0/1 flags instead of the SimStatus string.
        "collided": 1.0 if status is SimStatus.COLLISION else 0.0,
        "stalled": 1.0 if status is SimStatus.STALLED else 0.0,
    }
    if recorder is not None:
        if success:
            recorder.path_found(trajectory)
        recorder.planning_finished(success, metrics)
    return SimResult(
        status=status,
        success=success,
        steps=steps,
        time_to_goal=time_to_goal,
        distance_traveled=distance_traveled,
        min_clearance=min_clearance,
        trajectory=trajectory,
    )


def simulate(
    planner: LocalPlanner[ObstacleQuery],
    space: ObstacleQuery,
    start: RobotState,
    task: LocalTask,
    config: SimConfig,
    recorder: TraceRecorder | None = None,
) -> SimResult:
    """Run `planner` in closed loop against `space` from `start` until the
    episode ends (goal reached, collision, stall, or step budget exhausted)."""
    t_start = time.monotonic()
    planner.reset()
    footprint = Footprint(config.footprint_radius)
    min_clearance = space.distance_to_nearest((start.pose[0], start.pose[1]))
    trajectory: list[Pose] = [start.pose]
    if space.is_collision(footprint, start.pose):
        return _finish(
            recorder,
            SimStatus.COLLISION,
            0,
            config.control_dt,
            0.0,
            min_clearance,
            trajectory,
            t_start,
        )

    state = start
    distance_traveled = 0.0
    status = SimStatus.TIMEOUT
    steps = config.max_steps
    for step in range(1, config.max_steps + 1):
        cmd = planner.compute_command(space, state, task, config.control_dt, recorder)
        new_pose = integrate_unicycle(state.pose, cmd, config.control_dt)
        if recorder is not None:
            recorder.robot_moved(new_pose, data={"v": cmd.v, "omega": cmd.omega})
        if space.is_collision(footprint, new_pose):
            status = SimStatus.COLLISION
            steps = step
            break
        min_clearance = min(min_clearance, space.distance_to_nearest((new_pose[0], new_pose[1])))
        distance_traveled += _xy_dist(new_pose, (state.pose[0], state.pose[1]))
        trajectory.append(new_pose)
        if _xy_dist(new_pose, (task.goal[0], task.goal[1])) <= config.goal_tolerance:
            status = SimStatus.REACHED
            steps = step
            break
        if step >= config.stall_window:
            ref = trajectory[step - config.stall_window]
            if _xy_dist(new_pose, (ref[0], ref[1])) < config.stall_distance:
                status = SimStatus.STALLED
                steps = step
                break
        state = RobotState(new_pose, cmd.v, cmd.omega)

    return _finish(
        recorder,
        status,
        steps,
        config.control_dt,
        distance_traveled,
        min_clearance,
        trajectory,
        t_start,
    )
