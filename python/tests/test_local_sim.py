"""Closed-loop simulator behavior: termination modes, integration, trace contract."""

from __future__ import annotations

import io
import json
import math

import pytest
from conftest import grid_from, open_grid

from navigation.core.capabilities import Capability, ObstacleQuery
from navigation.core.params import ParamSet
from navigation.core.planner import LocalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import LocalTask, RobotState, VelocityCommand
from navigation.local_planning._geometry import wrap_to_pi
from navigation.local_planning.simulation import SimConfig, SimStatus, integrate_unicycle, simulate


def _script_params() -> ParamSet:
    # Script planners below declare no algorithm params of their own -- the
    # empty ParamSet just satisfies LocalPlanner's constructor contract.
    return ParamSet("script", "local_planning", {})


class _ScriptPlanner(LocalPlanner[ObstacleQuery]):
    """Shared base for the test-only fixture planners below.

    Not a mock: each subclass drives a real tick loop through `simulate()` to
    exercise the simulator's integration / termination / metric behavior, the
    thing this test module verifies.
    """

    def __init__(self) -> None:
        super().__init__(_script_params())

    def required_capabilities(self) -> set[Capability]:
        return {Capability.OBSTACLE_QUERY}


class _StraightPlanner(_ScriptPlanner):
    """Constant (v, omega) every tick -- the plain no-obstacle-awareness case."""

    def __init__(self, v: float = 1.0, omega: float = 0.0) -> None:
        super().__init__()
        self._v = v
        self._omega = omega

    @property
    def name(self) -> str:
        return "script_straight"

    def compute_command(
        self,
        space: ObstacleQuery,
        state: RobotState,
        task: LocalTask,
        dt: float,
        recorder: TraceRecorder | None = None,
    ) -> VelocityCommand:
        return VelocityCommand(self._v, self._omega)


class _ZeroPlanner(_ScriptPlanner):
    """Always issues the zero command -- drives the stall detector."""

    @property
    def name(self) -> str:
        return "script_zero"

    def compute_command(
        self,
        space: ObstacleQuery,
        state: RobotState,
        task: LocalTask,
        dt: float,
        recorder: TraceRecorder | None = None,
    ) -> VelocityCommand:
        return VelocityCommand(0.0, 0.0)


class _RampPlanner(_ScriptPlanner):
    """Drives forward for a fixed number of ticks, then stops.

    Its command depends on an internal tick counter, so a run only replays
    identically if `simulate()` actually calls `reset()` at the start of
    every episode -- this is what the reset-contract test below checks.
    """

    def __init__(self, forward_ticks: int, v: float = 1.0) -> None:
        super().__init__()
        self._forward_ticks = forward_ticks
        self._v = v
        self._tick = 0

    @property
    def name(self) -> str:
        return "script_ramp"

    def reset(self) -> None:
        self._tick = 0

    def compute_command(
        self,
        space: ObstacleQuery,
        state: RobotState,
        task: LocalTask,
        dt: float,
        recorder: TraceRecorder | None = None,
    ) -> VelocityCommand:
        v = self._v if self._tick < self._forward_ticks else 0.0
        self._tick += 1
        return VelocityCommand(v, 0.0)


# --- 1. straight-line reach -------------------------------------------------
def test_straight_line_reaches_goal_with_exact_time_to_goal() -> None:
    grid = open_grid(5, 5)
    planner = _StraightPlanner(v=1.0, omega=0.0)
    start = RobotState(pose=(0.5, 2.5, 0.0))
    task = LocalTask(goal=(4.5, 2.5, 0.0))
    config = SimConfig(
        control_dt=0.5, max_steps=20, goal_tolerance=0.3,
        footprint_radius=0.1, stall_window=100, stall_distance=0.0,
    )
    result = simulate(planner, grid, start, task, config)
    assert result.status is SimStatus.REACHED
    assert result.success is True
    assert result.steps == 8
    assert result.time_to_goal == pytest.approx(8 * 0.5)


# --- 2. collision ------------------------------------------------------------
def test_start_pose_already_colliding_is_immediate_collision() -> None:
    grid = grid_from(["###", "###", "###"])
    planner = _StraightPlanner()
    start = RobotState(pose=(1.5, 1.5, 0.0))
    task = LocalTask(goal=(1.5, 1.5, 0.0))
    config = SimConfig(
        control_dt=0.1, max_steps=10, goal_tolerance=0.1,
        footprint_radius=0.1, stall_window=5, stall_distance=0.01,
    )
    result = simulate(planner, grid, start, task, config)
    assert result.status is SimStatus.COLLISION
    assert result.success is False
    assert result.steps == 0
    assert result.trajectory == [start.pose]


def test_wall_ahead_collides_mid_loop() -> None:
    # Single obstacle at cell (2,2) -> world center (2.5, 2.5); the robot drives
    # straight into it along row 2 (also world y=2.5), a branch distinct from
    # the immediate-start-collision case above (different code path entirely).
    grid = grid_from(
        [
            ".....",
            ".....",
            "..#..",
            ".....",
            ".....",
        ]
    )
    planner = _StraightPlanner(v=1.0, omega=0.0)
    start = RobotState(pose=(0.5, 2.5, 0.0))
    task = LocalTask(goal=(4.5, 2.5, 0.0))
    config = SimConfig(
        control_dt=0.5, max_steps=10, goal_tolerance=0.05,
        footprint_radius=0.2, stall_window=100, stall_distance=0.0,
    )
    result = simulate(planner, grid, start, task, config)
    assert result.status is SimStatus.COLLISION
    assert result.success is False
    assert 0 < result.steps < 10


# --- 3. stall ------------------------------------------------------------
def test_zero_command_triggers_stall() -> None:
    grid = open_grid(5, 5)
    planner = _ZeroPlanner()
    start = RobotState(pose=(2.5, 2.5, 0.0))
    task = LocalTask(goal=(4.9, 4.9, 0.0))
    config = SimConfig(
        control_dt=0.1, max_steps=20, goal_tolerance=0.05,
        footprint_radius=0.1, stall_window=5, stall_distance=0.02,
    )
    result = simulate(planner, grid, start, task, config)
    assert result.status is SimStatus.STALLED
    assert result.success is False
    assert result.steps == 5
    assert result.distance_traveled == 0.0


# --- 4. timeout ------------------------------------------------------------
def test_unreachable_goal_times_out() -> None:
    grid = open_grid(5, 5)
    planner = _StraightPlanner(v=1.0, omega=0.0)
    start = RobotState(pose=(2.5, 2.5, 0.0))
    task = LocalTask(goal=(-100.0, -100.0, 0.0))  # behind the robot's forward heading
    config = SimConfig(
        control_dt=0.1, max_steps=6, goal_tolerance=0.05,
        footprint_radius=0.1, stall_window=100, stall_distance=0.0,
    )
    result = simulate(planner, grid, start, task, config)
    assert result.status is SimStatus.TIMEOUT
    assert result.success is False
    assert result.steps == 6


# --- 5. exact-arc integration ------------------------------------------------
def test_integrate_unicycle_constant_command_traces_a_circle() -> None:
    v, omega = 1.0, 0.5
    r = v / omega
    pose = (0.0, 0.0, 0.0)
    cmd = VelocityCommand(v, omega)
    dt = 0.05
    # (x - r*sin(theta), y + r*cos(theta)) is the invariant circle center for a
    # constant-(v, omega) unicycle arc; every subsequent pose must sit at
    # radius r from it if the closed-form integration is exact.
    center = (pose[0] - r * math.sin(pose[2]), pose[1] + r * math.cos(pose[2]))
    for _ in range(40):
        pose = integrate_unicycle(pose, cmd, dt)
        radius = math.hypot(pose[0] - center[0], pose[1] - center[1])
        assert radius == pytest.approx(r, abs=1e-9)


# --- wrap_to_pi: both normalization branches (integrate_unicycle's turns stay
# small in the tests above, so this covers the negative-angle wraparound too) --
def test_wrap_to_pi_normalizes_both_directions() -> None:
    assert wrap_to_pi(0.5) == pytest.approx(0.5)
    assert wrap_to_pi(math.pi) == pytest.approx(math.pi)
    assert wrap_to_pi(-math.pi) == pytest.approx(math.pi)  # (-pi, pi]: -pi wraps to +pi
    assert wrap_to_pi(3.0 * math.pi) == pytest.approx(math.pi)
    assert wrap_to_pi(-3.0 * math.pi) == pytest.approx(math.pi)


# --- 6. min_clearance / distance_traveled hand-check -------------------------
def test_min_clearance_and_distance_traveled_match_hand_computation() -> None:
    grid = open_grid(5, 5)
    planner = _StraightPlanner(v=1.0, omega=0.0)
    start = RobotState(pose=(0.5, 2.5, 0.0))
    task = LocalTask(goal=(100.0, 100.0, 0.0))  # far enough that REACHED never fires
    config = SimConfig(
        control_dt=0.2, max_steps=5, goal_tolerance=0.01,
        footprint_radius=0.1, stall_window=100, stall_distance=0.0,
    )
    result = simulate(planner, grid, start, task, config)
    assert result.status is SimStatus.TIMEOUT

    xs = [0.5 + 0.2 * i for i in range(6)]  # start + 5 ticks at v*dt = 0.2 each
    expected_clearance = min(grid.distance_to_nearest((x, 2.5)) for x in xs)
    assert result.min_clearance == pytest.approx(expected_clearance)
    assert result.distance_traveled == pytest.approx(0.2 * 5)


# --- 7. trace contract + zero-cost None recorder ------------------------------
def test_recorder_emits_events_and_none_recorder_is_zero_cost() -> None:
    grid = open_grid(5, 5)
    planner = _StraightPlanner(v=1.0, omega=0.0)
    start = RobotState(pose=(0.5, 2.5, 0.0))
    task = LocalTask(goal=(1.5, 2.5, 0.0))
    config = SimConfig(
        control_dt=0.5, max_steps=10, goal_tolerance=0.1,
        footprint_radius=0.1, stall_window=100, stall_distance=0.0,
    )

    buf = io.StringIO()
    recorder = TraceRecorder(buf)
    result_with = simulate(planner, grid, start, task, config, recorder)
    events = [json.loads(line)["event"] for line in buf.getvalue().splitlines()]
    assert result_with.status is SimStatus.REACHED
    assert events.count("robot_moved") == result_with.steps
    assert events.count("path_found") == 1
    assert events.count("planning_finished") == 1

    result_without = simulate(planner, grid, start, task, config, None)
    assert result_without.status == result_with.status
    assert result_without.steps == result_with.steps
    assert result_without.distance_traveled == pytest.approx(result_with.distance_traveled)


# --- 8. reset contract ------------------------------------------------------
def test_reset_makes_reruns_deterministic() -> None:
    grid = open_grid(5, 5)
    planner = _RampPlanner(forward_ticks=3, v=1.0)
    start = RobotState(pose=(0.5, 2.5, 0.0))
    task = LocalTask(goal=(100.0, 100.0, 0.0))  # unreachable -> outcome is script-driven
    config = SimConfig(
        control_dt=0.2, max_steps=10, goal_tolerance=0.01,
        footprint_radius=0.1, stall_window=3, stall_distance=0.01,
    )

    first = simulate(planner, grid, start, task, config)
    second = simulate(planner, grid, start, task, config)

    assert second.status == first.status
    assert second.steps == first.steps
    assert second.distance_traveled == pytest.approx(first.distance_traveled)
