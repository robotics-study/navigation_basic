"""Regulated Pure Pursuit (Macenski et al. 2023): the plain lookahead-arc law
plus three regulations -- adaptive lookahead, curvature-proportional slowdown,
proximity-proportional slowdown -- and a predictive lookahead collision stop.
Each regulation is exercised directly via `compute_command` on a small
synthetic grid so the trigger condition is unambiguous, mirroring
`test_vfh.py`'s style; end-to-end tracking is covered separately on the
`clutter01_s2` scenario."""

from __future__ import annotations

import io
import json
from pathlib import Path

import numpy as np
import pytest
import yaml
from conftest import REPO_ROOT, open_grid

from navigation.core.params import ParamError, ParamSet
from navigation.core.trace import TraceRecorder
from navigation.core.types import LocalTask, RobotState
from navigation.local_planning.simulation import SimConfig, SimStatus, simulate
from navigation.local_planning.tracking.regulated_pure_pursuit import RegulatedPurePursuit
from navigation.maps.loader import load_map, load_scenario
from navigation.maps.occupancy_grid import OccupancyGrid2D

_ALGO = "regulated_pure_pursuit"
_LOCAL_CONFIG_DIR = REPO_ROOT / "configs" / "local_planning"
_MAP = REPO_ROOT / "maps" / "grid" / "clutter01.yaml"
_SCENARIO = REPO_ROOT / "maps" / "scenarios" / "clutter01_s2.yaml"


def _real_params() -> ParamSet:
    return ParamSet.from_yaml(_LOCAL_CONFIG_DIR / f"{_ALGO}.yaml")


def _real_setup() -> tuple[ParamSet, RobotState, LocalTask, SimConfig]:
    params = _real_params()
    scenario = load_scenario(_SCENARIO)
    start = RobotState(pose=(scenario.start[0], scenario.start[1], scenario.start_theta))
    task = LocalTask(
        goal=(scenario.goal[0], scenario.goal[1], scenario.goal_theta),
        reference_path=scenario.reference_path,
    )
    config = SimConfig(
        control_dt=params.get_float("control_dt"),
        max_steps=params.get_int("max_steps"),
        goal_tolerance=params.get_float("goal_tolerance"),
        footprint_radius=params.get_float("footprint_radius"),
        stall_window=params.get_int("stall_window"),
        stall_distance=params.get_float("stall_distance"),
    )
    return params, start, task, config


def _grid_from_resolution(rows: list[str], resolution: float) -> OccupancyGrid2D:
    # conftest.grid_from hardcodes resolution=1.0, too coarse to place an
    # obstacle at a fractional-meter clearance (distance_to_nearest snaps to
    # the probe's own cell, so distances only ever land on whole multiples of
    # the resolution) -- built directly here for control over that quantum,
    # same pattern as test_vfh.py's fully-enclosed test.
    pixels = np.array([[255 if ch == "." else 0 for ch in row] for row in rows], dtype=np.uint16)
    return OccupancyGrid2D(pixels=pixels, resolution=resolution, origin=(0.0, 0.0, 0.0))


def _last_candidate_data(recorder_buf: io.StringIO) -> dict[str, float]:
    events = [json.loads(line) for line in recorder_buf.getvalue().splitlines()]
    candidates = [e for e in events if e["event"] == "candidate_evaluated"]
    assert len(candidates) == 1
    data: dict[str, float] = candidates[0]["data"]
    return data


# --- (a) clutter01_s2: reaches the goal without ever colliding -------------------
def test_reaches_goal_on_clutter_map_without_collision() -> None:
    params, start, task, config = _real_setup()
    grid = load_map(_MAP)
    planner = RegulatedPurePursuit(params)

    result = simulate(planner, grid, start, task, config)

    assert result.status is SimStatus.REACHED
    assert result.success is True
    assert result.min_clearance > 0.0


# --- (b) curvature regulation: tight lookahead turn caps v below v_goal ----------
def test_tight_turn_triggers_curvature_regulation() -> None:
    # Path runs due north from the robot while the robot faces east: the
    # lookahead target sits 90 degrees off-heading at L_d=0.5, giving a
    # commanded turn radius of 0.25 m -- well under regulated_min_radius
    # (0.9 m default) so curvature regulation must engage.
    params = _real_params()
    grid = open_grid(40, 40)
    planner = RegulatedPurePursuit(params)
    state = RobotState(pose=(20.0, 20.0, 0.0), v=0.5, omega=0.0)
    task = LocalTask(goal=(20.0, 100.0, 0.0), reference_path=((20.0, 20.0), (20.0, 23.0)))

    buf = io.StringIO()
    cmd = planner.compute_command(grid, state, task, 0.1, TraceRecorder(buf))

    v_goal = params.get_float("max_speed")  # remaining >> slow_radius, so v_goal == max_speed
    data = _last_candidate_data(buf)
    assert cmd.v < v_goal
    assert data["curvature_scale"] < 1.0
    assert data["blocked"] == 0.0


# --- (c) proximity regulation: nearby obstacle caps v below v_goal ---------------
def test_nearby_obstacle_triggers_proximity_regulation() -> None:
    # Obstacle cell sits exactly one grid cell (0.5 m at this resolution)
    # north of the robot's cell; distance_to_nearest quantizes to the probe's
    # own cell so this reads as clearance 0.5-0.2=0.3 m, under
    # proximity_distance (0.6 m default), while the physical footprint disc
    # (radius 0.2 m) stays 0.25 m clear of the obstacle cell's edge -- close
    # enough to regulate, not close enough to collide. Path points straight
    # east (alpha=0) so curvature regulation stays inactive, isolating the
    # proximity effect.
    params = _real_params()
    grid = _grid_from_resolution(
        [".......", ".......", ".......", "..#....", ".......", ".......", "......."],
        resolution=0.5,
    )
    planner = RegulatedPurePursuit(params)
    state = RobotState(pose=(1.25, 1.25, 0.0), v=0.5, omega=0.0)
    task = LocalTask(goal=(100.0, 1.25, 0.0), reference_path=((1.25, 1.25), (4.25, 1.25)))

    buf = io.StringIO()
    cmd = planner.compute_command(grid, state, task, 0.1, TraceRecorder(buf))

    v_goal = params.get_float("max_speed")
    data = _last_candidate_data(buf)
    assert cmd.v < v_goal
    assert data["proximity_scale"] < 1.0
    assert data["curvature_scale"] == 1.0
    assert data["blocked"] == 0.0


# --- (d) blocked: an obstacle sitting on the command arc forces a hard stop ------
def test_obstacle_on_command_arc_forces_stop() -> None:
    # Obstacle cell one meter due east, robot speed set so L_d (== lookahead
    # collision-check arc length here, since the target is straight ahead)
    # reaches exactly onto it.
    params = _real_params()
    grid = _grid_from_resolution(
        [".....", ".....", "...#.", ".....", "....."], resolution=1.0
    )
    planner = RegulatedPurePursuit(params)
    state = RobotState(pose=(2.5, 2.5, 0.0), v=1.0, omega=0.0)
    task = LocalTask(goal=(100.0, 2.5, 0.0), reference_path=((2.5, 2.5), (4.5, 2.5)))

    buf = io.StringIO()
    cmd = planner.compute_command(grid, state, task, 0.1, TraceRecorder(buf))

    data = _last_candidate_data(buf)
    assert cmd.v == 0.0
    assert cmd.omega == 0.0
    assert data["blocked"] == 1.0


# --- (e) adaptive lookahead follows state.v, clamped to [min, max] ---------------
def test_lookahead_scales_with_speed_within_clamp() -> None:
    params = _real_params()
    grid = open_grid(40, 40)
    task = LocalTask(goal=(100.0, 20.0, 0.0), reference_path=((20.0, 20.0), (30.0, 20.0)))
    min_lookahead = params.get_float("min_lookahead")
    max_lookahead = params.get_float("max_lookahead")
    lookahead_time = params.get_float("lookahead_time")

    # v=0 (stopped) clamps to the floor; a very high v clamps to the ceiling;
    # a moderate v in between reproduces lookahead_time*v exactly.
    for v, expected in (
        (0.0, min_lookahead),
        (5.0, max_lookahead),
        (0.5, lookahead_time * 0.5),
    ):
        planner = RegulatedPurePursuit(params)
        state = RobotState(pose=(20.0, 20.0, 0.0), v=v, omega=0.0)
        buf = io.StringIO()
        planner.compute_command(grid, state, task, 0.1, TraceRecorder(buf))
        assert _last_candidate_data(buf)["lookahead"] == pytest.approx(expected)


# --- (f) lookahead_time = 0 fails load-time range validation --------------------
def test_zero_lookahead_time_rejected_at_load_time(tmp_path: Path) -> None:
    doc = yaml.safe_load((_LOCAL_CONFIG_DIR / f"{_ALGO}.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] == "lookahead_time":
            entry["default"] = 0.0
    bad = tmp_path / f"{_ALGO}.yaml"
    bad.write_text(yaml.safe_dump(doc), encoding="utf-8")
    with pytest.raises(ParamError):
        ParamSet.from_yaml(bad)


# --- (behavior) reset() leaves no _progress_index cursor leak across reruns ------
def test_rerun_after_implicit_reset_is_deterministic() -> None:
    # simulate() calls planner.reset() itself at the top of every episode --
    # this proves that call actually clears _progress_index rather than the
    # planner silently resuming from wherever the first run's cursor stopped.
    _, start, task, config = _real_setup()
    grid = load_map(_MAP)
    planner = RegulatedPurePursuit(_real_params())

    first = simulate(planner, grid, start, task, config)
    second = simulate(planner, grid, start, task, config)

    assert second.status == first.status
    assert second.steps == first.steps
    assert second.distance_traveled == pytest.approx(first.distance_traveled)
    assert len(second.trajectory) == len(first.trajectory)
    for pose_a, pose_b in zip(first.trajectory, second.trajectory, strict=True):
        assert pose_a == pytest.approx(pose_b)
