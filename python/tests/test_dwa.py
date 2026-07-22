"""DWA (Fox, Burgard & Thrun 1997): samples the accel-limited (v, omega) window,
rolls each candidate forward as a constant-command arc, and picks the
highest-scoring one that can still stop before the nearest obstacle -- via the
closed-loop simulator on real maps/scenarios (clutter weaving, a dead-end
U-trap) and via direct compute_command calls for the admissibility/window
bounds a full episode can't isolate."""

from __future__ import annotations

import io
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml
from conftest import REPO_ROOT

from navigation.core.params import ParamError, ParamSet
from navigation.core.trace import TraceRecorder
from navigation.core.types import LocalTask, RobotState
from navigation.local_planning.reactive.dwa import Dwa
from navigation.local_planning.simulation import SimConfig, SimStatus, simulate
from navigation.maps.loader import load_map, load_scenario
from navigation.maps.occupancy_grid import OccupancyGrid2D

_ALGO = "dwa"
_CONFIG_PATH = REPO_ROOT / "configs" / "local_planning" / f"{_ALGO}.yaml"
_MAPS_DIR = REPO_ROOT / "maps"


def _config(tmp_path: Path, **overrides: Any) -> ParamSet:
    """Real config with selected defaults overridden."""
    doc = yaml.safe_load(_CONFIG_PATH.read_text())
    for entry in doc["params"]:
        if entry["name"] in overrides:
            entry["default"] = overrides[entry["name"]]
    out = tmp_path / f"{_ALGO}.yaml"
    out.write_text(yaml.safe_dump(doc), encoding="utf-8")
    return ParamSet.from_yaml(out)


def _sim_config(params: ParamSet) -> SimConfig:
    return SimConfig(
        control_dt=params.get_float("control_dt"),
        max_steps=params.get_int("max_steps"),
        goal_tolerance=params.get_float("goal_tolerance"),
        footprint_radius=params.get_float("footprint_radius"),
        stall_window=params.get_int("stall_window"),
        stall_distance=params.get_float("stall_distance"),
    )


def _run(map_name: str, scenario_name: str, params: ParamSet) -> Any:
    grid = load_map(_MAPS_DIR / "grid" / f"{map_name}.yaml")
    scenario = load_scenario(_MAPS_DIR / "scenarios" / f"{scenario_name}.yaml")
    planner = Dwa(params)
    start = RobotState(pose=(scenario.start[0], scenario.start[1], scenario.start_theta))
    task = LocalTask(goal=(scenario.goal[0], scenario.goal[1], scenario.goal_theta))
    return simulate(planner, grid, start, task, _sim_config(params))


# --- (a) weaves the clutter map's obstacles to the goal without colliding ------
def test_reaches_goal_on_clutter_map_weaving_obstacles(tmp_path: Path) -> None:
    params = _config(tmp_path)
    result = _run("clutter01", "clutter01_s1", params)
    assert result.status is SimStatus.REACHED
    assert result.success is True
    assert result.min_clearance > 0.0


# --- (b) U-trap dead end: admissibility forces a stop, not a wall graze --------
def test_dead_end_trap_stalls_without_colliding(tmp_path: Path) -> None:
    params = _config(tmp_path)
    result = _run("pf_trap01", "pf_trap01_s1", params)
    assert result.status is SimStatus.STALLED
    assert result.success is False


# --- (c) v_samples below the declared minimum fails load-time validation -------
def test_v_samples_below_min_rejected_at_load_time(tmp_path: Path) -> None:
    doc = yaml.safe_load(_CONFIG_PATH.read_text())
    for entry in doc["params"]:
        if entry["name"] == "v_samples":
            entry["default"] = 0
    bad = tmp_path / f"{_ALGO}.yaml"
    bad.write_text(yaml.safe_dump(doc), encoding="utf-8")
    with pytest.raises(ParamError):
        ParamSet.from_yaml(bad)


# --- (d) admissibility: a close head-on wall forces a slower, bounded command --
def test_admissible_bound_forces_deceleration_facing_a_close_wall(tmp_path: Path) -> None:
    # Fine resolution (0.02 m) keeps the EDT-based clearance estimate close to
    # the true continuous distance, so a hand-picked gap reliably separates
    # "max-speed candidates rejected" from "some slower candidate admissible" --
    # a coarser grid's cell-center quantization would make this razor-thin.
    params = _config(tmp_path)
    resolution = 0.02
    pixels = np.full((300, 3000), 255, dtype=np.uint16)
    x0, y0 = 5.0, 3.0
    gap = 1.7  # wall close enough that a full-speed approach is inadmissible
    wall_col = int(round((x0 + gap) / resolution - 0.5))
    pixels[:, wall_col:] = 0
    grid = OccupancyGrid2D(pixels=pixels, resolution=resolution, origin=(0.0, 0.0, 0.0))

    planner = Dwa(params)
    v_a = params.get_float("max_speed")
    state = RobotState(pose=(x0, y0, 0.0), v=v_a, omega=0.0)
    task = LocalTask(goal=(x0 + 100.0, y0, 0.0))  # straight ahead, past the wall

    buf = io.StringIO()
    recorder = TraceRecorder(buf)
    cmd = planner.compute_command(grid, state, task, params.get_float("control_dt"), recorder)

    assert cmd.v < v_a
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    candidates = [e for e in events if e["event"] == "candidate_evaluated"]
    selected = [e for e in candidates if e["data"]["selected"] == 1.0]
    assert len(selected) == 1
    data = selected[0]["data"]
    assert data["admissible"] == 1.0
    assert cmd.v == pytest.approx(data["v"])
    bound = math.sqrt(2.0 * data["clearance"] * params.get_float("accel"))
    assert cmd.v <= bound + 1e-9


# --- (d2) no admissible candidate at all: maximum-braking fallback -------------
def test_all_candidates_colliding_falls_back_to_maximum_braking(tmp_path: Path) -> None:
    # Wall so close (gap 0.25 m vs footprint 0.2 m) that every candidate's very
    # first rollout pose already collides: the window's v floor is positive at
    # this speed, so no candidate can stand still and none survives scoring.
    params = _config(tmp_path)
    resolution = 0.02
    pixels = np.full((300, 300), 255, dtype=np.uint16)
    x0, y0 = 3.0, 3.0
    wall_col = int(round((x0 + 0.25) / resolution - 0.5))
    pixels[:, wall_col:] = 0
    grid = OccupancyGrid2D(pixels=pixels, resolution=resolution, origin=(0.0, 0.0, 0.0))

    planner = Dwa(params)
    dt = params.get_float("control_dt")
    v_a = params.get_float("max_speed")
    omega_a = 1.0
    state = RobotState(pose=(x0, y0, 0.0), v=v_a, omega=omega_a)
    task = LocalTask(goal=(x0 + 100.0, y0, 0.0))

    buf = io.StringIO()
    recorder = TraceRecorder(buf)
    cmd = planner.compute_command(grid, state, task, dt, recorder)

    # Braking at the kinematic limits: v drops by exactly one tick's accel
    # budget and omega decays toward zero without flipping sign.
    assert cmd.v == pytest.approx(v_a - params.get_float("accel") * dt)
    expected_omega = omega_a - min(omega_a, params.get_float("accel_omega") * dt)
    assert cmd.omega == pytest.approx(expected_omega)
    assert 0.0 <= cmd.omega < omega_a
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    candidates = [e for e in events if e["event"] == "candidate_evaluated"]
    assert candidates and all(e["data"]["admissible"] == 0.0 for e in candidates)
    assert all(e["data"]["selected"] == 0.0 for e in candidates)


# --- (e) dynamic window: stopped robot cannot exceed one tick's accel budget ---
def test_dynamic_window_caps_acceleration_from_a_stop(tmp_path: Path) -> None:
    params = _config(tmp_path)
    pixels = np.full((200, 200), 255, dtype=np.uint16)
    grid = OccupancyGrid2D(pixels=pixels, resolution=0.1, origin=(0.0, 0.0, 0.0))
    planner = Dwa(params)
    state = RobotState(pose=(5.0, 5.0, 0.0), v=0.0, omega=0.0)
    task = LocalTask(goal=(50.0, 5.0, 0.0))

    cmd = planner.compute_command(grid, state, task, params.get_float("control_dt"), None)

    assert cmd.v <= params.get_float("accel") * params.get_float("control_dt") + 1e-9


# --- (f) reset/determinism: identical rerun produces the identical trajectory --
def test_rerun_after_implicit_reset_is_deterministic(tmp_path: Path) -> None:
    # simulate() calls planner.reset() itself at the top of every episode; Dwa
    # carries no cursor to leak, but the deterministic (v outer, omega inner)
    # sampling grid must still reproduce bit-identical candidate selection.
    params = _config(tmp_path)
    grid = load_map(_MAPS_DIR / "grid" / "clutter01.yaml")
    scenario = load_scenario(_MAPS_DIR / "scenarios" / "clutter01_s1.yaml")
    planner = Dwa(params)
    start = RobotState(pose=(scenario.start[0], scenario.start[1], scenario.start_theta))
    task = LocalTask(goal=(scenario.goal[0], scenario.goal[1], scenario.goal_theta))
    config = _sim_config(params)

    first = simulate(planner, grid, start, task, config)
    second = simulate(planner, grid, start, task, config)

    assert second.status == first.status
    assert second.steps == first.steps
    assert second.distance_traveled == pytest.approx(first.distance_traveled)
    assert len(second.trajectory) == len(first.trajectory)
    for pose_a, pose_b in zip(first.trajectory, second.trajectory, strict=True):
        assert pose_a == pytest.approx(pose_b)


# --- (g) trace: candidates carry a rollout, and selected == best admissible ----
def test_recorder_emits_rollouts_and_selects_the_best_admissible_candidate(
    tmp_path: Path,
) -> None:
    params = _config(tmp_path)
    grid = load_map(_MAPS_DIR / "grid" / "clutter01.yaml")
    scenario = load_scenario(_MAPS_DIR / "scenarios" / "clutter01_s1.yaml")
    planner = Dwa(params)
    state = RobotState(pose=(scenario.start[0], scenario.start[1], scenario.start_theta))
    task = LocalTask(goal=(scenario.goal[0], scenario.goal[1], scenario.goal_theta))

    buf = io.StringIO()
    recorder = TraceRecorder(buf)
    planner.compute_command(grid, state, task, params.get_float("control_dt"), recorder)

    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    candidates = [e for e in events if e["event"] == "candidate_evaluated"]
    expected = params.get_int("v_samples") * params.get_int("omega_samples")
    assert len(candidates) == expected
    assert all(len(e["rollout"]) == params.get_int("sim_steps") for e in candidates)

    admissible = [e for e in candidates if e["data"]["admissible"] == 1.0]
    selected = [e for e in candidates if e["data"]["selected"] == 1.0]
    assert admissible  # this tick has at least one safe candidate
    assert len(selected) == 1
    assert selected[0]["cost"] == max(e["cost"] for e in admissible)
