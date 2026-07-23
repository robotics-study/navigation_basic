"""MPC (Klančar & Škrjanc 2007): fixed-iteration projected gradient descent on
the shared receding-horizon cost J(U), verified via the closed-loop simulator on
an open-field interior-goal scenario (goal-seeking around an obstacle) and via
direct compute_command calls for behavior a full episode can't isolate cleanly (a
single tick's descent lowering J, the emitted horizon length, the obstacle term
bending the predicted horizon away from an obstacle)."""

from __future__ import annotations

import io
import json
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml
from conftest import REPO_ROOT

from navigation.core.params import ParamError, ParamSet
from navigation.core.trace import TraceRecorder
from navigation.core.types import LocalTask, RobotState
from navigation.local_planning._geometry import nearest_occupied
from navigation.local_planning.predictive.mpc import MpcPlanner
from navigation.local_planning.simulation import SimConfig, SimStatus, simulate
from navigation.maps.loader import load_map, load_scenario
from navigation.maps.occupancy_grid import OccupancyGrid2D

_ALGO = "mpc"
_CONFIG_PATH = REPO_ROOT / "configs" / "local_planning" / f"{_ALGO}.yaml"
_MAPS_DIR = REPO_ROOT / "maps"
# Reach on an open-field interior goal: predictive planners push clearance inside
# min_obstacle_dist away, so a goal clear of the obstacle-penalty zone is required.
# A corner goal (clutter01_s1) sits inside the zone and is the documented weakness.
_MAP = "open01"
_SCENARIO = "open01_s4"


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


def _run(params: ParamSet, recorder: TraceRecorder | None = None) -> Any:
    grid = load_map(_MAPS_DIR / "grid" / f"{_MAP}.yaml")
    scenario = load_scenario(_MAPS_DIR / "scenarios" / f"{_SCENARIO}.yaml")
    planner = MpcPlanner(params)
    start = RobotState(pose=(scenario.start[0], scenario.start[1], scenario.start_theta))
    task = LocalTask(
        goal=(scenario.goal[0], scenario.goal[1], scenario.goal_theta),
        reference_path=scenario.reference_path,
    )
    return simulate(planner, grid, start, task, _sim_config(params), recorder)


def _band_events(buf: io.StringIO) -> list[dict[str, Any]]:
    events = (json.loads(line) for line in buf.getvalue().splitlines())
    return [e for e in events if e["event"] == "band_updated"]


def _single_tick_band(params: ParamSet) -> dict[str, Any]:
    """One cold-start compute_command, returning its single band_updated event."""
    grid = load_map(_MAPS_DIR / "grid" / f"{_MAP}.yaml")
    scenario = load_scenario(_MAPS_DIR / "scenarios" / f"{_SCENARIO}.yaml")
    planner = MpcPlanner(params)
    state = RobotState(pose=(scenario.start[0], scenario.start[1], scenario.start_theta))
    task = LocalTask(
        goal=(scenario.goal[0], scenario.goal[1], scenario.goal_theta),
        reference_path=scenario.reference_path,
    )
    buf = io.StringIO()
    recorder = TraceRecorder(buf)
    planner.compute_command(grid, state, task, params.get_float("control_dt"), recorder)
    bands = _band_events(buf)
    assert len(bands) == 1
    return bands[0]


# --- (a) closed-loop MPC reaches an open-field interior goal ------------------
def test_reaches_goal_open_field(tmp_path: Path) -> None:
    result = _run(_config(tmp_path))
    assert result.status is SimStatus.REACHED
    assert result.success is True
    # A positive min clearance means it threaded past the obstacle rather than
    # grazing a wall on the way to the goal.
    assert result.min_clearance > 0.0


# --- (b) every executed command stays within the declared v/omega limits ------
def test_commands_respect_limits(tmp_path: Path) -> None:
    # The box projection + accel clamp hold on every tick, so a short capped run
    # exercises the invariant without paying for the full episode that (a) already
    # runs to REACHED.
    params = _config(tmp_path, max_steps=60)
    buf = io.StringIO()
    recorder = TraceRecorder(buf)
    _run(params, recorder)

    v_max = params.get_float("v_max")
    omega_max = params.get_float("omega_max")
    events = (json.loads(line) for line in buf.getvalue().splitlines())
    moves = [e for e in events if e["event"] == "robot_moved"]
    assert moves
    for move in moves:
        assert abs(move["data"]["v"]) <= v_max + 1e-9
        assert abs(move["data"]["omega"]) <= omega_max + 1e-9


# --- (c) more descent iterations never raise the optimized cost ---------------
def test_more_iterations_lower_cost(tmp_path: Path) -> None:
    (tmp_path / "lo").mkdir()
    (tmp_path / "hi").mkdir()
    low = _single_tick_band(_config(tmp_path / "lo", iterations=1))
    high = _single_tick_band(_config(tmp_path / "hi", iterations=30))
    assert high["data"]["total_cost"] <= low["data"]["total_cost"] + 1e-6


# --- (d) a stronger obstacle weight bends the predicted horizon away -----------
def test_obstacle_weight_increases_clearance(tmp_path: Path) -> None:
    # Deterministic single-tick test on a synthetic grid: one obstacle sits just
    # off the straight line from the robot to a goal set straight ahead, within
    # the prediction horizon's reach. With w_obstacle = 0 the predicted horizon
    # heads straight for the goal and passes close to the obstacle; a strong
    # w_obstacle bends that same horizon away, raising its minimum continuous
    # nearest-occupied clearance (the quantity the cost term optimizes). This
    # isolates the obstacle term from any full-episode reach.
    resolution = 0.05
    pixels = np.full((80, 160), 255, dtype=np.uint16)  # 4 m x 8 m free field
    obstacle = (2.0, 2.0)
    half = 0.15
    for dx in np.arange(-half, half, resolution):
        for dy in np.arange(-half, half, resolution):
            col = int(round((obstacle[0] + dx) / resolution - 0.5))
            row = int(round((obstacle[1] + dy) / resolution - 0.5))
            pixels[row, col] = 0
    grid = OccupancyGrid2D(pixels=pixels, resolution=resolution, origin=(0.0, 0.0, 0.0))

    def band_min_clearance(tag: str, w_obstacle: float) -> float:
        sub = tmp_path / tag
        sub.mkdir()
        params = _config(sub, w_obstacle=w_obstacle)
        planner = MpcPlanner(params)
        state = RobotState(pose=(1.0, 1.5, 0.0))
        task = LocalTask(goal=(6.0, 1.5, 0.0))
        buf = io.StringIO()
        dt = params.get_float("control_dt")
        planner.compute_command(grid, state, task, dt, TraceRecorder(buf))
        band = _band_events(buf)[-1]["band"]
        clearances = [
            d_tilde
            for x, y, *_ in band
            for _, d_tilde in [nearest_occupied(grid, (x, y), 3.0)]
            if d_tilde != float("inf")
        ]
        assert clearances
        return min(clearances)

    assert band_min_clearance("high", 200.0) > band_min_clearance("low", 0.0)


# --- (e) the emitted horizon band always has horizon+1 poses ------------------
def test_horizon_length(tmp_path: Path) -> None:
    # The band-length invariant holds on every tick, so a short capped run
    # suffices (no need for the full episode).
    params = _config(tmp_path, max_steps=25)
    horizon = params.get_int("horizon")
    buf = io.StringIO()
    recorder = TraceRecorder(buf)
    _run(params, recorder)
    bands = _band_events(buf)
    assert bands
    for band in bands:
        assert len(band["band"]) == horizon + 1


# --- (f) an out-of-range weight fails load-time validation --------------------
def test_param_validation(tmp_path: Path) -> None:
    with pytest.raises(ParamError):
        _config(tmp_path, w_goal=-1.0)
