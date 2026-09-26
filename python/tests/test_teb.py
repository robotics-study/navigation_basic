"""TEB (Rösmann et al. 2012/2017): a fixed-iteration damped gradient-descent
solver jointly optimizing a timed pose chain against reference tracking,
obstacle clearance, velocity/acceleration limits, time-optimality, and a
nonholonomic two-pose-arc constraint -- verified via the closed-loop simulator
on a real map/scenario (a path skimming an obstacle's edge) and via direct
compute_command calls for behavior a full episode can't isolate cleanly (a
single tick's cost-term response to a weight change, resize's pose-count
bounds)."""

from __future__ import annotations

import io
import json
from pathlib import Path
from typing import Any

import pytest
import yaml
from conftest import REPO_ROOT

from navigation.core.params import ParamError, ParamSet
from navigation.core.trace import TraceRecorder
from navigation.core.types import LocalTask, RobotState
from navigation.local_planning._geometry import nearest_occupied
from navigation.local_planning.band.teb import TebPlanner
from navigation.local_planning.simulation import SimConfig, SimStatus, integrate_unicycle, simulate
from navigation.maps.loader import load_map, load_scenario

_ALGO = "teb"
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


def _run(
    map_name: str, scenario_name: str, params: ParamSet, recorder: TraceRecorder | None = None
) -> Any:
    grid = load_map(_MAPS_DIR / "grid" / f"{map_name}.yaml")
    scenario = load_scenario(_MAPS_DIR / "scenarios" / f"{scenario_name}.yaml")
    planner = TebPlanner(params)
    start = RobotState(pose=(scenario.start[0], scenario.start[1], scenario.start_theta))
    task = LocalTask(
        goal=(scenario.goal[0], scenario.goal[1], scenario.goal_theta),
        reference_path=scenario.reference_path,
    )
    return simulate(planner, grid, start, task, _sim_config(params), recorder)


def _band_events(buf: io.StringIO) -> list[dict[str, Any]]:
    events = (json.loads(line) for line in buf.getvalue().splitlines())
    return [e for e in events if e["event"] == "band_updated"]


# --- (a) skims the clutter map's top-right obstacle edge without colliding ----
def test_reaches_goal_on_skimming_path(tmp_path: Path) -> None:
    params = _config(tmp_path)
    result = _run("clutter01", "clutter01_s2", params)
    assert result.status is SimStatus.REACHED
    assert result.success is True


# --- (b) every executed command stays within the declared v/omega limits ------
def test_commands_respect_limits(tmp_path: Path) -> None:
    params = _config(tmp_path)
    buf = io.StringIO()
    recorder = TraceRecorder(buf)
    result = _run("clutter01", "clutter01_s2", params, recorder)
    assert result.status is SimStatus.REACHED

    v_max = params.get_float("v_max")
    omega_max = params.get_float("omega_max")
    events = (json.loads(line) for line in buf.getvalue().splitlines())
    moves = [e for e in events if e["event"] == "robot_moved"]
    assert moves
    for move in moves:
        assert abs(move["data"]["v"]) <= v_max + 1e-9
        assert abs(move["data"]["omega"]) <= omega_max + 1e-9


# --- (c) raising w_time shortens the optimized band's total horizon time ------
def test_time_weight_shortens_band_time(tmp_path: Path) -> None:
    grid = load_map(_MAPS_DIR / "grid" / "clutter01.yaml")
    scenario = load_scenario(_MAPS_DIR / "scenarios" / "clutter01_s2.yaml")
    state = RobotState(pose=(scenario.start[0], scenario.start[1], scenario.start_theta))
    task = LocalTask(
        goal=(scenario.goal[0], scenario.goal[1], scenario.goal_theta),
        reference_path=scenario.reference_path,
    )

    def horizon_time(w_time: float) -> float:
        params = _config(tmp_path / f"w{w_time}", w_time=w_time)
        planner = TebPlanner(params)
        buf = io.StringIO()
        recorder = TraceRecorder(buf)
        planner.compute_command(grid, state, task, params.get_float("control_dt"), recorder)
        bands = _band_events(buf)
        assert len(bands) == 1
        return float(bands[0]["data"]["horizon_time"])

    (tmp_path / "w0.0").mkdir()
    (tmp_path / "w5.0").mkdir()
    off = horizon_time(0.0)
    on = horizon_time(5.0)
    assert on < off


# --- (d) raising w_obstacle increases clearance while skimming the obstacle ---
def test_obstacle_weight_increases_clearance(tmp_path: Path) -> None:
    # SimResult.min_clearance is the quantized EDT's trajectory minimum, which
    # for this scenario ties at exactly footprint_radius (0.2) plus one cell's
    # worth of slack near the start regardless of w_obstacle -- not sensitive
    # enough to show the obstacle term's effect. Instead this measures the
    # continuous nearest-occupied-cell distance d~ (the same quantity the
    # obstacle cost term itself optimizes) directly via occupied_within, at
    # every trajectory point within a fixed radius of some obstacle, averaged
    # over the whole episode. A single nearest-block minimum is too path-
    # shape-sensitive for a two-point comparison (the ~40-iteration damped
    # solver never fully converges, so two runs' emergent paths can cross at
    # any one obstacle without either being "wrong") -- the trajectory-wide
    # average is the aggregate the cost term is actually shaped to minimize.
    grid = load_map(_MAPS_DIR / "grid" / "clutter01.yaml")

    # Radius wide enough to reach some occupied cell from virtually every point
    # of this cluttered map's trajectory (empirically ~everywhere here) --
    # narrower radii undercount to a handful of very-local points, which are
    # exactly as path-shape-sensitive as the single-block minimum above.
    near_radius = 1.5

    def mean_near_clearance(w_obstacle: float) -> float:
        params = _config(tmp_path / f"o{w_obstacle}", w_obstacle=w_obstacle)
        result = _run("clutter01", "clutter01_s2", params)
        assert result.status is SimStatus.REACHED
        near = [
            d_tilde
            for x, y, _ in result.trajectory
            for _, d_tilde in [nearest_occupied(grid, (x, y), near_radius)]
            if d_tilde != float("inf")
        ]
        assert near
        return sum(near) / len(near)

    (tmp_path / "o0.0").mkdir()
    (tmp_path / "o15.0").mkdir()
    off = mean_near_clearance(0.0)
    default = mean_near_clearance(15.0)
    assert default > off


# --- (e) resize never lets the band's pose count leave [3, max_poses] ---------
def test_band_resizing_bounds(tmp_path: Path) -> None:
    # A window short enough that the robot stays far from the local goal (so
    # the n<3 degenerate branch, which legitimately reports poses=2 right
    # before REACHED, never triggers) but long enough to exercise several
    # resize passes.
    params = _config(tmp_path, max_steps=60)
    grid = load_map(_MAPS_DIR / "grid" / "clutter01.yaml")
    scenario = load_scenario(_MAPS_DIR / "scenarios" / "clutter01_s2.yaml")
    planner = TebPlanner(params)
    state = RobotState(pose=(scenario.start[0], scenario.start[1], scenario.start_theta))
    task = LocalTask(
        goal=(scenario.goal[0], scenario.goal[1], scenario.goal_theta),
        reference_path=scenario.reference_path,
    )
    dt = params.get_float("control_dt")
    max_poses = params.get_int("max_poses")
    planner.reset()

    buf = io.StringIO()
    recorder = TraceRecorder(buf)
    for _ in range(60):
        cmd = planner.compute_command(grid, state, task, dt, recorder)
        new_pose = integrate_unicycle(state.pose, cmd, dt)
        state = RobotState(new_pose, cmd.v, cmd.omega)

    bands = _band_events(buf)
    assert bands
    for band in bands:
        poses = band["data"]["poses"]
        assert 3 <= poses <= max_poses


# --- (f) an out-of-range weight fails load-time validation --------------------
def test_param_validation(tmp_path: Path) -> None:
    with pytest.raises(ParamError):
        _config(tmp_path, w_path=-1.0)
