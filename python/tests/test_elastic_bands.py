"""Elastic Bands (Quinlan & Khatib 1993): deforms a bubble chain draped over a
reference path away from obstacles each tick, via the closed-loop simulator on
a real map (obstacles the raw reference path cuts straight through) and via
direct compute_command calls for the deformation/maintenance behavior a full
episode can't isolate cleanly (straight-line equilibrium, band breaking)."""

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
from navigation.local_planning.band._band import resample_polyline
from navigation.local_planning.band.elastic_bands import ElasticBandsPlanner
from navigation.local_planning.simulation import SimConfig, SimStatus, simulate
from navigation.maps.loader import load_map, load_scenario

_ALGO = "elastic_bands"
_CONFIG_PATH = REPO_ROOT / "configs" / "local_planning" / f"{_ALGO}.yaml"
_MAPS_DIR = REPO_ROOT / "maps"


def _config(tmp_path: Path, **overrides: Any) -> ParamSet:
    """Real config with selected defaults overridden."""
    doc = yaml.safe_load(_CONFIG_PATH.read_text())
    for entry in doc["params"]:
        if entry["name"] in overrides:
            entry["default"] = overrides[entry["name"]]
    out = tmp_path / f"{_ALGO}.yaml"
    out.write_text(yaml.safe_dump(doc))
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
    planner = ElasticBandsPlanner(params)
    start = RobotState(pose=(scenario.start[0], scenario.start[1], scenario.start_theta))
    task = LocalTask(
        goal=(scenario.goal[0], scenario.goal[1], scenario.goal_theta),
        reference_path=scenario.reference_path,
    )
    return simulate(planner, grid, start, task, _sim_config(params), None)


def _band_events(buf: io.StringIO) -> list[dict[str, Any]]:
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    return [e for e in events if e["event"] == "band_updated"]


# --- (a) the raw reference path cuts through 5 obstacle blocks; the deformed --
# --- band routes around all of them without collision -------------------------
def test_reaches_goal_around_blocking_obstacles(tmp_path: Path) -> None:
    params = _config(tmp_path)
    result = _run("clutter01", "clutter01_s3", params)
    assert result.status is SimStatus.REACHED
    assert result.success is True
    assert result.min_clearance > 0.0


# --- (b) after one tick's deformation, the band's worst-case clearance beats --
# --- the raw (undeformed) reference path's worst-case clearance ---------------
def test_band_deforms_away_from_obstacle(tmp_path: Path) -> None:
    params = _config(tmp_path)
    grid = load_map(_MAPS_DIR / "grid" / "clutter01.yaml")
    scenario = load_scenario(_MAPS_DIR / "scenarios" / "clutter01_s3.yaml")
    planner = ElasticBandsPlanner(params)
    state = RobotState(pose=(scenario.start[0], scenario.start[1], scenario.start_theta))
    task = LocalTask(
        goal=(scenario.goal[0], scenario.goal[1], scenario.goal_theta),
        reference_path=scenario.reference_path,
    )

    spacing = params.get_float("bubble_spacing")
    baseline_points = resample_polyline(list(scenario.reference_path), spacing)
    baseline_min_clearance = min(grid.distance_to_nearest(p) for p in baseline_points)

    buf = io.StringIO()
    recorder = TraceRecorder(buf)
    planner.compute_command(grid, state, task, params.get_float("control_dt"), recorder)

    band = _band_events(buf)[-1]["band"]
    # radius = min(distance_to_nearest, rho_max), already computed per bubble.
    deformed_min_clearance = min(item[2] for item in band)

    assert deformed_min_clearance > baseline_min_clearance


# --- (c) an obstacle-free straight corridor: contraction alone keeps the band -
# --- exactly on the line joining its two fixed endpoints ----------------------
def test_straight_band_stays_straight(tmp_path: Path) -> None:
    params = _config(tmp_path)
    grid = load_map(_MAPS_DIR / "grid" / "open01.yaml")
    start_xy = (2.0, 8.75)
    goal_xy = (8.0, 8.75)  # horizontal segment verified obstacle-free within rho_influence
    planner = ElasticBandsPlanner(params)
    state = RobotState(pose=(start_xy[0], start_xy[1], 0.0))
    task = LocalTask(goal=(goal_xy[0], goal_xy[1], 0.0), reference_path=(start_xy, goal_xy))

    buf = io.StringIO()
    recorder = TraceRecorder(buf)
    planner.compute_command(grid, state, task, params.get_float("control_dt"), recorder)

    band = _band_events(buf)[-1]["band"]
    for _cx, cy, _rho in band:
        assert cy == pytest.approx(start_xy[1], abs=1e-6)


# --- (d) an unreachable rho_min forces the very first tick's validity check ---
# --- to fail: zero command, broken flagged in the trace ------------------------
def test_band_break_yields_zero_command(tmp_path: Path) -> None:
    params = _config(tmp_path, rho_min=1.2)
    grid = load_map(_MAPS_DIR / "grid" / "clutter01.yaml")
    scenario = load_scenario(_MAPS_DIR / "scenarios" / "clutter01_s3.yaml")
    planner = ElasticBandsPlanner(params)
    state = RobotState(pose=(scenario.start[0], scenario.start[1], scenario.start_theta))
    task = LocalTask(
        goal=(scenario.goal[0], scenario.goal[1], scenario.goal_theta),
        reference_path=scenario.reference_path,
    )

    buf = io.StringIO()
    recorder = TraceRecorder(buf)
    cmd = planner.compute_command(grid, state, task, params.get_float("control_dt"), recorder)

    assert cmd.v == 0.0
    assert cmd.omega == 0.0
    events = _band_events(buf)
    assert len(events) == 1
    assert events[0]["data"]["broken"] == 1.0


# --- (e) an out-of-range override fails load-time validation ------------------
def test_param_validation(tmp_path: Path) -> None:
    with pytest.raises(ParamError):
        _config(tmp_path, k_repulsion=-1.0)
