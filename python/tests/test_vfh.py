"""VFH (Borenstein & Koren 1991): polar-histogram reactive obstacle avoidance
via the closed-loop simulator -- clutter navigation, total-blockage stall, and
the polar-histogram trace contract (bins length, selected-direction sign)."""

from __future__ import annotations

import io
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml
from conftest import REPO_ROOT, grid_from

from navigation.core.capabilities import Capability
from navigation.core.params import ParamError, ParamSet
from navigation.core.trace import TraceRecorder, open_trace
from navigation.core.types import LocalTask, RobotState
from navigation.local_planning.reactive.vfh import Vfh
from navigation.local_planning.simulation import SimConfig, SimResult, SimStatus, simulate
from navigation.maps.loader import load_map, load_scenario
from navigation.maps.occupancy_grid import OccupancyGrid2D

_ALGO = "vfh"
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


def _run(tmp_path: Path, map_name: str, scenario_name: str, **overrides: Any) -> SimResult:
    params = _config(tmp_path, **overrides)
    grid = load_map(_MAPS_DIR / "grid" / f"{map_name}.yaml")
    scenario = load_scenario(_MAPS_DIR / "scenarios" / f"{scenario_name}.yaml")
    planner = Vfh(params)
    start = RobotState(pose=(scenario.start[0], scenario.start[1], scenario.start_theta))
    task = LocalTask(goal=(scenario.goal[0], scenario.goal[1], scenario.goal_theta))
    return simulate(planner, grid, start, task, _sim_config(params))


# --- (a) reaches goal on the cluttered map without ever colliding --------------
def test_reaches_goal_on_clutter_map_weaving_a_gap(tmp_path: Path) -> None:
    result = _run(tmp_path, "clutter01", "clutter01_s1")
    assert result.status is SimStatus.REACHED
    assert result.success is True
    assert result.min_clearance > 0.0


# --- (b) fully enclosed: every sector blocked -> no forward command -> STALLED -
def test_fully_enclosed_emits_no_forward_command_and_stalls(tmp_path: Path) -> None:
    # A single free cell boxed in on all 8 neighbors, resolution small enough
    # (0.2 m) that window_radius=1.32 (default) comfortably covers even the
    # diagonal neighbors (0.283 m away) well above threshold, so every sector
    # -- not just the 4 cardinal ones -- reads as blocked.
    pixels = np.zeros((3, 3), dtype=np.uint16)
    pixels[1, 1] = 255
    grid = OccupancyGrid2D(pixels=pixels, resolution=0.2, origin=(0.0, 0.0, 0.0))
    cx, cy = grid.cell_to_world(1, 1)

    params = _config(tmp_path)
    planner = Vfh(params)
    assert planner.name == "vfh"
    assert planner.required_capabilities() == {Capability.OBSTACLE_QUERY}
    state = RobotState(pose=(cx, cy, 0.0))
    task = LocalTask(goal=(100.0, 100.0, 0.0))  # far away in every direction: no lucky alignment
    cmd = planner.compute_command(grid, state, task, params.get_float("control_dt"), None)
    assert cmd.v == 0.0

    # Recorder present on the no-valley branch: still emits a histogram_updated
    # event (viz needs the blocked-out histogram even when nothing is selected),
    # with selected_direction falling back to target_direction.
    buf = io.StringIO()
    recorder = TraceRecorder(buf)
    planner.compute_command(grid, state, task, params.get_float("control_dt"), recorder)
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    hist = [e for e in events if e["event"] == "histogram_updated"]
    assert len(hist) == 1
    assert len(hist[0]["bins"]) == params.get_int("num_sectors")
    assert hist[0]["data"]["selected_direction"] == hist[0]["data"]["target_direction"]

    start = RobotState(pose=(cx, cy, 0.0))
    config = SimConfig(
        control_dt=params.get_float("control_dt"),
        max_steps=50,
        goal_tolerance=0.05,
        footprint_radius=0.06,
        stall_window=5,
        stall_distance=0.01,
    )
    result = simulate(planner, grid, start, task, config)
    assert result.status is SimStatus.STALLED
    assert result.success is False


# --- (c) param validation failures at load / construction time -----------------
def test_num_sectors_below_min_rejected(tmp_path: Path) -> None:
    doc = yaml.safe_load(_CONFIG_PATH.read_text())
    for entry in doc["params"]:
        if entry["name"] == "num_sectors":
            entry["default"] = entry["min"] - 1
    bad = tmp_path / f"{_ALGO}.yaml"
    bad.write_text(yaml.safe_dump(doc), encoding="utf-8")
    with pytest.raises(ParamError):
        ParamSet.from_yaml(bad)


def test_even_smoothing_window_rejected(tmp_path: Path) -> None:
    # param_schema.json cannot express "must be odd", so this is enforced by
    # the planner's own constructor (load time, not compute_command) -- an
    # even window has no unambiguous center sector for the moving average.
    params = _config(tmp_path, smoothing_window=4)
    with pytest.raises(ParamError):
        Vfh(params)


# --- (behavior) frontal obstacle + open side -----------------------------------
def test_frontal_obstacle_with_open_side_steers_toward_opening_and_reports_bins(
    tmp_path: Path,
) -> None:
    # Block sits east + southeast of the robot's free cell (row3 col3, row4-5
    # col3-4); goal is due east (straight into the block), north (rows 0-2) is
    # the only open side -- selected_direction must swing toward +y (north),
    # not stay near the blocked eastward goal bearing.
    grid = grid_from(
        [
            ".......",
            ".......",
            ".......",
            "...#...",
            "...##..",
            "...##..",
            ".......",
        ]
    )
    params = _config(tmp_path)
    planner = Vfh(params)
    state = RobotState(pose=(2.5, 3.5, 0.0))
    task = LocalTask(goal=(10.0, 3.5, 0.0))

    trace_path = tmp_path / "vfh.trace.jsonl"
    with open_trace(str(trace_path)) as recorder:
        planner.compute_command(grid, state, task, params.get_float("control_dt"), recorder)

    lines = [json.loads(line) for line in trace_path.read_text().splitlines()]
    hist_events = [line for line in lines if line["event"] == "histogram_updated"]
    assert len(hist_events) == 1
    data = hist_events[0]["data"]
    assert len(hist_events[0]["bins"]) == params.get_int("num_sectors")
    # Open side is north (+y): selected_direction should sit well off the
    # blocked eastward goal bearing (0 rad) and swing positive (CCW, toward +y).
    assert data["selected_direction"] > math.radians(20.0)
    assert data["selected_direction"] < math.radians(160.0)

    candidate_events = [line for line in lines if line["event"] == "candidate_evaluated"]
    assert len(candidate_events) >= 1
    assert sum(1 for e in candidate_events if e["data"]["selected"] == 1.0) == 1
