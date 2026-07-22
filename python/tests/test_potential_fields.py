"""Potential Fields (Khatib 1986): goal-seek + obstacle-avoidance behavior via
the closed-loop simulator, including the well-known local-minimum stall."""

from __future__ import annotations

import io
import json
from pathlib import Path
from typing import Any

import pytest
import yaml
from conftest import REPO_ROOT, grid_from

from navigation.core.params import ParamError, ParamSet
from navigation.core.trace import TraceRecorder
from navigation.core.types import LocalTask, RobotState
from navigation.local_planning.reactive.potential_fields import PotentialFields
from navigation.local_planning.simulation import SimConfig, SimResult, SimStatus, simulate
from navigation.maps.loader import load_map, load_scenario

_ALGO = "potential_fields"
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
    planner = PotentialFields(params)
    start = RobotState(pose=(scenario.start[0], scenario.start[1], scenario.start_theta))
    task = LocalTask(goal=(scenario.goal[0], scenario.goal[1], scenario.goal_theta))
    return simulate(planner, grid, start, task, _sim_config(params))


# --- (a) reaches goal on open/clutter maps, never collides --------------------
@pytest.mark.parametrize(
    ("map_name", "scenario_name"), [("open01", "open01_s1"), ("clutter01", "clutter01_s1")]
)
def test_reaches_goal_without_collision(
    tmp_path: Path, map_name: str, scenario_name: str
) -> None:
    result = _run(tmp_path, map_name, scenario_name)
    assert result.status is SimStatus.REACHED
    assert result.success is True
    assert result.min_clearance > 0.0


# --- (b) U-trap: correct local-minimum stall, not a bug ------------------------
def test_stalls_in_u_trap_local_minimum() -> None:
    # pf_trap01_s1: the goal sits straight through the U-trap's back wall, in
    # line with the opening. The attractive pull drives the robot in, and at
    # this row the top/bottom wall repulsion is symmetric so it cancels the
    # attractive pull head-on against the back wall -- a textbook local
    # minimum (Khatib 1986), not a simulator or tuning defect.
    grid = load_map(_MAPS_DIR / "grid" / "pf_trap01.yaml")
    scenario = load_scenario(_MAPS_DIR / "scenarios" / "pf_trap01_s1.yaml")
    params = ParamSet.from_yaml(_CONFIG_PATH)
    planner = PotentialFields(params)
    start = RobotState(pose=(scenario.start[0], scenario.start[1], scenario.start_theta))
    task = LocalTask(goal=(scenario.goal[0], scenario.goal[1], scenario.goal_theta))
    result = simulate(planner, grid, start, task, _sim_config(params))
    assert result.status is SimStatus.STALLED
    assert result.success is False


# --- (c) out-of-range param rejected at load time ------------------------------
def test_out_of_range_k_att_rejected(tmp_path: Path) -> None:
    doc = yaml.safe_load(_CONFIG_PATH.read_text())
    for entry in doc["params"]:
        if entry["name"] == "k_att":
            entry["default"] = entry["max"] + 1.0
    bad = tmp_path / f"{_ALGO}.yaml"
    bad.write_text(yaml.safe_dump(doc), encoding="utf-8")
    with pytest.raises(ParamError):
        ParamSet.from_yaml(bad)


# --- (behavior) repulsive force points away from a single obstacle ------------
def test_repulsive_force_points_away_from_obstacle(tmp_path: Path) -> None:
    # Single obstacle at ascii row 2 / col 2 -> world center (2.5, 2.5) (see
    # OccupancyGrid2D.cell_to_world). Robot sits one cell due north (2.5, 3.5),
    # so the FIRAS repulsion must point further north (+y), away from the
    # obstacle -- an influence_radius just above the 1.0 m gap keeps this the
    # only source, isolating F_rep from the (goal-independent) attractive term.
    grid = grid_from(
        [
            ".......",
            ".......",
            "..#....",
            ".......",
            ".......",
        ]
    )
    params = _config(tmp_path, influence_radius=1.05, k_rep=1.0)
    planner = PotentialFields(params)
    state = RobotState(pose=(2.5, 3.5, 0.0))
    task = LocalTask(goal=(2.5, 3.5, 0.0))  # coincident with start: F_att == 0

    buf = io.StringIO()
    recorder = TraceRecorder(buf)
    planner.compute_command(grid, state, task, params.get_float("control_dt"), recorder)

    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    force_events = [e for e in events if e["event"] == "force_computed"]
    assert len(force_events) == 1
    data = force_events[0]["data"]
    assert data["fx_rep"] == pytest.approx(0.0, abs=1e-9)
    assert data["fy_rep"] > 0.0  # pushed north, away from the obstacle to the south
