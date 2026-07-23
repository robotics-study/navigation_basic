"""RVO (van den Berg, Lin & Manocha 2008): VO with the cone apex shifted toward
the midpoint of both agents' velocities so a symmetric encounter doesn't
oscillate. Verified the same way as VO: reciprocal avoidance on the shared
multi-body scenarios, and an honest failure when there is no room to avoid."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml
from conftest import REPO_ROOT

from navigation.core.params import ParamError, ParamSet
from navigation.core.types import RobotState
from navigation.local_planning.simulation import SimConfig, SimStatus
from navigation.local_planning.velocity.agent_scenario import load_agent_scenario
from navigation.local_planning.velocity.agent_sim import AgentSpec, simulate_agents
from navigation.local_planning.velocity.rvo import Rvo
from navigation.maps.loader import load_map
from navigation.maps.occupancy_grid import OccupancyGrid2D

_ALGO = "rvo"
_CONFIG_PATH = REPO_ROOT / "configs" / "local_planning" / f"{_ALGO}.yaml"
_MAPS_DIR = REPO_ROOT / "maps"


def _config(tmp_path: Path, **overrides: Any) -> ParamSet:
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


def _run_scenario(scenario_name: str, params: ParamSet) -> list[Any]:
    scenario = load_agent_scenario(_MAPS_DIR / "scenarios" / "velocity" / f"{scenario_name}.yaml")
    grid = load_map(scenario.map_path)
    planners = [Rvo(params) for _ in scenario.agents]
    return simulate_agents(planners, list(scenario.agents), grid, _sim_config(params))


# --- (a) reciprocal avoidance succeeds on the shared multi-body scenarios -----
def test_head_on_two_agents_avoid_and_reach(tmp_path: Path) -> None:
    params = _config(tmp_path)
    results = _run_scenario("head_on", params)
    assert all(r.status != SimStatus.COLLISION for r in results)
    assert all(r.min_pair_clearance >= 0.0 for r in results)
    assert all(r.status == SimStatus.REACHED for r in results)


def test_circle_swap_four_agents_avoid_and_reach(tmp_path: Path) -> None:
    params = _config(tmp_path)
    results = _run_scenario("circle_swap", params)
    assert all(r.status != SimStatus.COLLISION for r in results)
    assert all(r.min_pair_clearance >= 0.0 for r in results)
    assert all(r.status == SimStatus.REACHED for r in results)


# --- (b) an honest failure: no room to avoid a head-on scripted mover ---------
def test_narrow_corridor_head_on_mover_fails_honestly(tmp_path: Path) -> None:
    params = _config(tmp_path, max_steps=300)
    resolution = 0.1
    band_rows = 8  # 0.8 m: below the 1.2 m two 0.3 m-radius bodies need to pass
    margin_rows = 10
    cols = 100
    pixels = np.zeros((band_rows + 2 * margin_rows, cols), dtype=np.uint16)
    pixels[margin_rows : margin_rows + band_rows, :] = 255
    grid = OccupancyGrid2D(pixels=pixels, resolution=resolution, origin=(0.0, 0.0, 0.0))
    corridor_y = grid.cell_to_world(margin_rows + band_rows // 2, 0)[1]

    planner_spec = AgentSpec(
        start=RobotState(pose=(1.0, corridor_y, 0.0)), goal=(9.0, corridor_y, 0.0), radius=0.3
    )
    mover_spec = AgentSpec(
        start=RobotState(pose=(9.0, corridor_y, 0.0)),
        goal=(1.0, corridor_y, 0.0),
        radius=0.3,
        scripted_velocity=(-1.0, 0.0),
    )
    results = simulate_agents(
        [Rvo(params), None], [planner_spec, mover_spec], grid, _sim_config(params)
    )
    assert results[0].status in (SimStatus.COLLISION, SimStatus.STALLED)
    assert results[0].status != SimStatus.REACHED


# --- (c) parameter validation failures at load time ---------------------------
def test_time_horizon_at_zero_rejected_at_load_time(tmp_path: Path) -> None:
    with pytest.raises(ParamError):
        _config(tmp_path, time_horizon=0.0)


def test_speed_samples_zero_rejected_at_load_time(tmp_path: Path) -> None:
    with pytest.raises(ParamError):
        _config(tmp_path, speed_samples=0)


# --- (d) determinism: identical rerun -> bit-identical per-agent trajectories -
def test_rerun_is_deterministic(tmp_path: Path) -> None:
    params = _config(tmp_path)
    scenario = load_agent_scenario(_MAPS_DIR / "scenarios" / "velocity" / "circle_swap.yaml")
    grid = load_map(scenario.map_path)
    config = _sim_config(params)

    first = simulate_agents(
        [Rvo(params) for _ in scenario.agents], list(scenario.agents), grid, config
    )
    second = simulate_agents(
        [Rvo(params) for _ in scenario.agents], list(scenario.agents), grid, config
    )

    for r1, r2 in zip(first, second, strict=True):
        assert r2.status == r1.status
        assert r2.steps == r1.steps
        assert len(r2.trajectory) == len(r1.trajectory)
        for p1, p2 in zip(r1.trajectory, r2.trajectory, strict=True):
            assert p1 == pytest.approx(p2)


# --- reciprocity=0 collapses onto plain VO (behavioral contract, not an echo) -
def test_reciprocity_zero_matches_plain_vo_apex() -> None:
    from navigation.local_planning.velocity._velocity_obstacle import rvo_apex

    v_self = (1.0, 0.5)
    v_other = (-0.5, 0.2)
    assert rvo_apex(v_self, v_other, 0.0) == pytest.approx(v_other)
    assert rvo_apex(v_self, v_other, 1.0) == pytest.approx(v_self)
