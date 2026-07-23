"""ORCA (van den Berg, Guy, Lin & Manocha 2011): an exact half-plane per
obstacle plus a deterministic 2D linear program, falling back to a
penetration-minimizing 3D solve when jointly infeasible. Verified via the
multi-agent harness (reciprocal avoidance on the shared scenarios) and direct
linear-program unit tests (closest-feasible-point selection, and the
hot-path-never-raises fallback contract)."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import pytest
import yaml
from conftest import REPO_ROOT

from navigation.core.params import ParamError, ParamSet
from navigation.local_planning.simulation import SimConfig, SimStatus
from navigation.local_planning.velocity._velocity_obstacle import (
    linear_program_2d,
    linear_program_3d,
)
from navigation.local_planning.velocity.agent_scenario import load_agent_scenario
from navigation.local_planning.velocity.agent_sim import simulate_agents
from navigation.local_planning.velocity.orca import Orca
from navigation.maps.loader import load_map

_ALGO = "orca"
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
    planners = [Orca(params) for _ in scenario.agents]
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


# --- (b) / ORCA LP unit: feasible set -> closest point to v_pref --------------
def test_linear_program_2d_selects_closest_feasible_point() -> None:
    # Two half-planes (x >= 1, y >= 1) intersecting in a quarter-plane whose
    # nearest corner to v_pref=(0,0) is (1,1) -- verifies the LP actually
    # projects onto the feasible region rather than just clamping to max_speed.
    planes = [((1.0, 0.0), (1.0, 0.0)), ((0.0, 1.0), (0.0, 1.0))]
    ok, result, fail_index = linear_program_2d(planes, (0.0, 0.0), 10.0)
    assert ok is True
    assert fail_index == len(planes)
    assert result == pytest.approx((1.0, 1.0))


# --- (b) / ORCA LP unit: over-constrained set -> 3D fallback never raises -----
def test_linear_program_3d_fallback_never_raises_on_over_constrained_set() -> None:
    # A third half-plane (x + y <= -6) makes the intersection with (x>=1, y>=1)
    # empty: linear_program_2d must report failure, and linear_program_3d must
    # still return a finite Point (penetration-min fallback) instead of raising.
    planes = [
        ((1.0, 0.0), (1.0, 0.0)),
        ((0.0, 1.0), (0.0, 1.0)),
        ((-3.0, -3.0), (-1.0, -1.0)),
    ]
    ok, result, fail_index = linear_program_2d(planes, (0.0, 0.0), 10.0)
    assert ok is False
    assert fail_index < len(planes)

    fallback = linear_program_3d(planes, fail_index, (0.0, 0.0), 10.0)
    assert len(fallback) == 2
    assert all(math.isfinite(c) for c in fallback)


# --- (c) parameter validation failure at load time ----------------------------
def test_time_horizon_at_zero_rejected_at_load_time(tmp_path: Path) -> None:
    with pytest.raises(ParamError):
        _config(tmp_path, time_horizon=0.0)


# --- (d) determinism: identical rerun -> bit-identical per-agent trajectories -
def test_rerun_is_deterministic(tmp_path: Path) -> None:
    params = _config(tmp_path)
    scenario = load_agent_scenario(_MAPS_DIR / "scenarios" / "velocity" / "circle_swap.yaml")
    grid = load_map(scenario.map_path)
    config = _sim_config(params)

    first = simulate_agents(
        [Orca(params) for _ in scenario.agents], list(scenario.agents), grid, config
    )
    second = simulate_agents(
        [Orca(params) for _ in scenario.agents], list(scenario.agents), grid, config
    )

    for r1, r2 in zip(first, second, strict=True):
        assert r2.status == r1.status
        assert r2.steps == r1.steps
        assert len(r2.trajectory) == len(r1.trajectory)
        for p1, p2 in zip(r1.trajectory, r2.trajectory, strict=True):
            assert p1 == pytest.approx(p2)
