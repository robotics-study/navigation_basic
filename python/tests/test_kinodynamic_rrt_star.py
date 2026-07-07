"""Kinodynamic RRT* (Webb & van den Berg 2013): finds a dynamically-feasible,
collision-free path to the goal, fails gracefully when walled off, validates its
params, and its optimal-steering cost obeys the metric contract."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml
from conftest import CONFIG_DIR, grid_from, open_grid

from navigation.core.params import ParamError, ParamSet
from navigation.global_planning.sampling.kinodynamic_rrt_star import (
    KinodynamicRRTStar,
    optimal_cost,
)
from navigation.maps.occupancy_grid import OccupancyGrid2D

_ALGO = "kinodynamic_rrt_star"


def _capped_config(tmp_path: Path, **overrides: float) -> ParamSet:
    """Real config with per-name overrides so tests stay fast and deterministic."""
    doc = yaml.safe_load((CONFIG_DIR / f"{_ALGO}.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] in overrides:
            entry["default"] = overrides[entry["name"]]
    out = tmp_path / f"{_ALGO}.yaml"
    out.write_text(yaml.safe_dump(doc), encoding="utf-8")
    return ParamSet.from_yaml(out)


def _motion_valid_path(grid: OccupancyGrid2D, path: list[tuple[float, float]]) -> bool:
    return all(grid.is_motion_valid(a, b) for a, b in zip(path, path[1:], strict=False))


def test_reaches_goal_with_feasible_collision_free_path(tmp_path: Path) -> None:
    grid = open_grid(10, 10, seed=7)
    start, goal = (0.75, 0.75), (9.0, 9.0)
    params = _capped_config(
        tmp_path, max_iterations=1500, goal_bias=0.2, goal_tolerance=1.5, seed=3
    )
    result = KinodynamicRRTStar(params).plan(grid, start, goal)
    assert result.success
    assert result.path[0] == pytest.approx(start)
    # Optimal steering arrives exactly at the goal rest-state; the projected endpoint
    # is within tolerance of the goal position.
    assert grid.distance(result.path[-1], goal) <= params.get_float("goal_tolerance")
    # The propagated double-integrator trajectory must be collision-free throughout.
    assert _motion_valid_path(grid, result.path)
    assert result.cost > 0.0


def test_no_path_when_walled_off(tmp_path: Path) -> None:
    # Fully occupied middle column: no trajectory can cross it.
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."], seed=3)
    params = _capped_config(tmp_path, max_iterations=300, goal_tolerance=1.0)
    result = KinodynamicRRTStar(params).plan(grid, (0.5, 0.5), (4.5, 4.5))
    assert not result.success
    assert result.path == []
    assert result.cost == 0.0


def test_out_of_range_default_rejected(tmp_path: Path) -> None:
    # control_weight default below its declared min must fail validation at load time.
    doc = yaml.safe_load((CONFIG_DIR / f"{_ALGO}.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] == "control_weight":
            entry["default"] = 0.0  # below min 0.001
    bad = tmp_path / f"{_ALGO}.yaml"
    bad.write_text(yaml.safe_dump(doc), encoding="utf-8")
    with pytest.raises(ParamError):
        ParamSet.from_yaml(bad)


def test_optimal_cost_is_a_metric_contract() -> None:
    r = 1.0
    at_rest = (1.0, 2.0, 0.0, 0.0)
    # Coincident states cost nothing (τ*=0), the identity of the cost geometry.
    cost_self, tau_self = optimal_cost(at_rest, at_rest, r)
    assert cost_self == 0.0
    assert tau_self == 0.0
    # Distinct states cost strictly more than zero, reached at a positive arrival time.
    for x1 in [(4.0, 6.0, 0.0, 0.0), (1.0, 2.0, 1.0, -1.0), (0.0, 0.0, 0.5, 0.5)]:
        cost, tau = optimal_cost(at_rest, x1, r)
        assert cost > 0.0
        assert tau > 0.0


def test_higher_control_weight_costs_more() -> None:
    # J = ∫ 1 + r·uᵀu dt: a larger control weight penalises the same maneuver more.
    x0, x1 = (0.0, 0.0, 0.0, 0.0), (3.0, 2.0, 0.0, 0.0)
    cheap, _ = optimal_cost(x0, x1, 0.5)
    dear, _ = optimal_cost(x0, x1, 5.0)
    assert dear > cheap
