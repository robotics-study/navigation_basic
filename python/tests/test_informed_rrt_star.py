"""Informed RRT* (Gammell, Srinivasa & Barfoot 2014): reaches the goal on open
space, fails when walled off, and tightens toward the straight-line optimum."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml
from conftest import CONFIG_DIR, grid_from, open_grid

from navigation.core.params import ParamError, ParamSet
from navigation.global_planning import InformedRRTStar
from navigation.maps.occupancy_grid import OccupancyGrid2D

_ALGO = "informed_rrt_star"


def _capped_config(tmp_path: Path, max_iter: int) -> ParamSet:
    """Real config with a smaller iteration budget so the tests stay fast."""
    doc = yaml.safe_load((CONFIG_DIR / f"{_ALGO}.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] == "max_iterations":
            entry["default"] = max_iter
    out = tmp_path / f"{_ALGO}.yaml"
    out.write_text(yaml.safe_dump(doc), encoding="utf-8")
    return ParamSet.from_yaml(out)


def _motion_valid_path(grid: OccupancyGrid2D, path: list[tuple[float, float]]) -> bool:
    return all(grid.is_motion_valid(a, b) for a, b in zip(path, path[1:], strict=False))


def test_reaches_goal_on_open_grid(tmp_path: Path) -> None:
    grid = open_grid(10, 10, seed=7)
    start, goal = (0.5, 0.5), (9.0, 9.0)
    params = _capped_config(tmp_path, max_iter=2000)
    result = InformedRRTStar(params).plan(grid, start, goal)
    assert result.success
    assert result.path[0] == pytest.approx(start)
    assert grid.distance(result.path[-1], goal) <= params.get_float("goal_tolerance")
    assert _motion_valid_path(grid, result.path)
    assert result.cost > 0.0


def test_no_path_when_walled_off(tmp_path: Path) -> None:
    # Fully occupied middle column: no motion can cross it.
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."], seed=3)
    params = _capped_config(tmp_path, max_iter=400)
    result = InformedRRTStar(params).plan(grid, (0.5, 0.5), (4.5, 4.5))
    assert not result.success
    assert result.path == []
    assert result.cost == 0.0


def test_near_straight_line_optimality(tmp_path: Path) -> None:
    # On an obstacle-free grid, informed sampling should concentrate post-solution
    # draws in the ellipse and drive the cost close to the straight-line lower bound.
    grid = open_grid(12, 12, seed=5)
    start, goal = (0.5, 0.5), (11.0, 11.0)
    lower_bound = grid.distance(start, goal)
    result = InformedRRTStar(_capped_config(tmp_path, max_iter=4000)).plan(grid, start, goal)
    assert result.success
    assert result.cost <= lower_bound * 1.15


def test_out_of_range_default_rejected(tmp_path: Path) -> None:
    # goal_bias default above its declared max must fail validation at load time.
    doc = yaml.safe_load((CONFIG_DIR / f"{_ALGO}.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] == "goal_bias":
            entry["default"] = 2.0
    bad = tmp_path / f"{_ALGO}.yaml"
    bad.write_text(yaml.safe_dump(doc), encoding="utf-8")
    with pytest.raises(ParamError):
        ParamSet.from_yaml(bad)
