"""AIT*: adaptive-heuristic batch planner — reaches the goal, fails when walled
off, respects param validation, and returns a near-optimal path no worse than a
plain RRT's on the same scenario."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml
from conftest import CONFIG_DIR, config, grid_from, open_grid

from navigation.core.params import ParamError, ParamSet
from navigation.global_planning import RRT
from navigation.global_planning.sampling.ait_star import AITStar
from navigation.maps.occupancy_grid import OccupancyGrid2D

# Small batch budget: still reaches the goal on the open grid while staying fast.
_BATCH = {"batch_size": 150, "max_batches": 4}


def _batch_config(tmp_path: Path) -> ParamSet:
    doc = yaml.safe_load((CONFIG_DIR / "ait_star.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] in _BATCH:
            entry["default"] = _BATCH[entry["name"]]
    out = tmp_path / "ait_star.yaml"
    out.write_text(yaml.safe_dump(doc), encoding="utf-8")
    return ParamSet.from_yaml(out)


def _motion_valid_path(grid: OccupancyGrid2D, path: list[tuple[float, float]]) -> bool:
    return all(grid.is_motion_valid(a, b) for a, b in zip(path, path[1:], strict=False))


def test_reaches_goal_on_open_grid(tmp_path: Path) -> None:
    grid = open_grid(10, 10, seed=7)
    start, goal = (0.5, 0.5), (9.0, 9.0)
    result = AITStar(_batch_config(tmp_path)).plan(grid, start, goal)
    assert result.success
    # start/goal are permanent samples, so the path pins to them exactly.
    assert result.path[0] == pytest.approx(start)
    assert result.path[-1] == pytest.approx(goal)
    assert _motion_valid_path(grid, result.path)
    assert result.cost > 0.0


def test_no_path_when_walled_off(tmp_path: Path) -> None:
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."], seed=3)
    result = AITStar(_batch_config(tmp_path)).plan(grid, (0.5, 0.5), (4.5, 4.5))
    assert not result.success
    assert result.path == []
    assert result.cost == 0.0


def test_near_straight_line_on_open_grid(tmp_path: Path) -> None:
    # Forward A* over the RGG returns the true graph-shortest path, so on an
    # obstacle-free grid the cost sits close to the straight-line lower bound.
    grid = open_grid(12, 12, seed=5)
    start, goal = (0.5, 0.5), (11.0, 11.0)
    lower_bound = grid.distance(start, goal)
    result = AITStar(_batch_config(tmp_path)).plan(grid, start, goal)
    assert result.success
    assert result.cost <= lower_bound * 1.15


def test_no_worse_than_plain_rrt(tmp_path: Path) -> None:
    # AIT* is asymptotically optimal; a plain RRT returns the first feasible path.
    # On the same scenario+budget AIT*'s path must be no longer than RRT's.
    grid = open_grid(12, 12, seed=11)
    start, goal = (0.5, 0.5), (11.0, 11.0)
    rrt = RRT(config("rrt")).plan(grid, start, goal)
    ait = AITStar(_batch_config(tmp_path)).plan(grid, start, goal)
    assert rrt.success and ait.success
    assert ait.cost <= rrt.cost + 1e-6


def test_bad_default_param_throws(tmp_path: Path) -> None:
    doc = yaml.safe_load((CONFIG_DIR / "ait_star.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] == "gamma":
            entry["default"] = 5000.0  # above declared max 1000.0
    out = tmp_path / "ait_star.yaml"
    out.write_text(yaml.safe_dump(doc), encoding="utf-8")
    with pytest.raises(ParamError):
        ParamSet.from_yaml(out)
