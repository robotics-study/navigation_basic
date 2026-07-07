"""ABIT*: inflated/truncated batch planner — reaches the goal, fails when walled
off, respects param validation, converges to a near-optimal (BIT*-grade) path as
its ε schedule relaxes to 1, and keeps the anytime property under heavy inflation."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml
from conftest import CONFIG_DIR, config, grid_from, open_grid

from navigation.core.params import ParamError, ParamSet
from navigation.global_planning import RRT
from navigation.global_planning.sampling import ABITStar
from navigation.maps.occupancy_grid import OccupancyGrid2D

# Small batch budget: still reaches the goal on the open grid while staying fast.
_BATCH = {"batch_size": 150, "max_batches": 4}


def _config(tmp_path: Path, overrides: dict[str, float]) -> ParamSet:
    doc = yaml.safe_load((CONFIG_DIR / "abit_star.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] in overrides:
            entry["default"] = overrides[entry["name"]]
    out = tmp_path / "abit_star.yaml"
    out.write_text(yaml.safe_dump(doc), encoding="utf-8")
    return ParamSet.from_yaml(out)


def _motion_valid_path(grid: OccupancyGrid2D, path: list[tuple[float, float]]) -> bool:
    return all(grid.is_motion_valid(a, b) for a, b in zip(path, path[1:], strict=False))


def test_reaches_goal_on_open_grid(tmp_path: Path) -> None:
    grid = open_grid(10, 10, seed=7)
    start, goal = (0.5, 0.5), (9.0, 9.0)
    result = ABITStar(_config(tmp_path, _BATCH)).plan(grid, start, goal)
    assert result.success
    # start/goal are permanent samples, so the path pins to them exactly.
    assert result.path[0] == pytest.approx(start)
    assert result.path[-1] == pytest.approx(goal)
    assert _motion_valid_path(grid, result.path)
    assert result.cost > 0.0


def test_no_path_when_walled_off(tmp_path: Path) -> None:
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."], seed=3)
    result = ABITStar(_config(tmp_path, _BATCH)).plan(grid, (0.5, 0.5), (4.5, 4.5))
    assert not result.success
    assert result.path == []
    assert result.cost == 0.0


def test_near_optimal_after_deflation(tmp_path: Path) -> None:
    # The ε schedule relaxes to 1 on the last batch, so ABIT* recovers BIT*'s
    # admissible search and its cost sits close to the straight-line lower bound
    # even when the first batch was heavily inflated.
    grid = open_grid(12, 12, seed=5)
    start, goal = (0.5, 0.5), (11.0, 11.0)
    lower_bound = grid.distance(start, goal)
    result = ABITStar(_config(tmp_path, {**_BATCH, "inflation_factor": 100.0})).plan(
        grid, start, goal
    )
    assert result.success
    assert result.cost <= lower_bound * 1.15


def test_no_worse_than_plain_rrt(tmp_path: Path) -> None:
    # ABIT* is asymptotically optimal; a plain RRT returns the first feasible path.
    # On the same scenario+budget ABIT*'s path must be no longer than RRT's.
    grid = open_grid(12, 12, seed=11)
    start, goal = (0.5, 0.5), (11.0, 11.0)
    rrt = RRT(config("rrt")).plan(grid, start, goal)
    abit = ABITStar(_config(tmp_path, _BATCH)).plan(grid, start, goal)
    assert rrt.success and abit.success
    assert abit.cost <= rrt.cost + 1e-6


def test_anytime_under_heavy_inflation(tmp_path: Path) -> None:
    # Anytime property: with a large ε_infl held across a single batch (no
    # de-inflation), the greedy-toward-goal ordering still returns a feasible,
    # collision-free path — inflation only reorders work, never blocks a solution.
    grid = open_grid(12, 12, seed=9)
    start, goal = (0.5, 0.5), (11.0, 11.0)
    cfg = {"batch_size": 300, "max_batches": 1, "inflation_factor": 1000.0}
    result = ABITStar(_config(tmp_path, cfg)).plan(grid, start, goal)
    assert result.success
    assert result.path[-1] == pytest.approx(goal)
    assert _motion_valid_path(grid, result.path)


def test_bad_default_param_throws(tmp_path: Path) -> None:
    doc = yaml.safe_load((CONFIG_DIR / "abit_star.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] == "inflation_factor":
            entry["default"] = 0.5  # below declared min 1.0
    out = tmp_path / "abit_star.yaml"
    out.write_text(yaml.safe_dump(doc), encoding="utf-8")
    with pytest.raises(ParamError):
        ParamSet.from_yaml(out)
