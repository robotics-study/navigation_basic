"""Sampling planners: reach the goal on open space, fail when walled off."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml
from conftest import CONFIG_DIR, config, grid_from, open_grid

from nav_study.core.params import ParamSet
from nav_study.global_planning.fast_rrt import FastRRT
from nav_study.global_planning.rrt import RRT
from nav_study.global_planning.rrt_star import RRTStar
from nav_study.maps.occupancy_grid import OccupancyGrid2D

_PLANNERS = [("rrt", RRT), ("rrt_star", RRTStar), ("fast_rrt", FastRRT)]


def _capped_config(tmp_path: Path, algo: str, max_iter: int) -> ParamSet:
    """Real config with a smaller iteration budget so the no-path test stays fast."""
    doc = yaml.safe_load((CONFIG_DIR / f"{algo}.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] == "max_iterations":
            entry["default"] = max_iter
    out = tmp_path / f"{algo}.yaml"
    out.write_text(yaml.safe_dump(doc), encoding="utf-8")
    return ParamSet.from_yaml(out)


def _motion_valid_path(grid: OccupancyGrid2D, path: list[tuple[float, float]]) -> bool:
    return all(grid.is_motion_valid(a, b) for a, b in zip(path, path[1:], strict=False))


@pytest.mark.parametrize("algo,cls", _PLANNERS)
def test_reaches_goal_on_open_grid(algo: str, cls: type, tmp_path: Path) -> None:
    grid = open_grid(10, 10, seed=7)
    start, goal = (0.5, 0.5), (9.0, 9.0)
    # Anytime planners exhaust their budget; cap it so the test stays quick.
    params = _capped_config(tmp_path, algo, max_iter=2000)
    result = cls(params).plan(grid, start, goal)
    assert result.success
    assert result.path[0] == pytest.approx(start)
    assert grid.distance(result.path[-1], goal) <= params.get_float("goal_tolerance")
    assert _motion_valid_path(grid, result.path)
    assert result.cost > 0.0


@pytest.mark.parametrize("algo,cls", _PLANNERS)
def test_no_path_when_walled_off(algo: str, cls: type, tmp_path: Path) -> None:
    # Fully occupied middle column: no motion can cross it.
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."], seed=3)
    params = _capped_config(tmp_path, algo, max_iter=400)
    result = cls(params).plan(grid, (0.5, 0.5), (4.5, 4.5))
    assert not result.success
    assert result.path == []
    assert result.cost == 0.0


def test_fast_rrt_shortcut_not_longer_than_rrt(tmp_path: Path) -> None:
    # Fast-Optimal pruning should keep Fast-RRT's path at least as short as plain RRT.
    grid = open_grid(12, 12, seed=11)
    start, goal = (0.5, 0.5), (11.0, 11.0)
    rrt = RRT(config("rrt")).plan(grid, start, goal)
    fast = FastRRT(_capped_config(tmp_path, "fast_rrt", max_iter=2000)).plan(grid, start, goal)
    assert rrt.success and fast.success
    # Straight-line lower bound; pruned path should be close to it and <= RRT's.
    assert fast.cost <= rrt.cost + 1e-6
