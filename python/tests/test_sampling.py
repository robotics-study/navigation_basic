"""Sampling planners: reach the goal on open space, fail when walled off."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml
from conftest import CONFIG_DIR, config, grid_from, open_grid

from navigation.core.params import ParamSet
from navigation.global_planning import PRM, RRT, BITStar, FastRRT, FMTStar, PRMStar, RRTStar
from navigation.maps.occupancy_grid import OccupancyGrid2D

_PLANNERS = [("rrt", RRT), ("rrt_star", RRTStar), ("fast_rrt", FastRRT)]

# Roadmap / batch planners: keyed to a small budget that still reaches the goal
# on the 10x10 open grid, so the suite stays fast.
_BATCH_PLANNERS = [
    ("prm", PRM), ("prm_star", PRMStar), ("fmt_star", FMTStar), ("bit_star", BITStar),
]
_BATCH_BUDGET: dict[str, dict[str, int]] = {
    "prm": {"num_samples": 600},
    "prm_star": {"num_samples": 600},
    "fmt_star": {"num_samples": 600},
    "bit_star": {"batch_size": 150, "max_batches": 4},
}


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


def _batch_config(tmp_path: Path, algo: str) -> ParamSet:
    """Real config with reduced sample/batch budgets so batch planners stay fast."""
    doc = yaml.safe_load((CONFIG_DIR / f"{algo}.yaml").read_text())
    overrides = _BATCH_BUDGET[algo]
    for entry in doc["params"]:
        if entry["name"] in overrides:
            entry["default"] = overrides[entry["name"]]
    out = tmp_path / f"{algo}.yaml"
    out.write_text(yaml.safe_dump(doc), encoding="utf-8")
    return ParamSet.from_yaml(out)


@pytest.mark.parametrize("algo,cls", _BATCH_PLANNERS)
def test_batch_reaches_goal_on_open_grid(algo: str, cls: type, tmp_path: Path) -> None:
    grid = open_grid(10, 10, seed=7)
    start, goal = (0.5, 0.5), (9.0, 9.0)
    result = cls(_batch_config(tmp_path, algo)).plan(grid, start, goal)
    assert result.success
    # start/goal are exact roadmap/tree nodes, so the path pins to them exactly.
    assert result.path[0] == pytest.approx(start)
    assert result.path[-1] == pytest.approx(goal)
    assert _motion_valid_path(grid, result.path)
    assert result.cost > 0.0


@pytest.mark.parametrize("algo,cls", _BATCH_PLANNERS)
def test_batch_no_path_when_walled_off(algo: str, cls: type, tmp_path: Path) -> None:
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."], seed=3)
    result = cls(_batch_config(tmp_path, algo)).plan(grid, (0.5, 0.5), (4.5, 4.5))
    assert not result.success
    assert result.path == []
    assert result.cost == 0.0


@pytest.mark.parametrize("algo,cls", [("prm_star", PRMStar), ("fmt_star", FMTStar),
                                      ("bit_star", BITStar)])
def test_optimal_family_near_straight_line(algo: str, cls: type, tmp_path: Path) -> None:
    # The asymptotically optimal roadmap/tree planners should return a cost close
    # to the straight-line lower bound on an obstacle-free grid.
    grid = open_grid(12, 12, seed=5)
    start, goal = (0.5, 0.5), (11.0, 11.0)
    lower_bound = grid.distance(start, goal)
    result = cls(_batch_config(tmp_path, algo)).plan(grid, start, goal)
    assert result.success
    assert result.cost <= lower_bound * 1.15


def test_fast_rrt_shortcut_not_longer_than_rrt(tmp_path: Path) -> None:
    # Fast-Optimal pruning should keep Fast-RRT's path at least as short as plain RRT.
    grid = open_grid(12, 12, seed=11)
    start, goal = (0.5, 0.5), (11.0, 11.0)
    rrt = RRT(config("rrt")).plan(grid, start, goal)
    fast = FastRRT(_capped_config(tmp_path, "fast_rrt", max_iter=2000)).plan(grid, start, goal)
    assert rrt.success and fast.success
    # Straight-line lower bound; pruned path should be close to it and <= RRT's.
    assert fast.cost <= rrt.cost + 1e-6
