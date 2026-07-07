"""Lazy Theta*: any-angle validity, cost-parity with Theta*, no-path, param check."""

from __future__ import annotations

import math
from pathlib import Path

import pytest
from conftest import config, grid_from, open_grid, write_config

from navigation.core.params import ParamError, ParamSet
from navigation.core.types import Cell
from navigation.global_planning.search.lazy_theta_star import LazyThetaStar
from navigation.global_planning.search.theta_star import ThetaStar


def _path_los_clear(grid: object, path: list[Cell]) -> bool:
    # Every reconstructed edge must be a legal straight move — validate via
    # line_of_sight, not neighbor adjacency, since any-angle paths are sparse.
    for a, b in zip(path, path[1:], strict=False):
        if not grid.line_of_sight(a, b):  # type: ignore[attr-defined]
            return False
    return True


def test_lazy_theta_star_takes_any_angle_shortcut() -> None:
    # (a) any-angle path on a known grid: goal directly visible from start, so the
    # deferred check confirms the optimistic straight segment (cost = Euclidean).
    # Lazy Theta* must return the identical cost Theta* does on this instance.
    grid = open_grid(3, 3)
    start, goal = (2, 0), (0, 1)
    res = LazyThetaStar(config("lazy_theta_star")).plan(grid, start, goal)
    assert res.success
    assert res.path[0] == start and res.path[-1] == goal
    assert res.cost == pytest.approx(math.hypot(2, 1))
    assert _path_los_clear(grid, res.path)
    theta = ThetaStar(config("theta_star")).plan(grid, start, goal)
    assert res.cost == pytest.approx(theta.cost)


def test_lazy_theta_star_bends_around_obstacle_matching_theta() -> None:
    # (a) the lazy-repair path: a blocker hides the goal, so the optimistic parent of
    # some vertex fails its deferred line-of-sight check and is repaired to a grid
    # neighbour. Every leg stays LOS-clear and the cost matches eager Theta*.
    grid = grid_from([".....", ".....", "..#..", "..#..", "....."])
    start, goal = (4, 0), (0, 4)
    res = LazyThetaStar(config("lazy_theta_star")).plan(grid, start, goal)
    assert res.success
    assert res.path[0] == start and res.path[-1] == goal
    assert len(res.path) >= 3  # bends -> at least one interior waypoint
    assert not grid.line_of_sight(start, goal)  # goal genuinely hidden
    assert _path_los_clear(grid, res.path)
    theta = ThetaStar(config("theta_star")).plan(grid, start, goal)
    assert res.cost == pytest.approx(theta.cost)


def test_lazy_theta_star_no_path_when_walled_off() -> None:
    # (b) a fully occupied middle column separates start from goal.
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."])
    res = LazyThetaStar(config("lazy_theta_star")).plan(grid, (0, 0), (0, 4))
    assert not res.success
    assert res.path == []
    assert res.cost == 0.0


def test_lazy_theta_star_rejects_out_of_range_weight(tmp_path: Path) -> None:
    # (c) lazy_theta_star declares heuristic_weight >= 1.0; a below-min default must
    # fail at load time, so an invalid weight can never reach plan().
    cfg = write_config(
        tmp_path / "bad_lazy_theta.yaml",
        "lazy_theta_star",
        [{"name": "heuristic_weight", "type": "float", "default": 0.5,
          "min": 1.0, "max": 5.0, "description": "below min"}],
    )
    with pytest.raises(ParamError):
        ParamSet.from_yaml(cfg)
