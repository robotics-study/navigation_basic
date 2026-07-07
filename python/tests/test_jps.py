"""JPS: same optimum as 8-connected A* with fewer expansions, and the no-path /
param-validation cases."""

from __future__ import annotations

import math
from pathlib import Path

import pytest
from conftest import config, grid_from, open_grid, write_config

from navigation.core.params import ParamError, ParamSet
from navigation.core.types import Cell
from navigation.global_planning import AStar

# JPS is not re-exported through the package facade in this change; import it directly.
from navigation.global_planning.search.jps import JPS


def _path_connected(grid: object, path: list[Cell]) -> bool:
    # Every reported cell is a legal 8-connected move from the previous — proves the
    # interpolated staircase never cuts a corner the grid forbids.
    for a, b in zip(path, path[1:], strict=False):
        succ = {cell for cell, _ in grid.neighbors(a)}  # type: ignore[attr-defined]
        if b not in succ:
            return False
    return True


def test_jps_matches_astar_optimum_with_fewer_expansions() -> None:
    grid = open_grid(9, 9)
    start, goal = (8, 0), (0, 8)
    ast = AStar(config("astar")).plan(grid, start, goal)
    jps = JPS(config("jps")).plan(grid, start, goal)
    assert ast.success and jps.success
    assert jps.cost == pytest.approx(ast.cost)  # JPS returns the 8-connected optimum
    assert jps.cost == pytest.approx(8 * math.sqrt(2))
    assert jps.path[0] == start and jps.path[-1] == goal
    assert _path_connected(grid, jps.path)  # corner-cut-free interpolated staircase
    assert jps.stats.expanded_nodes < ast.stats.expanded_nodes  # symmetry pruning pays off


def test_jps_bends_optimally_around_obstacle() -> None:
    # A wall with a single bottom gap: JPS must match A*'s grid optimum, not the
    # straight-line lower bound, and every reported step must be legal.
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "....."])
    start, goal = (0, 0), (0, 4)
    ast = AStar(config("astar")).plan(grid, start, goal)
    jps = JPS(config("jps")).plan(grid, start, goal)
    assert jps.success
    assert jps.cost == pytest.approx(ast.cost)
    assert jps.path[0] == start and jps.path[-1] == goal
    assert _path_connected(grid, jps.path)
    assert jps.cost > math.hypot(start[0] - goal[0], start[1] - goal[1])  # genuine detour


def test_jps_no_path_when_walled_off() -> None:
    # A fully occupied middle column separates start from goal.
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."])
    res = JPS(config("jps")).plan(grid, (0, 0), (0, 4))
    assert not res.success
    assert res.path == []
    assert res.cost == 0.0


def test_jps_rejects_out_of_range_param(tmp_path: Path) -> None:
    # A jps config carrying an out-of-range default must fail at load time (the param
    # validation contract), so a bad value can never reach plan().
    cfg = write_config(
        tmp_path / "bad_jps.yaml",
        "jps",
        [{"name": "heuristic_weight", "type": "float", "default": 0.5,
          "min": 1.0, "max": 5.0, "description": "below min"}],
    )
    with pytest.raises(ParamError):
        ParamSet.from_yaml(cfg)
