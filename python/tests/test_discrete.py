"""Discrete planners: optimality / validity, and the no-path case."""

from __future__ import annotations

import math
from pathlib import Path

import pytest
from conftest import config, grid_from, open_grid, write_config

from navigation.core.params import ParamError, ParamSet
from navigation.core.types import Cell
from navigation.global_planning import BFS, AStar, Dijkstra, ThetaStar

_OPTIMAL_DIAG = 4 * math.sqrt(2)  # (4,0)->(0,4) on an open 8-connected grid


def _path_is_connected(grid: object, path: list[Cell]) -> bool:
    for a, b in zip(path, path[1:], strict=False):
        succ = {cell for cell, _ in grid.neighbors(a)}  # type: ignore[attr-defined]
        if b not in succ:
            return False
    return True


def _path_los_clear(grid: object, path: list[Cell]) -> bool:
    # Every reconstructed Theta* edge must be a legal straight move — validate via
    # line_of_sight, not neighbor adjacency, since any-angle paths are sparse.
    for a, b in zip(path, path[1:], strict=False):
        if not grid.line_of_sight(a, b):  # type: ignore[attr-defined]
            return False
    return True


def test_dijkstra_and_astar_are_optimal() -> None:
    grid = open_grid(5, 5)
    dj = Dijkstra(config("dijkstra")).plan(grid, (4, 0), (0, 4))
    ast = AStar(config("astar")).plan(grid, (4, 0), (0, 4))
    assert dj.success and ast.success
    assert dj.cost == pytest.approx(_OPTIMAL_DIAG)
    assert ast.cost == pytest.approx(_OPTIMAL_DIAG)
    assert ast.cost == pytest.approx(dj.cost)


def test_astar_expands_no_more_than_dijkstra() -> None:
    # The heuristic must not make A* explore more than uninformed Dijkstra here.
    grid = open_grid(9, 9)
    dj = Dijkstra(config("dijkstra")).plan(grid, (8, 0), (0, 8))
    ast = AStar(config("astar")).plan(grid, (8, 0), (0, 8))
    assert ast.stats.expanded_nodes <= dj.stats.expanded_nodes


def test_bfs_finds_fewest_edge_path() -> None:
    grid = open_grid(5, 5)
    res = BFS(config("bfs")).plan(grid, (4, 0), (0, 4))
    assert res.success
    # 4 diagonal moves -> 5 waypoints; BFS minimizes edge count.
    assert len(res.path) == 5
    assert _path_is_connected(grid, res.path)


def test_no_path_when_walled_off() -> None:
    # A fully occupied middle column separates start from goal.
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."])
    for planner in (
        BFS(config("bfs")),
        Dijkstra(config("dijkstra")),
        AStar(config("astar")),
        ThetaStar(config("theta_star")),
    ):
        res = planner.plan(grid, (0, 0), (0, 4))
        assert not res.success
        assert res.path == []
        assert res.cost == 0.0


def test_theta_star_takes_any_angle_shortcut() -> None:
    # Path 2: on an open grid the goal is directly visible from start, so Theta*
    # returns the single straight segment (cost = Euclidean), strictly shorter
    # than A*'s grid-locked octile path over the same non-diagonal offset.
    grid = open_grid(3, 3)
    start, goal = (2, 0), (0, 1)
    res = ThetaStar(config("theta_star")).plan(grid, start, goal)
    assert res.success
    assert res.path[0] == start and res.path[-1] == goal
    assert res.cost == pytest.approx(math.hypot(2, 1))
    assert _path_los_clear(grid, res.path)
    ast = AStar(config("astar")).plan(grid, start, goal)
    assert res.cost < ast.cost


def test_theta_star_bends_around_obstacle() -> None:
    # Path 1: a blocker hides the goal from start (no direct LOS), forcing a turn.
    # The any-angle path keeps an interior waypoint near the corner, every leg is
    # LOS-clear, and it still beats the grid-locked A* cost.
    grid = grid_from([".....", ".....", "..#..", "..#..", "....."])
    start, goal = (4, 0), (0, 4)
    res = ThetaStar(config("theta_star")).plan(grid, start, goal)
    assert res.success
    assert res.path[0] == start and res.path[-1] == goal
    assert len(res.path) >= 3  # bends -> at least one interior waypoint
    assert not grid.line_of_sight(start, goal)  # goal genuinely hidden
    assert _path_los_clear(grid, res.path)
    ast = AStar(config("astar")).plan(grid, start, goal)
    assert res.cost < ast.cost


def test_theta_star_rejects_out_of_range_weight(tmp_path: Path) -> None:
    # theta_star declares heuristic_weight >= 1.0; a below-min default must fail
    # at load time, so an invalid weight can never reach plan().
    cfg = write_config(
        tmp_path / "bad_theta.yaml",
        "theta_star",
        [{"name": "heuristic_weight", "type": "float", "default": 0.5,
          "min": 1.0, "max": 5.0, "description": "below min"}],
    )
    with pytest.raises(ParamError):
        ParamSet.from_yaml(cfg)
