"""Discrete planners: optimality / validity, and the no-path case."""

from __future__ import annotations

import math

import pytest
from conftest import config, grid_from, open_grid

from navigation.core.types import Cell
from navigation.global_planning import BFS, AStar, Dijkstra

_OPTIMAL_DIAG = 4 * math.sqrt(2)  # (4,0)->(0,4) on an open 8-connected grid


def _path_is_connected(grid: object, path: list[Cell]) -> bool:
    for a, b in zip(path, path[1:], strict=False):
        succ = {cell for cell, _ in grid.neighbors(a)}  # type: ignore[attr-defined]
        if b not in succ:
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
    ):
        res = planner.plan(grid, (0, 0), (0, 4))
        assert not res.success
        assert res.path == []
        assert res.cost == 0.0
