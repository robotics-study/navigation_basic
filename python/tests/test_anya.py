"""Anya — optimal any-angle: matches the true shortest any-angle cost, dominates
Theta*, and the no-path / parameter-validation contracts."""

from __future__ import annotations

import heapq
import math
from pathlib import Path

import pytest
from conftest import config, grid_from, open_grid, write_config

from navigation.core.params import ParamError, ParamSet
from navigation.core.types import Cell
from navigation.global_planning.search.anya import Anya
from navigation.global_planning.search.astar import AStar
from navigation.global_planning.search.theta_star import ThetaStar


def _euclid(a: Cell, b: Cell) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _path_los_clear(grid: object, path: list[Cell]) -> bool:
    # Every reconstructed any-angle leg must be a legal straight move.
    for a, b in zip(path, path[1:], strict=False):
        if not grid.line_of_sight(a, b):  # type: ignore[attr-defined]
            return False
    return True


def _visibility_optimum(grid: object, start: Cell, goal: Cell) -> float:
    """Independent ground truth: the Euclidean-shortest any-angle cost over
    cell-centre vertices = Dijkstra on the full visibility graph (every pair of
    mutually LOS-visible reachable free cells, weighted by straight-line length).
    A different, obviously-correct computation than Anya's interval search, so
    agreement is a real optimality check, not a restatement of the planner."""
    seen: set[Cell] = {start}
    stack = [start]
    while stack:  # reachable free component via 8-connected neighbors
        cur = stack.pop()
        for nb, _c in grid.neighbors(cur):  # type: ignore[attr-defined]
            if nb not in seen:
                seen.add(nb)
                stack.append(nb)
    cells = list(seen)
    dist: dict[Cell, float] = {start: 0.0}
    pq: list[tuple[float, Cell]] = [(0.0, start)]
    settled: set[Cell] = set()
    while pq:
        d, u = heapq.heappop(pq)
        if u in settled:
            continue
        settled.add(u)
        if u == goal:
            return d
        for v in cells:
            if v in settled or v == u:
                continue
            if grid.line_of_sight(u, v):  # type: ignore[attr-defined]
                nd = d + _euclid(u, v)
                if v not in dist or nd < dist[v]:
                    dist[v] = nd
                    heapq.heappush(pq, (nd, v))
    return math.inf


# (a) optimal any-angle path -------------------------------------------------

def test_anya_returns_optimal_straight_line() -> None:
    # Goal directly visible from start on an open grid: the global any-angle
    # optimum is the single straight segment, cost = Euclidean. Anya must return
    # exactly it, no longer than Theta* and strictly shorter than grid A*.
    grid = open_grid(3, 3)
    start, goal = (2, 0), (0, 1)
    res = Anya(config("anya")).plan(grid, start, goal)
    assert res.success
    assert res.path[0] == start and res.path[-1] == goal
    assert res.cost == pytest.approx(math.sqrt(5))  # analytically-known shortest
    assert _path_los_clear(grid, res.path)
    theta = ThetaStar(config("theta_star")).plan(grid, start, goal)
    astar = AStar(config("astar")).plan(grid, start, goal)
    assert res.cost <= theta.cost + 1e-9
    assert res.cost < astar.cost


def test_anya_matches_true_optimum_around_obstacle() -> None:
    # A blocker hides the goal, forcing a bend. Anya's cost must equal the
    # visibility-graph optimum (true shortest any-angle cost) and never exceed
    # Theta*, which only relaxes toward its grandparent and can be suboptimal.
    grid = grid_from([".....", ".....", "..#..", "..#..", "....."])
    start, goal = (4, 0), (0, 4)
    res = Anya(config("anya")).plan(grid, start, goal)
    assert res.success
    assert res.path[0] == start and res.path[-1] == goal
    assert _path_los_clear(grid, res.path)
    assert not grid.line_of_sight(start, goal)  # goal genuinely hidden
    optimum = _visibility_optimum(grid, start, goal)
    assert res.cost == pytest.approx(optimum)
    theta = ThetaStar(config("theta_star")).plan(grid, start, goal)
    assert res.cost <= theta.cost + 1e-9


def test_anya_matches_true_optimum_on_asymmetric_map() -> None:
    # Second, differently-shaped instance: guards against a projection that is
    # optimal only on the symmetric bend above.
    grid = grid_from([
        "........",
        "...##...",
        "...##...",
        ".....#..",
        "..#.....",
        "........",
    ])
    start, goal = (5, 0), (0, 7)
    res = Anya(config("anya")).plan(grid, start, goal)
    assert res.success
    assert _path_los_clear(grid, res.path)
    assert res.cost == pytest.approx(_visibility_optimum(grid, start, goal))
    theta = ThetaStar(config("theta_star")).plan(grid, start, goal)
    assert res.cost <= theta.cost + 1e-9


# (b) no-path case -----------------------------------------------------------

def test_anya_no_path_when_walled_off() -> None:
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."])
    res = Anya(config("anya")).plan(grid, (0, 0), (0, 4))
    assert not res.success
    assert res.path == []
    assert res.cost == 0.0


# (c) parameter validation failure -------------------------------------------

def test_anya_rejects_out_of_range_weight(tmp_path: Path) -> None:
    # anya declares heuristic_weight >= 1.0; a below-min default must fail at load
    # time, so an invalid weight can never reach plan().
    cfg = write_config(
        tmp_path / "bad_anya.yaml",
        "anya",
        [{"name": "heuristic_weight", "type": "float", "default": 0.5,
          "min": 1.0, "max": 5.0, "description": "below min"}],
    )
    with pytest.raises(ParamError):
        ParamSet.from_yaml(cfg)
