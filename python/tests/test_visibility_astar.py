"""Visibility A* — matches the cell-centre visibility-graph shortest cost, never
exceeds Theta*, and the no-path / parameter-validation contracts."""

from __future__ import annotations

import heapq
import io
import json
import math
from pathlib import Path

import pytest
from conftest import config, grid_from, open_grid, write_config

from navigation.core.params import ParamError, ParamSet
from navigation.core.trace import TraceRecorder
from navigation.core.types import Cell
from navigation.global_planning.search.astar import AStar
from navigation.global_planning.search.theta_star import ThetaStar
from navigation.global_planning.search.visibility_astar import VisibilityAStarPlanner


def _euclid(a: Cell, b: Cell) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _path_los_clear(grid: object, path: list[Cell]) -> bool:
    # Every reconstructed any-angle leg must be a legal straight move.
    for a, b in zip(path, path[1:], strict=False):
        if not grid.line_of_sight(a, b):  # type: ignore[attr-defined]
            return False
    return True


def _visibility_optimum(grid: object, start: Cell, goal: Cell) -> float:
    """Independent ground truth: the shortest path over the cell-centre visibility
    graph = Dijkstra over every pair of mutually LOS-visible reachable free cells,
    weighted by straight-line length. This is what Visibility A* optimises — a
    cell-centre approximation of the true any-angle optimum, since turns are pinned
    to cell centres rather than obstacle corners. Computed differently from the
    planner's interval search, so agreement is a real correctness check, not a
    restatement of the planner."""
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


# (a) valid any-angle path ---------------------------------------------------

def test_visibility_astar_returns_straight_line_when_visible() -> None:
    # Goal directly visible from start on an open grid: the cell-centre visibility
    # optimum is the single straight segment, cost = Euclidean. Visibility A* must
    # return exactly it, no longer than Theta* and strictly shorter than grid A*.
    grid = open_grid(3, 3)
    start, goal = (2, 0), (0, 1)
    res = VisibilityAStarPlanner(config("visibility_astar")).plan(grid, start, goal)
    assert res.success
    assert res.path[0] == start and res.path[-1] == goal
    assert res.cost == pytest.approx(math.sqrt(5))  # analytically-known straight leg
    assert _path_los_clear(grid, res.path)
    theta = ThetaStar(config("theta_star")).plan(grid, start, goal)
    astar = AStar(config("astar")).plan(grid, start, goal)
    assert res.cost <= theta.cost + 1e-9
    assert res.cost < astar.cost


def test_visibility_astar_matches_visibility_optimum_around_obstacle() -> None:
    # A blocker hides the goal, forcing a bend. The cost must equal the cell-centre
    # visibility-graph optimum and never exceed Theta*, which only relaxes toward
    # its grandparent and can leave the string slightly slack.
    grid = grid_from([".....", ".....", "..#..", "..#..", "....."])
    start, goal = (4, 0), (0, 4)
    res = VisibilityAStarPlanner(config("visibility_astar")).plan(grid, start, goal)
    assert res.success
    assert res.path[0] == start and res.path[-1] == goal
    assert _path_los_clear(grid, res.path)
    assert not grid.line_of_sight(start, goal)  # goal genuinely hidden
    optimum = _visibility_optimum(grid, start, goal)
    assert res.cost == pytest.approx(optimum)
    theta = ThetaStar(config("theta_star")).plan(grid, start, goal)
    assert res.cost <= theta.cost + 1e-9


def test_visibility_astar_matches_visibility_optimum_on_asymmetric_map() -> None:
    # Second, differently-shaped instance: guards against a projection that is
    # correct only on the symmetric bend above.
    grid = grid_from([
        "........",
        "...##...",
        "...##...",
        ".....#..",
        "..#.....",
        "........",
    ])
    start, goal = (5, 0), (0, 7)
    res = VisibilityAStarPlanner(config("visibility_astar")).plan(grid, start, goal)
    assert res.success
    assert _path_los_clear(grid, res.path)
    assert res.cost == pytest.approx(_visibility_optimum(grid, start, goal))
    theta = ThetaStar(config("theta_star")).plan(grid, start, goal)
    assert res.cost <= theta.cost + 1e-9


# trace: (root, interval) info surfaced via the event data field ------------

def test_visibility_astar_emits_root_interval_data() -> None:
    buf = io.StringIO()
    VisibilityAStarPlanner(config("visibility_astar")).plan(
        open_grid(6, 6), (0, 0), (5, 5), TraceRecorder(buf)
    )
    intervals = [
        json.loads(line)["data"]
        for line in buf.getvalue().splitlines()
        if json.loads(line)["event"] == "edge_added"
    ]
    assert intervals, "must tag any-angle edges with their (root, interval) node"
    for d in intervals:
        assert d.keys() == {"row", "col_lo", "col_hi"}
        assert d["col_lo"] <= d["col_hi"]  # a real column run


# (b) no-path case -----------------------------------------------------------

def test_visibility_astar_no_path_when_walled_off() -> None:
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."])
    res = VisibilityAStarPlanner(config("visibility_astar")).plan(grid, (0, 0), (0, 4))
    assert not res.success
    assert res.path == []
    assert res.cost == 0.0


# (c) parameter validation failure -------------------------------------------

def test_visibility_astar_rejects_out_of_range_weight(tmp_path: Path) -> None:
    # visibility_astar declares heuristic_weight >= 1.0; a below-min default must
    # fail at load time, so an invalid weight can never reach plan().
    cfg = write_config(
        tmp_path / "bad_visibility_astar.yaml",
        "visibility_astar",
        [{"name": "heuristic_weight", "type": "float", "default": 0.5,
          "min": 1.0, "max": 5.0, "description": "below min"}],
    )
    with pytest.raises(ParamError):
        ParamSet.from_yaml(cfg)
