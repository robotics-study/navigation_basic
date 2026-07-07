"""ARA*: converges to the A* optimum (eps -> 1), the no-path case, param
validation, and the defining anytime property (each solution no worse than the
next, first >= last)."""

from __future__ import annotations

import io
import json
from pathlib import Path

import pytest
from conftest import REPO_ROOT, config, grid_from, write_config

from navigation.core.params import ParamError, ParamSet
from navigation.core.trace import TraceRecorder
from navigation.core.types import Cell
from navigation.global_planning import AStar
from navigation.global_planning.search import ARAStar
from navigation.maps.loader import load_map, load_scenario

# wastar_greedy01: an open field with two obstacles that lie between start and goal,
# crafted so an inflated heuristic rounds them on the costly side first (see the A*
# counterexample). With eps 3 -> 1 ARA* returns a suboptimal path, then repairs to
# the optimum — an unambiguous anytime demonstration.
_WASTAR_ROWS = [
    ".............",
    "..........#..",
    "...........#.",
    ".............",
    ".............",
    ".............",
    ".............",
    ".............",
    ".............",
    ".............",
    ".............",
    ".............",
    ".............",
    ".............",
]
_WASTAR_START: Cell = (11, 12)
_WASTAR_GOAL: Cell = (0, 11)


def _path_cost(grid: object, path: list[Cell]) -> float:
    total = 0.0
    for a, b in zip(path, path[1:], strict=False):
        for cell, cost in grid.neighbors(a):  # type: ignore[attr-defined]
            if cell == b:
                total += cost
                break
    return total


def _anytime_config(tmp_path: Path, eps_start: float, eps_step: float) -> ParamSet:
    return ParamSet.from_yaml(
        write_config(
            tmp_path / "ara_any.yaml",
            "ara_star",
            [
                {"name": "eps_start", "type": "float", "default": eps_start,
                 "min": 1.0, "max": 10.0, "description": "start eps"},
                {"name": "eps_final", "type": "float", "default": 1.0,
                 "min": 1.0, "max": 10.0, "description": "final eps"},
                {"name": "eps_step", "type": "float", "default": eps_step,
                 "min": 0.01, "max": 10.0, "description": "eps decrement"},
                {"name": "max_expansions", "type": "int", "default": 100000,
                 "min": 1, "max": 100000000, "description": "cap"},
            ],
        )
    )


def test_ara_star_converges_to_astar_optimum_on_maze01() -> None:
    # eps -> eps_final (1.0) makes ARA*'s final solution the admissible A* optimum.
    grid = load_map(str(REPO_ROOT / "maps/grid/maze01.yaml"), seed=0, connectivity=8)
    sc = load_scenario(str(REPO_ROOT / "maps/scenarios/maze01_s1.yaml"))
    start = grid.world_to_cell(*sc.start)
    goal = grid.world_to_cell(*sc.goal)
    ara = ARAStar(config("ara_star")).plan(grid, start, goal)
    ast = AStar(config("astar")).plan(grid, start, goal)
    assert ara.success and ast.success
    assert ara.path[0] == start and ara.path[-1] == goal
    assert ara.cost == pytest.approx(ast.cost)  # final ARA* == A* optimum


def test_ara_star_no_path_when_walled_off() -> None:
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."])
    res = ARAStar(config("ara_star")).plan(grid, (0, 0), (0, 4))
    assert not res.success
    assert res.path == []
    assert res.cost == 0.0


def test_ara_star_rejects_out_of_range_param(tmp_path: Path) -> None:
    # eps_start declares min 1.0; a below-min default must fail at load time so an
    # inflation factor < 1 (which would break the suboptimality bound) never runs.
    cfg = write_config(
        tmp_path / "bad_ara.yaml",
        "ara_star",
        [{"name": "eps_start", "type": "float", "default": 0.5,
          "min": 1.0, "max": 10.0, "description": "below min"}],
    )
    with pytest.raises(ParamError):
        ParamSet.from_yaml(cfg)


def test_ara_star_anytime_solutions_improve_monotonically(tmp_path: Path) -> None:
    grid = grid_from(_WASTAR_ROWS)
    cfg = _anytime_config(tmp_path, eps_start=3.0, eps_step=1.0)
    buf = io.StringIO()
    res = ARAStar(cfg).plan(grid, _WASTAR_START, _WASTAR_GOAL, TraceRecorder(buf))

    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    solutions = [[tuple(s) for s in e["path"]] for e in events if e["event"] == "path_found"]
    costs = [_path_cost(grid, p) for p in solutions]

    assert len(costs) >= 2  # genuinely anytime: at least one repair after the first
    assert costs == sorted(costs, reverse=True)  # each solution no worse than the next
    assert costs[0] > costs[-1]  # the first eps-inflated path is strictly suboptimal here
    ast = AStar(config("astar")).plan(grid, _WASTAR_START, _WASTAR_GOAL)
    assert res.cost == pytest.approx(costs[-1])
    assert res.cost == pytest.approx(ast.cost)  # converged to the optimum
