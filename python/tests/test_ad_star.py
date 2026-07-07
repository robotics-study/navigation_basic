"""AD* (Anytime Dynamic A*): the executed trajectory is belief-optimal like D*
Lite (equals A* when the freespace assumption doesn't mislead), replans a valid
detour when an obstacle is revealed, handles the no-path case, validates params,
and — combining ARA*'s anytime repair — emits a strictly suboptimal first
solution that improves to the optimum as ε -> 1."""

from __future__ import annotations

import io
import json
from pathlib import Path

import pytest
from conftest import REPO_ROOT, config, grid_from, write_config

from navigation.core.params import ParamError, ParamSet
from navigation.core.trace import TraceRecorder
from navigation.core.types import Cell
from navigation.global_planning.search.ad_star import ADStar
from navigation.global_planning.search.astar import AStar
from navigation.maps.loader import load_map, load_scenario
from navigation.maps.occupancy_grid import OccupancyGrid2D

# Same field as the ARA* anytime fixture but with start/goal swapped so the two
# obstacles sit near the *robot*: AD*'s backward (goal->start) search rushes toward
# the robot under the inflated key and rounds them the costly way first, then
# repairs to the optimum. (ARA*'s forward search wanted them near the goal instead.)
_FIELD_ROWS = [
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
_FIELD_START: Cell = (0, 11)
_FIELD_GOAL: Cell = (11, 12)


def _path_cost(grid: OccupancyGrid2D, path: list[Cell]) -> float:
    total = 0.0
    for a, b in zip(path, path[1:], strict=False):
        for cell, cost in grid.neighbors(a):
            if cell == b:
                total += cost
                break
    return total


def _is_valid_trajectory(grid: OccupancyGrid2D, path: list[Cell]) -> bool:
    # Every consecutive pair is a legal move under GROUND TRUTH (grid.neighbors),
    # proving the executed trajectory never crosses a real obstacle.
    for a, b in zip(path, path[1:], strict=False):
        if not any(cell == b for cell, _ in grid.neighbors(a)):
            return False
    return True


def _anytime_config(
    tmp_path: Path, eps_start: float, eps_step: float, sensor_radius: int
) -> ParamSet:
    return ParamSet.from_yaml(
        write_config(
            tmp_path / "ad_any.yaml",
            "ad_star",
            [
                {"name": "eps_start", "type": "float", "default": eps_start,
                 "min": 1.0, "max": 10.0, "description": "start eps"},
                {"name": "eps_final", "type": "float", "default": 1.0,
                 "min": 1.0, "max": 10.0, "description": "final eps"},
                {"name": "eps_step", "type": "float", "default": eps_step,
                 "min": 0.01, "max": 10.0, "description": "eps decrement"},
                {"name": "sensor_radius", "type": "int", "default": sensor_radius,
                 "min": 1, "max": 50, "description": "sensor radius"},
                {"name": "max_expansions", "type": "int", "default": 100000,
                 "min": 1, "max": 100000000, "description": "cap"},
            ],
        )
    )


def test_ad_star_static_trajectory_matches_astar_optimum_on_maze01() -> None:
    # eps_final=1 makes each belief-optimal step follow the true shortest path; on
    # maze01 the freespace assumption never pushes the robot off it, so the executed
    # trajectory equals the omniscient A* optimum (the D* Lite guarantee under AD*).
    grid = load_map(str(REPO_ROOT / "maps/grid/maze01.yaml"), seed=0, connectivity=8)
    sc = load_scenario(str(REPO_ROOT / "maps/scenarios/maze01_s1.yaml"))
    start = grid.world_to_cell(*sc.start)
    goal = grid.world_to_cell(*sc.goal)
    res = ADStar(config("ad_star")).plan(grid, start, goal)
    ast = AStar(config("astar")).plan(grid, start, goal)
    assert res.success and ast.success
    assert res.path[0] == start and res.path[-1] == goal
    assert _is_valid_trajectory(grid, res.path)
    assert res.cost == pytest.approx(ast.cost)


def test_ad_star_replans_valid_detour_when_trap_revealed() -> None:
    # dstar_trap01 is a C-shaped trap whose mouth faces the robot. Assuming freespace,
    # AD* heads straight in, senses the back wall, and must replan out and around. The
    # executed trajectory stays collision-free in ground truth, reaches the goal, and —
    # because the trap forced a real detour — is strictly longer than the omniscient
    # A* optimum. At least one replan must have fired.
    grid = load_map(str(REPO_ROOT / "maps/grid/dstar_trap01.yaml"), seed=0, connectivity=8)
    sc = load_scenario(str(REPO_ROOT / "maps/scenarios/dstar_trap01_s1.yaml"))
    start = grid.world_to_cell(*sc.start)
    goal = grid.world_to_cell(*sc.goal)
    res = ADStar(config("ad_star")).plan(grid, start, goal)
    ast = AStar(config("astar")).plan(grid, start, goal)
    assert res.success
    assert res.path[0] == start and res.path[-1] == goal
    assert _is_valid_trajectory(grid, res.path)
    assert res.stats.iterations >= 1  # replanned at least once after sensing the trap
    assert res.cost > ast.cost + 1e-9  # the detour is genuinely longer than omniscient A*


def test_ad_star_no_path_when_walled_off() -> None:
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."])
    res = ADStar(config("ad_star")).plan(grid, (0, 0), (0, 4))
    assert not res.success
    assert res.path == []
    assert res.cost == 0.0


def test_ad_star_rejects_out_of_range_param(tmp_path: Path) -> None:
    # eps_start declares min 1.0; a below-min default must fail at load time so an
    # inflation factor < 1 (which would break the suboptimality bound) never runs.
    cfg = write_config(
        tmp_path / "bad_ad.yaml",
        "ad_star",
        [{"name": "eps_start", "type": "float", "default": 0.5,
          "min": 1.0, "max": 10.0, "description": "below min"}],
    )
    with pytest.raises(ParamError):
        ParamSet.from_yaml(cfg)


def test_ad_star_anytime_solutions_improve_to_optimum(tmp_path: Path) -> None:
    grid = grid_from(_FIELD_ROWS)
    # Large sensor radius: the whole field is sensed from the start, so belief == truth
    # and no motion-time changes muddy the ε sweep — the anytime improvement is
    # isolated to the initial position, exactly like a static ARA* sweep.
    cfg = _anytime_config(tmp_path, eps_start=4.0, eps_step=0.5, sensor_radius=50)
    buf = io.StringIO()
    res = ADStar(cfg).plan(grid, _FIELD_START, _FIELD_GOAL, TraceRecorder(buf))

    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    # Collect the plans published while the robot is still at the start (before it
    # leaves on its first move, i.e. before the 2nd robot_moved event).
    moves = 0
    start_phase: list[list[Cell]] = []
    for e in events:
        if e["event"] == "robot_moved":
            moves += 1
        if e["event"] == "path_found" and moves <= 1:
            start_phase.append([tuple(s) for s in e["path"]])
    costs = [_path_cost(grid, p) for p in start_phase]

    assert len(costs) >= 2  # genuinely anytime: at least one repair after the first
    assert costs == sorted(costs, reverse=True)  # each solution no worse than the next
    assert costs[0] > costs[-1]  # the first ε-inflated plan is strictly suboptimal here
    ast = AStar(config("astar")).plan(grid, _FIELD_START, _FIELD_GOAL)
    assert costs[-1] == pytest.approx(ast.cost)  # final ε -> 1 plan is the optimum
    assert res.cost == pytest.approx(ast.cost)  # executed trajectory is optimal too
