"""Shared demo scaffolding: arg parsing + run flow for discrete/sampling planners.

Demos are assembly only — they wire params + map + scenario + planner and emit
`planning_started` (the one event the planner cannot emit, since the map path is
known only here). Everything else is emitted by the planner.
"""

from __future__ import annotations

import argparse
import json
from collections.abc import Callable

from nav_study.core.params import ParamSet
from nav_study.core.planner import GlobalPlanner
from nav_study.core.trace import open_trace
from nav_study.core.types import Cell, PlanResult, Point
from nav_study.maps.loader import load_map, load_scenario
from nav_study.maps.occupancy_grid import OccupancyGrid2D

PlannerFactory = Callable[[ParamSet], GlobalPlanner]


def _parse_args(name: str) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=f"nav_study {name} demo")
    parser.add_argument("--map", required=True, help="map yaml path")
    parser.add_argument("--scenario", required=True, help="scenario yaml path")
    parser.add_argument("--params", required=True, help="algorithm config yaml path")
    parser.add_argument("--trace", required=True, help="output trace jsonl path")
    parser.add_argument("--seed", type=int, default=0, help="RNG seed if config has none")
    parser.add_argument("--connectivity", type=int, default=8, choices=(4, 8))
    return parser.parse_args()


def _load(args: argparse.Namespace) -> tuple[ParamSet, OccupancyGrid2D, Point, Point]:
    params = ParamSet.from_yaml(args.params)
    seed = params.get_int("seed") if params.has("seed") else args.seed
    grid = load_map(args.map, seed=seed, connectivity=args.connectivity)
    assert isinstance(grid, OccupancyGrid2D)
    scenario = load_scenario(args.scenario)
    return params, grid, scenario.start, scenario.goal


def _report(name: str, result: PlanResult) -> None:
    summary = {
        "algorithm": name,
        "success": result.success,
        "path_cost": round(result.cost, 4),
        "expanded_nodes": result.stats.expanded_nodes,
        "samples": result.stats.samples,
        "tree_size": result.stats.tree_size,
        "iterations": result.stats.iterations,
        "path_len": len(result.path),
    }
    print(json.dumps(summary))


def run_discrete(name: str, factory: PlannerFactory) -> None:
    args = _parse_args(name)
    params, grid, start_world, goal_world = _load(args)
    planner = factory(params)
    start: Cell = grid.world_to_cell(*start_world)
    goal: Cell = grid.world_to_cell(*goal_world)
    with open_trace(args.trace) as recorder:
        recorder.planning_started(planner.name, args.map, params.values())
        result = planner.plan(grid, start, goal, recorder)
    _report(planner.name, result)


def run_sampling(name: str, factory: PlannerFactory) -> None:
    args = _parse_args(name)
    params, grid, start, goal = _load(args)
    planner = factory(params)
    with open_trace(args.trace) as recorder:
        recorder.planning_started(planner.name, args.map, params.values())
        result = planner.plan(grid, start, goal, recorder)
    _report(planner.name, result)
