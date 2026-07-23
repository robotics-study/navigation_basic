"""Shared demo scaffolding: arg parsing + run flow for discrete/sampling planners.

Demos are assembly only — they wire params + map + scenario + planner and emit
`planning_started` (the one event the planner cannot emit, since the map path is
known only here). Everything else is emitted by the planner.
"""

from __future__ import annotations

import argparse
import json
from collections.abc import Callable

from navigation.core.params import ParamSet
from navigation.core.planner import GlobalPlanner, LocalPlanner
from navigation.core.trace import open_trace
from navigation.core.types import Cell, LocalTask, PlanResult, Point, Pose, RobotState
from navigation.local_planning.simulation import SimConfig, simulate
from navigation.local_planning.velocity._velocity_obstacle import VelocityObstaclePlanner
from navigation.local_planning.velocity.agent_scenario import load_agent_scenario
from navigation.local_planning.velocity.agent_sim import simulate_agents
from navigation.maps.loader import load_map, load_scenario
from navigation.maps.occupancy_grid import OccupancyGrid2D

PlannerFactory = Callable[[ParamSet], GlobalPlanner]
LocalPlannerFactory = Callable[[ParamSet], LocalPlanner]
VelocityPlannerFactory = Callable[[ParamSet], VelocityObstaclePlanner]


def _parse_args(name: str) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=f"navigation {name} demo")
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


def run_local(name: str, factory: LocalPlannerFactory) -> None:
    # Local planners have no `plan()` result to summarize -- assembly instead
    # wires a closed-loop SimConfig (from the yaml's shared sim-params block)
    # and hands the run to the simulator, which owns tick order + termination.
    args = _parse_args(name)
    params = ParamSet.from_yaml(args.params)
    seed = params.get_int("seed") if params.has("seed") else args.seed
    grid = load_map(args.map, seed=seed, connectivity=args.connectivity)
    assert isinstance(grid, OccupancyGrid2D)
    scenario = load_scenario(args.scenario)
    planner = factory(params)
    task = LocalTask(
        goal=(scenario.goal[0], scenario.goal[1], scenario.goal_theta),
        reference_path=scenario.reference_path,
    )
    if planner.requires_reference_path() and not task.reference_path:
        raise ValueError(
            f"{planner.name} requires a reference_path, but {args.scenario} declares none"
        )
    config = SimConfig(
        control_dt=params.get_float("control_dt"),
        max_steps=params.get_int("max_steps"),
        goal_tolerance=params.get_float("goal_tolerance"),
        footprint_radius=params.get_float("footprint_radius"),
        stall_window=params.get_int("stall_window"),
        stall_distance=params.get_float("stall_distance"),
    )
    start = RobotState(pose=(scenario.start[0], scenario.start[1], scenario.start_theta))
    with open_trace(args.trace) as recorder:
        recorder.planning_started(planner.name, args.map, params.values(), scenario=args.scenario)
        result = simulate(planner, grid, start, task, config, recorder)
    summary = {
        "algorithm": planner.name,
        "status": result.status.value,
        "success": result.success,
        "time_to_goal": round(result.time_to_goal, 4),
        "distance_traveled": round(result.distance_traveled, 4),
        "min_clearance": round(result.min_clearance, 4),
        "steps": result.steps,
    }
    print(json.dumps(summary))


def run_agents(name: str, factory: VelocityPlannerFactory) -> None:
    # Multi-agent assembly for the velocity-obstacle family (VO/RVO/ORCA):
    # wires an AgentScenario (N bodies, one goal each, some possibly scripted
    # non-cooperative movers) instead of run_local's single-agent Scenario, and
    # hands the run to simulate_agents, whose tick loop owns termination + the
    # per-body trace order.
    args = _parse_args(name)
    params = ParamSet.from_yaml(args.params)
    seed = params.get_int("seed") if params.has("seed") else args.seed
    grid = load_map(args.map, seed=seed, connectivity=args.connectivity)
    assert isinstance(grid, OccupancyGrid2D)
    scenario = load_agent_scenario(args.scenario)
    config = SimConfig(
        control_dt=params.get_float("control_dt"),
        max_steps=params.get_int("max_steps"),
        goal_tolerance=params.get_float("goal_tolerance"),
        footprint_radius=params.get_float("footprint_radius"),
        stall_window=params.get_int("stall_window"),
        stall_distance=params.get_float("stall_distance"),
    )
    planners: list[VelocityObstaclePlanner | None] = [
        None if spec.scripted_velocity is not None else factory(params) for spec in scenario.agents
    ]
    with open_trace(args.trace) as recorder:
        recorder.planning_started(name, args.map, params.values())
        results = simulate_agents(planners, list(scenario.agents), grid, config, recorder)
    summary = [
        {
            "agent": k,
            "status": result.status.value,
            "steps": result.steps,
            "min_pair_clearance": round(result.min_pair_clearance, 4),
        }
        for k, result in enumerate(results)
    ]
    print(json.dumps(summary))


def run_kinodynamic(name: str, factory: PlannerFactory) -> None:
    # Kinodynamic (SE(2)) demo: builds Pose start/goal from the scenario (world x, y +
    # optional start_theta/goal_theta) and binds the grid as an SE2CollisionSpace[Pose].
    args = _parse_args(name)
    params = ParamSet.from_yaml(args.params)
    seed = params.get_int("seed") if params.has("seed") else args.seed
    grid = load_map(args.map, seed=seed, connectivity=args.connectivity)
    assert isinstance(grid, OccupancyGrid2D)
    scenario = load_scenario(args.scenario)
    planner = factory(params)
    start: Pose = (scenario.start[0], scenario.start[1], scenario.start_theta)
    goal: Pose = (scenario.goal[0], scenario.goal[1], scenario.goal_theta)
    with open_trace(args.trace) as recorder:
        recorder.planning_started(planner.name, args.map, params.values())
        result = planner.plan(grid, start, goal, recorder)
    _report(planner.name, result)
