"""Hybrid A* (kinodynamic SE(2), Pose state): feasibility, no-path, reverse, params, trace.

A third test family paralleling test_discrete (Cell) / test_sampling (Point): Hybrid A*
is the first Pose-state / SE2CollisionSpace planner, so its fixtures live here rather
than blurring the Cell-focused test_discrete.
"""

from __future__ import annotations

import io
import json
import math
from pathlib import Path

import pytest
from conftest import config, grid_from, open_grid, write_config

from navigation.core.params import ParamError, ParamSet
from navigation.core.trace import TraceRecorder
from navigation.core.types import Footprint, Pose
from navigation.global_planning import HybridAStar

# Full 11-param declaration so a written config satisfies every params.get_*; tests
# override only the fields they exercise (reverse/turn radius).
_TYPES = {"num_steering": "int", "theta_bins": "int", "allow_reverse": "bool"}
_MINS = {"min_turn_radius": 0.1, "steer_penalty": 0.0}


def _hybrid_params(**overrides: object) -> list[dict[str, object]]:
    base: dict[str, object] = dict(
        min_turn_radius=1.0, arc_step=0.5, num_steering=5, theta_bins=72, xy_resolution=0.5,
        footprint_radius=0.3, allow_reverse=False, reverse_penalty=2.0, steer_penalty=0.1,
        goal_pos_tolerance=0.5, goal_heading_tolerance=0.26,
    )
    base.update(overrides)
    params: list[dict[str, object]] = []
    for name, value in base.items():
        ptype = _TYPES.get(name, "float")
        decl: dict[str, object] = {"name": name, "type": ptype, "default": value,
                                   "description": name}
        if ptype == "float":
            decl["min"] = _MINS.get(name, 0.01)
            decl["max"] = 1000.0
        elif ptype == "int":
            decl["min"] = 2
            decl["max"] = 1000
        params.append(decl)
    return params


def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def test_hybrid_astar_finds_feasible_kinodynamic_path() -> None:
    # Defining behaviour: an open grid with reachable start/goal poses yields a
    # collision-free, kinematically feasible curve — each step within one primitive
    # (<= arc_step) and each heading change within the max-curvature bound
    # (<= arc_step / min_turn_radius). Exercises kappa=0 and kappa!=0 primitives,
    # collision acceptance, heuristic, reconstruction, and goal tolerance.
    grid = open_grid(24, 24)
    cfg = config("hybrid_astar")
    start: Pose = (3.0, 3.0, 0.0)
    goal: Pose = (14.0, 14.0, 0.0)
    res = HybridAStar(cfg).plan(grid, start, goal)
    assert res.success
    fp = Footprint(cfg.get_float("footprint_radius"))
    arc = cfg.get_float("arc_step")
    turn_r = cfg.get_float("min_turn_radius")
    assert res.path[0] == start
    ex, ey, eth = res.path[-1]
    assert math.hypot(ex - goal[0], ey - goal[1]) <= cfg.get_float("goal_pos_tolerance") + 1e-9
    assert abs(_wrap_pi(eth - goal[2])) <= cfg.get_float("goal_heading_tolerance") + 1e-9
    for pose in res.path:
        assert not grid.is_collision(fp, pose)
    for a, b in zip(res.path, res.path[1:], strict=False):
        assert math.hypot(b[0] - a[0], b[1] - a[1]) <= arc + 1e-9
        assert abs(_wrap_pi(b[2] - a[2])) <= arc / turn_r + 1e-9


def test_hybrid_astar_no_path_across_full_wall() -> None:
    # A fully occupied middle column separates start from goal; the footprint (r=0.3)
    # cannot pass. No path -> not success, empty path, zero cost. Also exercises
    # is_collision footprint rejection behaviourally.
    grid = grid_from(["...#...", "...#...", "...#...", "...#...", "...#...", "...#...", "...#..."])
    res = HybridAStar(config("hybrid_astar")).plan(grid, (1.5, 3.5, 0.0), (5.5, 3.5, 0.0))
    assert not res.success
    assert res.path == []
    assert res.cost == 0.0


def test_hybrid_astar_uses_reverse_when_enabled(tmp_path: Path) -> None:
    # With allow_reverse and a large turn radius, a goal directly behind the start is
    # reached by reversing (a forward U-turn is far costlier). Covers the reverse
    # primitive + reverse_penalty branch (off in the default config).
    cfg_path = write_config(
        tmp_path / "rev.yaml", "hybrid_astar",
        _hybrid_params(allow_reverse=True, min_turn_radius=3.0),
    )
    grid = open_grid(20, 20)
    res = HybridAStar(ParamSet.from_yaml(cfg_path)).plan(grid, (10.0, 10.0, 0.0), (6.0, 10.0, 0.0))
    assert res.success
    # A reverse segment: motion opposite the heading (dot(displacement, heading) < 0).
    has_reverse = any(
        (b[0] - a[0]) * math.cos(a[2]) + (b[1] - a[1]) * math.sin(a[2]) < 0.0
        for a, b in zip(res.path, res.path[1:], strict=False)
    )
    assert has_reverse


def test_hybrid_astar_rejects_nonpositive_turn_radius(tmp_path: Path) -> None:
    # hybrid_astar declares min_turn_radius >= 0.1; a below-min default must fail at
    # load time, so a non-positive (physically meaningless) turn radius can never reach
    # plan() and divide kappa_max by zero.
    cfg_path = write_config(
        tmp_path / "bad.yaml", "hybrid_astar",
        [{"name": "min_turn_radius", "type": "float", "default": 0.0,
          "min": 0.1, "max": 50.0, "description": "below min"}],
    )
    with pytest.raises(ParamError):
        ParamSet.from_yaml(cfg_path)


def test_hybrid_astar_emits_arc_and_path_trace() -> None:
    # Step-by-step replay contract (the emit block is all-new code the no-recorder tests
    # skip): assert the event set + that edge_added / path_found states are 3-element
    # [x, y, theta] Poses (pins the chained-sub-pose arc wiring), and that
    # planning_finished carries the three metric keys.
    grid = open_grid(10, 10)
    buf = io.StringIO()
    res = HybridAStar(config("hybrid_astar")).plan(grid, (2.0, 2.0, 0.0), (5.0, 5.0, 0.0),
                                                   TraceRecorder(buf))
    assert res.success
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    kinds = [e["event"] for e in events]
    assert kinds.count("node_expanded") >= 1
    edges = [e for e in events if e["event"] == "edge_added"]
    assert edges and all(len(e["state"]) == 3 and len(e["parent"]) == 3 for e in edges)
    path_events = [e for e in events if e["event"] == "path_found"]
    assert path_events and all(len(s) == 3 for s in path_events[-1]["path"])
    finished = events[-1]
    assert finished["event"] == "planning_finished" and finished["success"]
    assert {"runtime_sec", "path_cost", "expanded_nodes"} <= finished["metrics"].keys()
