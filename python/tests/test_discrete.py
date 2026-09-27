"""Discrete planners: optimality / validity, and the no-path case."""

from __future__ import annotations

import io
import json
import math
from pathlib import Path

import pytest
from conftest import config, grid_from, open_grid, write_config

from navigation.core.params import ParamError, ParamSet
from navigation.core.trace import TraceRecorder
from navigation.core.types import Cell
from navigation.global_planning import BFS, AStar, Dijkstra, DStarLite, LpaStar, ThetaStar

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
        LpaStar(config("lpa_star")),
        DStarLite(config("dstar_lite")),
    ):
        res = planner.plan(grid, (0, 0), (0, 4))
        assert not res.success
        assert res.path == []
        assert res.cost == 0.0


def test_lpa_star_matches_astar_and_expands_no_more_than_dijkstra() -> None:
    # Round 0 with no revisions is plain A* (same octile heuristic): the cost must
    # equal A*'s exactly, and the heuristic must not expand more than uninformed
    # Dijkstra on the same open grid.
    grid = open_grid(9, 9)
    start, goal = (8, 0), (0, 8)
    res = LpaStar(config("lpa_star")).plan(grid, start, goal)
    assert res.success and res.path[0] == start and res.path[-1] == goal
    ast = AStar(config("astar")).plan(grid, start, goal)
    dj = Dijkstra(config("dijkstra")).plan(grid, start, goal)
    assert res.cost == pytest.approx(ast.cost)
    assert res.stats.expanded_nodes <= dj.stats.expanded_nodes


def test_lpa_star_replans_on_a_reversible_wall_and_back() -> None:
    # Lifelong cycle on ONE cell (the demo scenario in miniature): gate open -> cost
    # equals A* on the open grid; blocking the gate seals the map (no path); freeing
    # THE SAME cell again restores optimality. Revisions are reversible, and only the
    # planner's own model (never the static ground truth) tracks both directions.
    opened = [".....", "..#..", "..#.."]
    start, goal = (1, 0), (1, 4)
    grid = grid_from(opened)
    planner = LpaStar(config("lpa_star"))
    res_open = planner.plan(grid, start, goal)
    assert res_open.success and res_open.stats.iterations == 0
    ast = AStar(config("astar")).plan(grid_from(opened), start, goal)
    assert res_open.cost == pytest.approx(ast.cost)
    planner.apply_revision([(0, 2)], True)  # free in the base map -> sealed column
    res_sealed = planner.plan(grid, start, goal)
    assert not res_sealed.success and res_sealed.path == [] and res_sealed.cost == 0.0
    assert res_sealed.stats.iterations == 1
    planner.apply_revision([(0, 2)], False)  # the reversible half: open it again
    res_back = planner.plan(grid, start, goal)
    assert res_back.success and _path_is_connected(grid, res_back.path)
    assert (0, 2) in res_back.path  # the reopened gate is the only crossing again
    assert res_back.cost == pytest.approx(res_open.cost)
    assert res_back.stats.iterations == 2


def test_lpa_star_emits_obstacle_changed_and_per_round_trace() -> None:
    # Replay contract: one obstacle_changed per flipped cell (carrying the post-flip
    # state), one planning_finished per plan() round (last wins) with cumulative
    # metrics, and path_found only on rounds that actually reach the goal.
    grid = grid_from(["..#..", "..#..", "..#.."])
    buf = io.StringIO()
    rec = TraceRecorder(buf)
    planner = LpaStar(config("lpa_star"))
    res0 = planner.plan(grid, (1, 0), (1, 4), rec)
    assert not res0.success
    planner.apply_revision([(1, 2)], False)  # open the wall -> reachable again
    res1 = planner.plan(grid, (1, 0), (1, 4), rec)
    assert res1.success and (1, 2) in res1.path
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    kinds = [e["event"] for e in events]
    changed = [e for e in events if e["event"] == "obstacle_changed"]
    assert len(changed) == 1 and changed[0]["state"] == [1, 2] and changed[0]["blocked"] is False
    assert kinds.count("path_found") == 1  # round 0 was unreachable -> no path_found
    finished = [e for e in events if e["event"] == "planning_finished"]
    assert len(finished) == 2  # one per plan() round; the driver's _report reads the last
    assert finished[0]["success"] is False and finished[1]["success"] is True
    metrics = finished[1]["metrics"]
    assert {"runtime_sec", "path_cost", "expanded_nodes", "replan_count"} <= metrics.keys()
    assert metrics["replan_count"] == 1.0


def test_lpa_star_apply_revision_before_plan_raises() -> None:
    # The model (and the recorder it emits through) is seeded in plan(); a revision
    # before that has no model to repair.
    planner = LpaStar(config("lpa_star"))
    with pytest.raises(RuntimeError):
        planner.apply_revision([(0, 0)], True)


def test_lpa_star_rejects_moved_endpoints() -> None:
    # Fixed start/goal IS the algorithm; re-rooting is D* Lite's job.
    grid = open_grid(3, 3)
    planner = LpaStar(config("lpa_star"))
    planner.plan(grid, (1, 0), (1, 2))
    with pytest.raises(ValueError):
        planner.plan(grid, (1, 0), (2, 2))


def test_dstar_lite_reaches_goal_without_replan_on_open_grid() -> None:
    # All-free grid: nothing is ever sensed as blocked, so D* Lite never replans and
    # the executed trajectory is the freespace optimum (matches A*). Interior
    # start/goal keep the sensor disk inside the grid.
    grid = open_grid(11, 11)
    start, goal = (7, 3), (3, 7)
    res = DStarLite(config("dstar_lite")).plan(grid, start, goal)
    assert res.success
    assert res.path[0] == start and res.path[-1] == goal
    assert res.stats.iterations == 0  # no obstacle revealed -> no replan
    ast = AStar(config("astar")).plan(grid, start, goal)
    assert res.cost == pytest.approx(ast.cost)


def test_dstar_lite_replans_around_hidden_wall(tmp_path: Path) -> None:
    # D* Lite's defining behaviour: a wall the 1-cell sensor cannot see from the start
    # forces the robot to step, discover it, and incrementally repair. Only the bottom
    # row is open, so the robot must detour — a genuinely longer path than A*'s
    # full-knowledge optimum is a lower bound it cannot beat.
    cfg = write_config(
        tmp_path / "dstar_r1.yaml",
        "dstar_lite",
        [{"name": "sensor_radius", "type": "int", "default": 1,
          "min": 1, "max": 50, "description": "one-cell sensor"}],
    )
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#..", "..#..", "....."])
    start, goal = (3, 0), (3, 4)
    res = DStarLite(ParamSet.from_yaml(cfg)).plan(grid, start, goal)
    assert res.success
    assert res.path[0] == start and res.path[-1] == goal
    assert res.stats.iterations >= 1  # the hidden wall triggered a replan
    assert _path_is_connected(grid, res.path)  # every step is a legal true-grid move
    assert res.cost > math.hypot(start[0] - goal[0], start[1] - goal[1])  # real detour
    ast = AStar(config("astar")).plan(grid, start, goal)
    assert res.cost >= ast.cost  # A* has full knowledge; realized trajectory is >= optimal


def test_dstar_lite_emits_robot_and_obstacle_trace(tmp_path: Path) -> None:
    # The step-by-step replay contract: the planner must emit one robot_moved per
    # executed cell and an obstacle_revealed when the sensor finds a hidden wall.
    cfg = write_config(
        tmp_path / "dstar_r1.yaml",
        "dstar_lite",
        [{"name": "sensor_radius", "type": "int", "default": 1,
          "min": 1, "max": 50, "description": "one-cell sensor"}],
    )
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#..", "..#..", "....."])
    buf = io.StringIO()
    res = DStarLite(ParamSet.from_yaml(cfg)).plan(grid, (3, 0), (3, 4), TraceRecorder(buf))
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    kinds = [e["event"] for e in events]
    # one robot_moved per trajectory cell; at least one obstacle discovered mid-run.
    assert kinds.count("robot_moved") == len(res.path)
    assert kinds.count("obstacle_revealed") >= 1
    finished = events[-1]
    assert finished["event"] == "planning_finished" and finished["success"]
    assert {"replan_count", "sensed_cells", "path_cost"} <= finished["metrics"].keys()


def test_dstar_lite_does_not_cut_isolated_diagonal_obstacle(tmp_path: Path) -> None:
    # Regression: the 1-cell sensor's Euclidean disk (dr^2+dc^2<=1) omits the diagonals,
    # yet the robot may step diagonally next. The immediate 8-neighbourhood must always
    # be sensed, or the robot walks onto an isolated diagonal obstacle it never detected.
    cfg = write_config(
        tmp_path / "dstar_r1.yaml", "dstar_lite",
        [{"name": "sensor_radius", "type": "int", "default": 1,
          "min": 1, "max": 50, "description": "one-cell sensor"}],
    )
    grid = grid_from(["...", ".#.", "..."])  # lone obstacle at (1,1)
    res = DStarLite(ParamSet.from_yaml(cfg)).plan(grid, (2, 0), (0, 2))
    assert res.success
    assert (1, 1) not in res.path  # never routed through the true obstacle
    assert _path_is_connected(grid, res.path)  # every step is a legal true-grid move


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


def test_dstar_lite_rejects_out_of_range_radius(tmp_path: Path) -> None:
    # dstar_lite declares sensor_radius >= 1; a below-min default must fail at load
    # time, so an invalid radius can never reach plan().
    cfg = write_config(
        tmp_path / "bad_dstar.yaml",
        "dstar_lite",
        [{"name": "sensor_radius", "type": "int", "default": 0,
          "min": 1, "max": 50, "description": "below min"}],
    )
    with pytest.raises(ParamError):
        ParamSet.from_yaml(cfg)
