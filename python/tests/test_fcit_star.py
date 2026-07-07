"""FCIT*: reaches the goal on open space, fails when walled off, near-optimal.

Kept fast with small grids and a modest sample budget, since FCIT*'s candidate
graph is fully connected (O(n^2) edges).
"""

from __future__ import annotations

import collections
import io
import json
from pathlib import Path

import pytest
import yaml
from conftest import CONFIG_DIR, grid_from, open_grid

from navigation.core.params import ParamError, ParamSet
from navigation.core.trace import TraceRecorder
from navigation.global_planning import FCITStar
from navigation.maps.occupancy_grid import OccupancyGrid2D


def _config(tmp_path: Path, batch_size: int, max_batches: int) -> ParamSet:
    """Real config with a reduced sample budget so the O(n^2) graph stays quick."""
    doc = yaml.safe_load((CONFIG_DIR / "fcit_star.yaml").read_text())
    overrides = {"batch_size": batch_size, "max_batches": max_batches}
    for entry in doc["params"]:
        if entry["name"] in overrides:
            entry["default"] = overrides[entry["name"]]
    out = tmp_path / "fcit_star.yaml"
    out.write_text(yaml.safe_dump(doc), encoding="utf-8")
    return ParamSet.from_yaml(out)


def _motion_valid_path(grid: OccupancyGrid2D, path: list[tuple[float, float]]) -> bool:
    return all(grid.is_motion_valid(a, b) for a, b in zip(path, path[1:], strict=False))


def test_reaches_goal_on_open_grid(tmp_path: Path) -> None:
    grid = open_grid(10, 10, seed=7)
    start, goal = (0.5, 0.5), (9.0, 9.0)
    result = FCITStar(_config(tmp_path, batch_size=60, max_batches=3)).plan(grid, start, goal)
    assert result.success
    # start/goal are permanent samples, so the incumbent path pins to them exactly.
    assert result.path[0] == pytest.approx(start)
    assert result.path[-1] == pytest.approx(goal)
    assert _motion_valid_path(grid, result.path)
    assert result.cost > 0.0


def test_no_path_when_walled_off(tmp_path: Path) -> None:
    # Fully occupied middle column: every motion across it fails validation, so the
    # forward search exhausts the start side without ever reaching the goal. This
    # exercises the invalid-edge feedback and the rewire branch (many start-side
    # vertices expand and get re-routed) that the open-grid case does not.
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."], seed=3)
    params = _config(tmp_path, batch_size=40, max_batches=3)
    result = FCITStar(params).plan(grid, (0.5, 0.5), (4.5, 4.5))
    assert not result.success
    assert result.path == []
    assert result.cost == 0.0


def test_near_straight_line_on_open_grid(tmp_path: Path) -> None:
    # On an obstacle-free grid the fully connected graph contains the direct
    # start-goal edge, so an asymptotically optimal search returns essentially the
    # straight-line lower bound. 1.15x leaves slack for the discrete grid metric.
    grid = open_grid(12, 12, seed=5)
    start, goal = (0.5, 0.5), (11.0, 11.0)
    lower_bound = grid.distance(start, goal)
    result = FCITStar(_config(tmp_path, batch_size=60, max_batches=3)).plan(grid, start, goal)
    assert result.success
    assert result.cost <= lower_bound * 1.15


def test_trace_events_on_detour_map(tmp_path: Path) -> None:
    # A wall with a single bottom-row gap forces the solution around a detour, so
    # the fully connected forward search reaches some vertices only through
    # intermediates (the direct start-edge being blocked) and improves them via a
    # cheaper route mid-search. That drives every recorder branch: sample_drawn,
    # candidate_evaluated, edge_added (first connection), rewire (cheaper reroute),
    # and path_found (each incumbent improvement across the anytime batches).
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "....."], seed=3)
    start, goal = (0.5, 4.5), (4.5, 0.5)
    buf = io.StringIO()
    result = FCITStar(_config(tmp_path, batch_size=40, max_batches=3)).plan(
        grid, start, goal, TraceRecorder(buf)
    )
    assert result.success
    assert _motion_valid_path(grid, result.path)
    assert grid.distance(result.path[-1], goal) == pytest.approx(0.0)
    events = collections.Counter(json.loads(line)["event"] for line in buf.getvalue().splitlines())
    assert events["sample_drawn"] > 0
    assert events["candidate_evaluated"] > 0
    assert events["edge_added"] > 0
    assert events["rewire"] > 0  # a vertex reached via a detour was later improved
    assert events["path_found"] > 0
    assert events["planning_finished"] == 1


def test_out_of_range_param_rejected(tmp_path: Path) -> None:
    # Load-time declarative validation: a default outside [min, max] must throw.
    doc = yaml.safe_load((CONFIG_DIR / "fcit_star.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] == "batch_size":
            entry["default"] = 0  # below min = 1
    out = tmp_path / "fcit_star_bad.yaml"
    out.write_text(yaml.safe_dump(doc), encoding="utf-8")
    with pytest.raises(ParamError):
        ParamSet.from_yaml(out)
