"""Each planner emits its documented trace events (the visualization contract)."""

from __future__ import annotations

import io
import json
from pathlib import Path

import yaml
from conftest import CONFIG_DIR, config, grid_from, open_grid

from navigation.core.params import ParamSet
from navigation.core.trace import TraceRecorder
from navigation.global_planning import RRT, AStar, FastRRT, RRTStar


def _capped(tmp_path: Path, algo: str, max_iter: int) -> ParamSet:
    doc = yaml.safe_load((CONFIG_DIR / f"{algo}.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] == "max_iterations":
            entry["default"] = max_iter
    out = tmp_path / f"{algo}.yaml"
    out.write_text(yaml.safe_dump(doc), encoding="utf-8")
    return ParamSet.from_yaml(out)


def _events(recorder_buf: io.StringIO) -> set[str]:
    return {json.loads(line)["event"] for line in recorder_buf.getvalue().splitlines()}


def test_astar_emits_search_and_path_events() -> None:
    buf = io.StringIO()
    AStar(config("astar")).plan(open_grid(6, 6), (5, 0), (0, 5), TraceRecorder(buf))
    seen = _events(buf)
    assert {"node_expanded", "edge_added", "candidate_evaluated", "path_found",
            "planning_finished"} <= seen


def test_rrt_emits_samples_and_path() -> None:
    buf = io.StringIO()
    RRT(config("rrt")).plan(open_grid(8, 8, seed=5), (0.5, 0.5), (7.0, 7.0), TraceRecorder(buf))
    assert {"sample_drawn", "edge_added", "path_found", "planning_finished"} <= _events(buf)


def test_rrt_star_emits_candidate_and_rewire(tmp_path: Path) -> None:
    buf = io.StringIO()
    RRTStar(_capped(tmp_path, "rrt_star", 800)).plan(
        open_grid(8, 8, seed=5), (0.5, 0.5), (7.0, 7.0), TraceRecorder(buf)
    )
    seen = _events(buf)
    assert {"candidate_evaluated", "rewire", "path_found", "planning_finished"} <= seen


def test_fast_rrt_random_steering_on_walls(tmp_path: Path) -> None:
    # A walled grid forces blocked straight extensions -> Random Steering fires
    # (and no path exists, so planning_finished reports failure).
    buf = io.StringIO()
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."], seed=2)
    result = FastRRT(_capped(tmp_path, "fast_rrt", 400)).plan(
        grid, (0.5, 0.5), (4.5, 4.5), TraceRecorder(buf)
    )
    assert not result.success
    assert "planning_finished" in _events(buf)
