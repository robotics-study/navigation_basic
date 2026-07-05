"""TraceRecorder JSON Lines output + null-recorder behavior."""

from __future__ import annotations

import io
import json
from pathlib import Path

from conftest import config, open_grid

from navigation.core.trace import TraceRecorder, open_trace
from navigation.global_planning import AStar


def _emit_sample_trace() -> list[dict[str, object]]:
    buf = io.StringIO()
    rec = TraceRecorder(buf)
    rec.planning_started("astar", "maps/grid/x.yaml", {"heuristic_weight": 1.0})
    rec.node_expanded((0, 0), 0.0)
    rec.edge_added((0, 1), (0, 0), 1.0)
    rec.candidate_evaluated((0, 1), 1.0)
    rec.path_found([(0, 0), (0, 1)])
    rec.planning_finished(True, {"runtime_sec": 0.01, "path_cost": 1.0, "expanded_nodes": 1.0})
    return [json.loads(line) for line in buf.getvalue().splitlines()]


def test_seq_starts_zero_and_is_monotonic() -> None:
    events = _emit_sample_trace()
    seqs = [e["seq"] for e in events]
    assert seqs == list(range(len(events)))


def test_required_fields_per_event() -> None:
    events = {e["event"]: e for e in _emit_sample_trace()}
    assert events["planning_started"].keys() >= {"algorithm", "map", "params"}
    assert events["node_expanded"]["state"] == [0, 0]
    assert events["edge_added"]["parent"] == [0, 0]
    assert events["path_found"]["path"] == [[0, 0], [0, 1]]
    finished = events["planning_finished"]
    assert finished["success"] is True
    metrics = finished["metrics"]
    assert isinstance(metrics, dict)
    assert set(metrics) >= {"runtime_sec", "path_cost", "expanded_nodes"}
    # State arrays are numeric per schema.
    assert all(isinstance(v, (int, float)) for v in events["node_expanded"]["state"])


def test_optional_fields_omitted_when_absent() -> None:
    buf = io.StringIO()
    rec = TraceRecorder(buf)
    rec.node_expanded((1, 1))  # no cost
    rec.edge_added((1, 2), (1, 1))  # no cost
    rec.sample_drawn((0.5, 0.5))
    rec.rewire((1, 2), (1, 1))
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    assert "cost" not in events[0]  # node_expanded without cost
    assert "cost" not in events[1]  # edge_added without cost
    assert events[2]["event"] == "sample_drawn" and events[2]["state"] == [0.5, 0.5]
    assert events[3]["event"] == "rewire" and events[3]["parent"] == [1, 1]


def test_open_trace_writes_and_closes(tmp_path: Path) -> None:
    path = str(tmp_path / "t.jsonl")
    with open_trace(path) as rec:
        rec.planning_finished(False, {"runtime_sec": 0.0, "path_cost": 0.0, "expanded_nodes": 0.0})
    with open(path) as fh:
        line = json.loads(fh.readline())
    assert line == {"seq": 0, "t": line["t"], "event": "planning_finished",
                    "success": False, "metrics": line["metrics"]}


def test_null_recorder_is_zero_cost() -> None:
    # Passing None must not raise and must still produce a result (guarded emits).
    grid = open_grid(5, 5)
    result = AStar(config("astar")).plan(grid, (4, 0), (0, 4), None)
    assert result.success
