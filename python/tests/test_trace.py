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


def test_data_field_carries_algorithm_info_and_is_omitted_when_empty() -> None:
    buf = io.StringIO()
    rec = TraceRecorder(buf)
    rec.node_expanded((0, 0), 0.0, data={"row": 3, "col_lo": 1, "col_hi": 4})
    rec.edge_added((0, 1), (0, 0), 1.0, data={"row": 3})
    rec.candidate_evaluated((0, 1), 1.0)  # no data
    rec.node_expanded((0, 2), 2.0, data={})  # empty data must be elided
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    assert events[0]["data"] == {"row": 3, "col_lo": 1, "col_hi": 4}
    assert events[1]["data"] == {"row": 3}
    assert "data" not in events[2]
    assert "data" not in events[3]


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


def test_robot_moved_data_is_optional_and_changes_output() -> None:
    # data=None keeps the historical (pre-local-planning) trace byte-identical;
    # only passing data adds the field.
    buf = io.StringIO()
    rec = TraceRecorder(buf)
    rec.robot_moved((1.0, 1.0, 0.0))
    rec.robot_moved((2.0, 2.0, 0.0), data={"v": 0.5, "omega": 0.1})
    lines = [json.loads(line) for line in buf.getvalue().splitlines()]
    assert "data" not in lines[0]
    assert lines[1]["data"] == {"v": 0.5, "omega": 0.1}


def test_force_computed_emits_state_and_data() -> None:
    buf = io.StringIO()
    rec = TraceRecorder(buf)
    force_data = {"fx_att": 1.0, "fy_att": 0.5, "fx_rep": -0.2, "fy_rep": 0.1, "fx": 0.8, "fy": 0.6}
    rec.force_computed((1.0, 2.0, 0.0), data=force_data)
    event = json.loads(buf.getvalue())
    assert event["event"] == "force_computed"
    assert event["state"] == [1.0, 2.0, 0.0]
    assert event["data"] == force_data


def test_histogram_updated_emits_bins_array() -> None:
    buf = io.StringIO()
    rec = TraceRecorder(buf)
    rec.histogram_updated((0.0, 0.0, 0.0), bins=[0.1, 0.2, 0.3], data={"threshold": 0.5})
    event = json.loads(buf.getvalue())
    assert event["event"] == "histogram_updated"
    assert event["bins"] == [0.1, 0.2, 0.3]
    assert event["data"] == {"threshold": 0.5}


def test_planning_started_scenario_field_present_only_when_given() -> None:
    buf = io.StringIO()
    rec = TraceRecorder(buf)
    rec.planning_started("potential_fields", "maps/grid/clutter01.yaml", {"k_att": 1.0})
    rec.planning_started(
        "pure_pursuit",
        "maps/grid/open01.yaml",
        {"lookahead_distance": 0.6},
        scenario="maps/scenarios/open01_s2.yaml",
    )
    lines = [json.loads(line) for line in buf.getvalue().splitlines()]
    assert "scenario" not in lines[0]
    assert lines[1]["scenario"] == "maps/scenarios/open01_s2.yaml"
