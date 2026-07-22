"""tools/bench/run_matrix.py: pure-function logic (no subprocess).

End-to-end (map x algorithm x scenario) execution is exercised by actually running
the matrix (self-verification, per the tools/ testing convention) — this file covers
the new local/global classification, status reconstruction, and the _NEEDS_PATH
compatibility gate directly, since those are cheap pure functions and the gate must
be proven to short-circuit *before* the subprocess call it guards.

run_matrix lives outside the package (tools/), so it is loaded by file path.
"""

from __future__ import annotations

import importlib.util
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import pytest
from conftest import REPO_ROOT


def _load_run_matrix() -> ModuleType:
    path = REPO_ROOT / "tools" / "bench" / "run_matrix.py"
    spec = importlib.util.spec_from_file_location("run_matrix", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


bench = _load_run_matrix()


def test_is_local_algo_matches_the_three_reactive_tracking_planners() -> None:
    assert bench._is_local_algo("potential_fields")
    assert bench._is_local_algo("vfh")
    assert bench._is_local_algo("pure_pursuit")
    assert not bench._is_local_algo("astar")
    assert not bench._is_local_algo("rrt_star")


def test_local_status_reconstructs_from_numeric_flags() -> None:
    assert bench._local_status({"success": 1.0}) == "ok"
    assert bench._local_status({"success": 0.0, "collided": 1.0}) == "collision"
    assert bench._local_status({"success": 0.0, "collided": 0.0, "stalled": 1.0}) == "stalled"
    assert bench._local_status({"success": 0.0, "collided": 0.0, "stalled": 0.0}) == "timeout"


def test_is_local_row_classifies_ok_rows_by_metric_key() -> None:
    local_row = bench.Row("s", "potential_fields", "ok", {"time_to_goal": 1.0})
    global_row = bench.Row("s", "astar", "ok", {"path_cost": 2.0})
    assert bench._is_local_row(local_row)
    assert not bench._is_local_row(global_row)


def test_is_local_row_falls_back_to_capability_when_metrics_empty() -> None:
    # incompatible/error rows carry no trace (empty metrics), so classification falls
    # back to the algorithm's own required capability instead of a second category map.
    incompatible_local = bench.Row("s", "pure_pursuit", "incompatible", {})
    incompatible_global = bench.Row("s", "rrt", "incompatible", {})
    assert bench._is_local_row(incompatible_local)
    assert not bench._is_local_row(incompatible_global)


def test_config_path_resolves_first_match_across_dirs() -> None:
    dirs = [REPO_ROOT / "configs" / "global_planning", REPO_ROOT / "configs" / "local_planning"]
    resolved = bench._config_path(dirs, "astar")
    assert resolved == dirs[0] / "astar.yaml"
    assert bench._config_path(dirs, "definitely_not_an_algorithm") is None


def test_run_one_needs_path_gate_fires_before_subprocess(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # open01_s1.yaml has no reference_path, so Pure Pursuit must be reported
    # incompatible from the static gate — never reaching the demo subprocess.
    scenario_path = REPO_ROOT / "maps" / "scenarios" / "open01_s1.yaml"

    def _fail_if_called(*args: object, **kwargs: object) -> subprocess.CompletedProcess[str]:
        raise AssertionError("subprocess must not run once the _NEEDS_PATH gate fires")

    monkeypatch.setattr(bench.subprocess, "run", _fail_if_called)
    row = bench._run_one(
        "python3", Path("unused_demos"), Path("unused_config.yaml"), scenario_path,
        "pure_pursuit",
    )
    assert row.status == "incompatible"
    assert row.metrics == {}


def test_run_one_needs_path_gate_passes_once_reference_path_present(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # open01_s2.yaml has a reference_path, so the gate must let this scenario through
    # to the subprocess step (which then fails since demo_pure_pursuit.py is a later
    # wave's deliverable — that failure is _run_one's normal "error" path, not the gate).
    scenario_path = REPO_ROOT / "maps" / "scenarios" / "open01_s2.yaml"
    calls: list[list[str]] = []

    def _record_call(cmd: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        calls.append(cmd)
        return subprocess.CompletedProcess(cmd, returncode=1, stdout="", stderr="")

    monkeypatch.setattr(bench.subprocess, "run", _record_call)
    row = bench._run_one(
        "python3", Path("unused_demos"), Path("unused_config.yaml"), scenario_path,
        "pure_pursuit",
    )
    assert calls, "the gate must not block a scenario that has a reference_path"
    assert row.status == "error"


def test_render_splits_local_and_global_into_separate_tables() -> None:
    rows = [
        bench.Row("s1", "astar", "ok", {"path_cost": 1.0, "expanded_nodes": 2.0,
                                         "runtime_sec": 0.01}),
        bench.Row("s1", "potential_fields", "ok",
                  {"time_to_goal": 1.0, "distance_traveled": 2.0, "min_clearance": 0.3,
                   "steps": 4.0, "runtime_sec": 0.02}),
    ]
    report = bench._render(rows)
    assert "## Global planning" in report
    assert "## Local planning" in report
    assert "path_cost" in report
    assert "time_to_goal" in report


def test_render_stays_a_single_table_for_global_only_runs() -> None:
    # Byte-format regression: the historical global_planning-only invocation must not
    # gain a "## Global planning" heading it never had.
    rows = [bench.Row("s1", "astar", "ok", {"path_cost": 1.0})]
    report = bench._render(rows)
    assert "## Global planning" not in report
    assert "## Local planning" not in report
    assert "path_cost" in report
