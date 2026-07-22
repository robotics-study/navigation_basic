"""Pure Pursuit (Coulter 1992): tracks a reference path with a single
constant-curvature lookahead arc, requires a reference_path to run at all, and
never leaks progress state between episodes."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import pytest
import yaml
from conftest import REPO_ROOT

from navigation.core.params import ParamError, ParamSet
from navigation.core.types import LocalTask, Point, RobotState
from navigation.local_planning.simulation import SimConfig, SimStatus, simulate
from navigation.local_planning.tracking.pure_pursuit import PurePursuit
from navigation.maps.loader import load_map, load_scenario

_ALGO = "pure_pursuit"
_LOCAL_CONFIG_DIR = REPO_ROOT / "configs" / "local_planning"
_MAP = REPO_ROOT / "maps" / "grid" / "open01.yaml"
_SCENARIO_WITH_PATH = REPO_ROOT / "maps" / "scenarios" / "open01_s2.yaml"
_SCENARIO_NO_PATH = REPO_ROOT / "maps" / "scenarios" / "open01_s1.yaml"


def _load_demo_common() -> ModuleType:
    # demos/ is a script directory, not an installed package -- loaded by file
    # path like tools/bench/run_matrix.py in test_bench.py, so the assembly-gate
    # branch inside run_local (not just PurePursuit itself) is what gets exercised.
    path = REPO_ROOT / "python" / "demos" / "demo_common.py"
    spec = importlib.util.spec_from_file_location("demo_common", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _dist_to_polyline(p: tuple[float, float], path: tuple[Point, ...]) -> float:
    best = float("inf")
    for a, b in zip(path, path[1:], strict=False):
        dx, dy = b[0] - a[0], b[1] - a[1]
        seg_len_sq = dx * dx + dy * dy
        t = 0.0 if seg_len_sq < 1e-12 else max(
            0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / seg_len_sq)
        )
        cx, cy = a[0] + t * dx, a[1] + t * dy
        best = min(best, ((p[0] - cx) ** 2 + (p[1] - cy) ** 2) ** 0.5)
    return best


def _real_params() -> ParamSet:
    return ParamSet.from_yaml(_LOCAL_CONFIG_DIR / f"{_ALGO}.yaml")


def _real_setup() -> tuple[ParamSet, RobotState, LocalTask, SimConfig]:
    params = _real_params()
    scenario = load_scenario(_SCENARIO_WITH_PATH)
    start = RobotState(pose=(scenario.start[0], scenario.start[1], scenario.start_theta))
    task = LocalTask(
        goal=(scenario.goal[0], scenario.goal[1], scenario.goal_theta),
        reference_path=scenario.reference_path,
    )
    config = SimConfig(
        control_dt=params.get_float("control_dt"),
        max_steps=params.get_int("max_steps"),
        goal_tolerance=params.get_float("goal_tolerance"),
        footprint_radius=params.get_float("footprint_radius"),
        stall_window=params.get_int("stall_window"),
        stall_distance=params.get_float("stall_distance"),
    )
    return params, start, task, config


# --- (a) follows the open01_s2 S-curve to the goal, bounded cross-track error ----
def test_follows_s_curve_to_goal_with_bounded_cross_track_error() -> None:
    params, start, task, config = _real_setup()
    grid = load_map(_MAP)
    planner = PurePursuit(params)

    result = simulate(planner, grid, start, task, config)

    assert result.status is SimStatus.REACHED
    assert result.success is True
    max_cross_track = max(
        _dist_to_polyline((p[0], p[1]), task.reference_path) for p in result.trajectory
    )
    assert max_cross_track < params.get_float("lookahead_distance")


# --- (b) missing reference_path is rejected at assembly time, not mid-tick -------
def test_run_local_rejects_scenario_without_reference_path(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    demo_common = _load_demo_common()
    trace_path = tmp_path / "trace.jsonl"
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "demo_pure_pursuit.py",
            "--map",
            str(_MAP),
            "--scenario",
            str(_SCENARIO_NO_PATH),
            "--params",
            str(_LOCAL_CONFIG_DIR / f"{_ALGO}.yaml"),
            "--trace",
            str(trace_path),
        ],
    )
    with pytest.raises(ValueError, match="reference_path"):
        demo_common.run_local(_ALGO, PurePursuit)


# --- (c) lookahead_distance = 0 fails load-time range validation -----------------
def test_zero_lookahead_distance_rejected_at_load_time(tmp_path: Path) -> None:
    doc = yaml.safe_load((_LOCAL_CONFIG_DIR / f"{_ALGO}.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] == "lookahead_distance":
            entry["default"] = 0.0
    bad = tmp_path / f"{_ALGO}.yaml"
    bad.write_text(yaml.safe_dump(doc), encoding="utf-8")
    with pytest.raises(ParamError):
        ParamSet.from_yaml(bad)


# --- (behavior) reset() leaves no _progress_index cursor leak across reruns ------
def test_rerun_after_implicit_reset_is_deterministic() -> None:
    # simulate() calls planner.reset() itself at the top of every episode -- this
    # proves that call actually clears _progress_index rather than the planner
    # silently resuming from wherever the first run's cursor stopped.
    _, start, task, config = _real_setup()
    grid = load_map(_MAP)
    planner = PurePursuit(_real_params())

    first = simulate(planner, grid, start, task, config)
    second = simulate(planner, grid, start, task, config)

    assert second.status == first.status
    assert second.steps == first.steps
    assert second.distance_traveled == pytest.approx(first.distance_traveled)
    assert len(second.trajectory) == len(first.trajectory)
    for pose_a, pose_b in zip(first.trajectory, second.trajectory, strict=True):
        assert pose_a == pytest.approx(pose_b)
