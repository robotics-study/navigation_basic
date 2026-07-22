"""Stanley (Thrun et al. 2006, sec. 9.2 + Hoffmann et al. 2007 k_soft): steers
on heading + crosstrack error measured at the front axle, requires a
reference_path to run at all, and never leaks progress state between
episodes."""

from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml
from conftest import REPO_ROOT

from navigation.core.params import ParamError, ParamSet
from navigation.core.types import LocalTask, Point, RobotState
from navigation.local_planning.simulation import SimConfig, SimStatus, simulate
from navigation.local_planning.tracking.stanley import Stanley
from navigation.maps.loader import load_map, load_scenario

_ALGO = "stanley"
_LOCAL_CONFIG_DIR = REPO_ROOT / "configs" / "local_planning"
_MAP = REPO_ROOT / "maps" / "grid" / "open01.yaml"
_SCENARIO_ON_PATH = REPO_ROOT / "maps" / "scenarios" / "open01_s2.yaml"
_SCENARIO_OFFSET = REPO_ROOT / "maps" / "scenarios" / "open01_s3.yaml"


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


def _real_setup(scenario_path: Path) -> tuple[ParamSet, RobotState, LocalTask, SimConfig]:
    params = _real_params()
    scenario = load_scenario(scenario_path)
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


# --- (a) follows the open01_s2 S-curve to the goal with a tight terminal crosstrack error ----
def test_follows_s_curve_to_goal_with_small_terminal_cross_track_error() -> None:
    params, start, task, config = _real_setup(_SCENARIO_ON_PATH)
    grid = load_map(_MAP)
    planner = Stanley(params)

    result = simulate(planner, grid, start, task, config)

    assert result.status is SimStatus.REACHED
    assert result.success is True
    terminal = result.trajectory[-1]
    terminal_cross_track = _dist_to_polyline((terminal[0], terminal[1]), task.reference_path)
    assert terminal_cross_track < 0.5


# --- (b) open01_s3 offset start converges: crosstrack error shrinks toward the path ----
def test_offset_start_converges_toward_reference_path() -> None:
    params, start, task, config = _real_setup(_SCENARIO_OFFSET)
    grid = load_map(_MAP)
    planner = Stanley(params)

    result = simulate(planner, grid, start, task, config)

    assert result.status is SimStatus.REACHED
    errors = [_dist_to_polyline((p[0], p[1]), task.reference_path) for p in result.trajectory]
    initial_offset = errors[0]
    assert initial_offset > 1.0
    # Converged substantially by the end of the run.
    assert min(errors[-5:]) < initial_offset * 0.1
    # A monotone non-increasing run of consecutive steps exists somewhere --
    # the crosstrack error does not just wander down to a low value by chance.
    found_monotone_run = any(
        errors[i] >= errors[i + 1] >= errors[i + 2] >= errors[i + 3]
        for i in range(len(errors) - 3)
    )
    assert found_monotone_run


# --- (c) low-speed singularity: v near 0 with a large crosstrack error still ----
# --- yields a finite command (k_soft prevents the arctan argument from blowing up) ----
def test_low_speed_large_cross_track_error_yields_finite_command() -> None:
    params = _real_params()
    grid = load_map(_MAP)
    planner = Stanley(params)
    # Straight path along +x; robot sits far off to the side (large e) but at
    # its own goal (remaining ~= 0), so the speed profile drives v to ~0.
    path: tuple[Point, ...] = ((0.0, 0.0), (10.0, 0.0))
    state = RobotState(pose=(5.0, 5.0, 0.0))
    task = LocalTask(goal=(5.0, 5.0, 0.0), reference_path=path)

    cmd = planner.compute_command(grid, state, task, params.get_float("control_dt"))

    assert math.isfinite(cmd.v)
    assert math.isfinite(cmd.omega)
    assert abs(cmd.omega) <= params.get_float("max_omega") + 1e-9


# --- (d) max_steer above the declared range (1.55) fails load-time validation ----
def test_max_steer_above_range_rejected_at_load_time(tmp_path: Path) -> None:
    doc = yaml.safe_load((_LOCAL_CONFIG_DIR / f"{_ALGO}.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] == "max_steer":
            entry["default"] = 2.0
    bad = tmp_path / f"{_ALGO}.yaml"
    bad.write_text(yaml.safe_dump(doc), encoding="utf-8")
    with pytest.raises(ParamError):
        ParamSet.from_yaml(bad)


# --- (behavior) reset() leaves no _progress_index cursor leak across reruns ------
def test_rerun_after_implicit_reset_is_deterministic() -> None:
    # simulate() calls planner.reset() itself at the top of every episode -- this
    # proves that call actually clears _progress_index rather than the planner
    # silently resuming from wherever the first run's cursor stopped.
    _, start, task, config = _real_setup(_SCENARIO_ON_PATH)
    grid = load_map(_MAP)
    planner = Stanley(_real_params())

    first = simulate(planner, grid, start, task, config)
    second = simulate(planner, grid, start, task, config)

    assert second.status == first.status
    assert second.steps == first.steps
    assert second.distance_traveled == pytest.approx(first.distance_traveled)
    assert len(second.trajectory) == len(first.trajectory)
    for pose_a, pose_b in zip(first.trajectory, second.trajectory, strict=True):
        assert pose_a == pytest.approx(pose_b)
