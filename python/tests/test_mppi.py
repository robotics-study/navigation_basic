"""MPPI (Williams et al. 2016/2018): softmax importance-weighted sampling on the
shared receding-horizon cost J(U), verified via the closed-loop simulator on an
open-field interior-goal scenario (goal-seeking around an obstacle) and via direct
compute_command calls for behavior a full episode can't isolate cleanly (seeded
reproducibility, softmax temperature concentrating the sample weights, the
obstacle term bending the predicted horizon away from an obstacle)."""

from __future__ import annotations

import io
import json
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml
from conftest import REPO_ROOT

from navigation.core.params import ParamError, ParamSet
from navigation.core.trace import TraceRecorder
from navigation.core.types import LocalTask, RobotState
from navigation.local_planning._geometry import nearest_occupied
from navigation.local_planning.predictive.mppi import MppiPlanner
from navigation.local_planning.simulation import SimConfig, SimStatus, simulate
from navigation.maps.loader import load_map, load_scenario
from navigation.maps.occupancy_grid import OccupancyGrid2D

_ALGO = "mppi"
_CONFIG_PATH = REPO_ROOT / "configs" / "local_planning" / f"{_ALGO}.yaml"
_MAPS_DIR = REPO_ROOT / "maps"
# Reach on an open-field interior goal: predictive planners push clearance inside
# min_obstacle_dist away, so a goal clear of the obstacle-penalty zone is required.
# A corner goal (clutter01_s1) sits inside the zone and is the documented weakness.
_MAP = "open01"
_SCENARIO = "open01_s4"


def _config(tmp_path: Path, **overrides: Any) -> ParamSet:
    """Real config with selected defaults overridden."""
    doc = yaml.safe_load(_CONFIG_PATH.read_text())
    for entry in doc["params"]:
        if entry["name"] in overrides:
            entry["default"] = overrides[entry["name"]]
    out = tmp_path / f"{_ALGO}.yaml"
    out.write_text(yaml.safe_dump(doc), encoding="utf-8")
    return ParamSet.from_yaml(out)


def _sim_config(params: ParamSet) -> SimConfig:
    return SimConfig(
        control_dt=params.get_float("control_dt"),
        max_steps=params.get_int("max_steps"),
        goal_tolerance=params.get_float("goal_tolerance"),
        footprint_radius=params.get_float("footprint_radius"),
        stall_window=params.get_int("stall_window"),
        stall_distance=params.get_float("stall_distance"),
    )


def _run(params: ParamSet, recorder: TraceRecorder | None = None) -> Any:
    grid = load_map(_MAPS_DIR / "grid" / f"{_MAP}.yaml")
    scenario = load_scenario(_MAPS_DIR / "scenarios" / f"{_SCENARIO}.yaml")
    planner = MppiPlanner(params)
    start = RobotState(pose=(scenario.start[0], scenario.start[1], scenario.start_theta))
    task = LocalTask(
        goal=(scenario.goal[0], scenario.goal[1], scenario.goal_theta),
        reference_path=scenario.reference_path,
    )
    return simulate(planner, grid, start, task, _sim_config(params), recorder)


def _events(buf: io.StringIO, event: str) -> list[dict[str, Any]]:
    parsed = (json.loads(line) for line in buf.getvalue().splitlines())
    return [e for e in parsed if e["event"] == event]


def _moved_poses(buf: io.StringIO) -> list[tuple[float, ...]]:
    return [tuple(e["state"]) for e in _events(buf, "robot_moved")]


def _single_tick_buffer(params: ParamSet) -> io.StringIO:
    """One cold-start compute_command from the scenario start, capturing its trace
    (K candidate_evaluated events + one nominal band_updated)."""
    grid = load_map(_MAPS_DIR / "grid" / f"{_MAP}.yaml")
    scenario = load_scenario(_MAPS_DIR / "scenarios" / f"{_SCENARIO}.yaml")
    planner = MppiPlanner(params)
    state = RobotState(pose=(scenario.start[0], scenario.start[1], scenario.start_theta))
    task = LocalTask(
        goal=(scenario.goal[0], scenario.goal[1], scenario.goal_theta),
        reference_path=scenario.reference_path,
    )
    buf = io.StringIO()
    planner.compute_command(grid, state, task, params.get_float("control_dt"), TraceRecorder(buf))
    return buf


# --- (a) closed-loop MPPI reaches an open-field interior goal ------------------
def test_reaches_goal_open_field(tmp_path: Path) -> None:
    result = _run(_config(tmp_path))
    assert result.status is SimStatus.REACHED
    assert result.success is True
    # A positive min clearance means it threaded past the obstacle rather than
    # grazing a wall on the way to the goal.
    assert result.min_clearance > 0.0


# --- (b) every executed command stays within the declared v/omega limits ------
def test_commands_respect_limits(tmp_path: Path) -> None:
    # The box clip on the perturbed samples, the box projection on the weighted
    # update, and the accel clamp on the executed command all hold every tick, so
    # a short capped run exercises the invariant cheaply.
    params = _config(tmp_path, max_steps=40)
    buf = io.StringIO()
    _run(params, TraceRecorder(buf))

    v_max = params.get_float("v_max")
    omega_max = params.get_float("omega_max")
    moves = _events(buf, "robot_moved")
    assert moves
    for move in moves:
        assert abs(move["data"]["v"]) <= v_max + 1e-9
        assert abs(move["data"]["omega"]) <= omega_max + 1e-9


# --- (c) a fixed seed replays identically; a different seed diverges -----------
def test_same_seed_is_deterministic(tmp_path: Path) -> None:
    (tmp_path / "a").mkdir()
    (tmp_path / "b").mkdir()
    (tmp_path / "c").mkdir()
    buf_a = io.StringIO()
    buf_b = io.StringIO()
    buf_c = io.StringIO()
    _run(_config(tmp_path / "a", seed=1, max_steps=30), TraceRecorder(buf_a))
    _run(_config(tmp_path / "b", seed=1, max_steps=30), TraceRecorder(buf_b))
    _run(_config(tmp_path / "c", seed=2, max_steps=30), TraceRecorder(buf_c))

    poses_a = _moved_poses(buf_a)
    poses_b = _moved_poses(buf_b)
    poses_c = _moved_poses(buf_c)
    assert poses_a
    # Same seed -> bit-for-bit identical closed-loop trajectory.
    assert len(poses_a) == len(poses_b)
    for pa, pb in zip(poses_a, poses_b, strict=True):
        assert pa == pytest.approx(pb, abs=0.0)
    # Different seed -> a different sampled noise stream, so the trajectory differs.
    assert poses_a != poses_c


# --- (d) a lower temperature concentrates the softmax weights ------------------
def test_low_temperature_concentrates_weights(tmp_path: Path) -> None:
    # N_eff = (sum w)^2 / sum(w^2) is the effective sample count; weights are
    # normalized (sum = 1) so N_eff = 1 / sum(w^2). A lower temperature sharpens
    # the softmax toward the min-cost sample (smaller N_eff); a higher temperature
    # flattens it toward uniform (larger N_eff).
    def n_eff(temperature: float, tag: str) -> float:
        sub = tmp_path / tag
        sub.mkdir()
        buf = _single_tick_buffer(_config(sub, temperature=temperature))
        weights = [e["data"]["weight"] for e in _events(buf, "candidate_evaluated")]
        assert weights
        return (sum(weights) ** 2) / sum(w * w for w in weights)

    assert n_eff(0.05, "low") < n_eff(50.0, "high")


# --- (e) a stronger obstacle weight bends the nominal horizon away -------------
def test_obstacle_weight_increases_clearance(tmp_path: Path) -> None:
    # Deterministic single-tick test on a synthetic grid: one obstacle sits just
    # off the straight line from the robot to a goal set straight ahead, within
    # the prediction horizon's reach. With w_obstacle = 0 the nominal horizon
    # heads straight for the goal and passes close to the obstacle; a strong
    # w_obstacle bends that same horizon away, raising its minimum continuous
    # nearest-occupied clearance (the quantity the cost term optimizes). This
    # isolates the obstacle term from any full-episode reach.
    resolution = 0.05
    pixels = np.full((80, 160), 255, dtype=np.uint16)  # 4 m x 8 m free field
    obstacle = (2.0, 2.0)
    half = 0.15
    for dx in np.arange(-half, half, resolution):
        for dy in np.arange(-half, half, resolution):
            col = int(round((obstacle[0] + dx) / resolution - 0.5))
            row = int(round((obstacle[1] + dy) / resolution - 0.5))
            pixels[row, col] = 0
    grid = OccupancyGrid2D(pixels=pixels, resolution=resolution, origin=(0.0, 0.0, 0.0))

    def band_min_clearance(tag: str, w_obstacle: float) -> float:
        sub = tmp_path / tag
        sub.mkdir()
        params = _config(sub, w_obstacle=w_obstacle)
        planner = MppiPlanner(params)
        state = RobotState(pose=(1.0, 1.5, 0.0))
        task = LocalTask(goal=(6.0, 1.5, 0.0))
        buf = io.StringIO()
        dt = params.get_float("control_dt")
        planner.compute_command(grid, state, task, dt, TraceRecorder(buf))
        band = _events(buf, "band_updated")[-1]["band"]
        clearances = [
            d_tilde
            for x, y, *_ in band
            for _, d_tilde in [nearest_occupied(grid, (x, y), 3.0)]
            if d_tilde != float("inf")
        ]
        assert clearances
        return min(clearances)

    assert band_min_clearance("high", 200.0) > band_min_clearance("low", 0.0)


# --- (f) an out-of-range parameter fails load-time validation -----------------
def test_param_validation(tmp_path: Path) -> None:
    with pytest.raises(ParamError):
        _config(tmp_path, temperature=0.0)
    with pytest.raises(ParamError):
        _config(tmp_path, num_samples=0)
