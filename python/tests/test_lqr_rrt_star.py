"""LQR-RRT* (Perez et al. 2012): finds a dynamically-feasible, collision-free
path to the goal, fails gracefully when walled off, validates its params, and its
extension heuristics are genuinely LQR-derived (the metric matrix solves the
Riccati equation, and changing Q/R changes the metric and the resulting path)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml
from conftest import CONFIG_DIR, grid_from, open_grid

from navigation.core.params import ParamError, ParamSet
from navigation.global_planning.sampling.lqr_rrt_star import (
    LQRRRTStar,
    lqr_cost_to_go,
    solve_dlqr,
)
from navigation.maps.occupancy_grid import OccupancyGrid2D

_ALGO = "lqr_rrt_star"


def _capped_config(tmp_path: Path, **overrides: float) -> ParamSet:
    """Real config with per-name overrides so tests stay fast and deterministic."""
    doc = yaml.safe_load((CONFIG_DIR / f"{_ALGO}.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] in overrides:
            entry["default"] = overrides[entry["name"]]
    out = tmp_path / f"{_ALGO}.yaml"
    out.write_text(yaml.safe_dump(doc), encoding="utf-8")
    return ParamSet.from_yaml(out)


def _motion_valid_path(grid: OccupancyGrid2D, path: list[tuple[float, float]]) -> bool:
    return all(grid.is_motion_valid(a, b) for a, b in zip(path, path[1:], strict=False))


def test_reaches_goal_with_feasible_collision_free_path(tmp_path: Path) -> None:
    grid = open_grid(10, 10, seed=7)
    start, goal = (0.75, 0.75), (9.0, 9.0)
    params = _capped_config(
        tmp_path, max_iterations=1500, goal_bias=0.2, goal_tolerance=1.5, seed=3
    )
    result = LQRRRTStar(params).plan(grid, start, goal)
    assert result.success
    assert result.path[0] == pytest.approx(start)
    # The LQR feedback regulates onto the goal rest-state; the projected endpoint is
    # within tolerance of the goal position.
    assert grid.distance(result.path[-1], goal) <= params.get_float("goal_tolerance")
    # The propagated double-integrator trajectory must be collision-free throughout.
    assert _motion_valid_path(grid, result.path)
    assert result.cost > 0.0


def test_no_path_when_walled_off(tmp_path: Path) -> None:
    # Fully occupied middle column: no trajectory can cross it.
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."], seed=3)
    params = _capped_config(tmp_path, max_iterations=300, goal_tolerance=1.0)
    result = LQRRRTStar(params).plan(grid, (0.5, 0.5), (4.5, 4.5))
    assert not result.success
    assert result.path == []
    assert result.cost == 0.0


def test_out_of_range_default_rejected(tmp_path: Path) -> None:
    # r_ctrl default below its declared min must fail validation at load time.
    doc = yaml.safe_load((CONFIG_DIR / f"{_ALGO}.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] == "r_ctrl":
            entry["default"] = 0.0  # below min 0.001
    bad = tmp_path / f"{_ALGO}.yaml"
    bad.write_text(yaml.safe_dump(doc), encoding="utf-8")
    with pytest.raises(ParamError):
        ParamSet.from_yaml(bad)


def test_metric_matrix_solves_the_riccati_equation() -> None:
    # The distance metric is dist(a,b)=(a−b)ᵀS(a−b) with S the LQR cost-to-go. Prove
    # S is genuinely Riccati-derived (not an ad-hoc form) by checking the discrete
    # algebraic Riccati residual S − (Q + AᵀSA − AᵀSB(R+BᵀSB)⁻¹BᵀSA) ≈ 0.
    q_pos, q_vel, r_ctrl, dt = 1.0, 1.0, 1.0, 0.2
    s, k = solve_dlqr(q_pos, q_vel, r_ctrl, dt)
    a = np.array([[1.0, dt], [0.0, 1.0]])
    b = np.array([[0.5 * dt * dt], [dt]])
    q = np.diag([q_pos, q_vel])
    r = np.array([[r_ctrl]])
    bt_p = b.T @ s
    gain = np.linalg.solve(r + bt_p @ b, bt_p @ a)
    residual = q + a.T @ s @ a - (a.T @ s @ b) @ gain - s
    assert float(np.max(np.abs(residual))) < 1e-9
    # The returned gain is the one induced by S (the Riccati fixed point).
    assert np.allclose(k, gain[0])


def test_lqr_metric_is_a_metric_contract() -> None:
    s, _ = solve_dlqr(1.0, 1.0, 1.0, 0.2)
    at_rest = (1.0, 2.0, 0.0, 0.0)
    # Coincident states cost nothing, the identity of the cost geometry.
    assert lqr_cost_to_go(at_rest, at_rest, s) == 0.0
    # Distinct states cost strictly more than zero.
    for x1 in [(4.0, 6.0, 0.0, 0.0), (1.0, 2.0, 1.0, -1.0), (0.0, 0.0, 0.5, 0.5)]:
        assert lqr_cost_to_go(at_rest, x1, s) > 0.0


def test_control_weight_changes_the_metric() -> None:
    # A different R (control weight) yields a different Riccati S, hence a different
    # cost-to-go for the same maneuver — the metric is derived from the LQR, not fixed.
    a, b = (0.0, 0.0, 0.0, 0.0), (3.0, 2.0, 1.0, -1.0)
    s_cheap, _ = solve_dlqr(1.0, 1.0, 0.2, 0.2)
    s_dear, _ = solve_dlqr(1.0, 1.0, 5.0, 0.2)
    assert lqr_cost_to_go(a, b, s_dear) != pytest.approx(lqr_cost_to_go(a, b, s_cheap))


def test_control_weight_changes_the_path(tmp_path: Path) -> None:
    # End-to-end: a heavier control penalty reshapes the LQR steering, so the planned
    # trajectory differs. Same map/seed/iterations, only r_ctrl changes.
    grid = open_grid(10, 10, seed=5)
    start, goal = (0.75, 0.75), (9.0, 9.0)
    cheap = LQRRRTStar(
        _capped_config(tmp_path, max_iterations=1200, seed=5, r_ctrl=0.2, goal_tolerance=1.5)
    ).plan(grid, start, goal)
    dear = LQRRRTStar(
        _capped_config(tmp_path, max_iterations=1200, seed=5, r_ctrl=50.0, goal_tolerance=1.5)
    ).plan(grid, start, goal)
    assert cheap.success and dear.success
    assert _motion_valid_path(grid, cheap.path)
    assert _motion_valid_path(grid, dear.path)
    # Different LQR gains ⇒ different realised trajectories/costs.
    assert cheap.path != dear.path
