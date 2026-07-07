"""SST / SST* (Li, Littlefield & Bekris 2016): forward-propagation kinodynamic
planner that reaches the goal on open space, fails when walled off, and keeps the
active tree sparse via the witness set (which SST* shrinks over iterations)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest
import yaml
from conftest import CONFIG_DIR, grid_from, open_grid

from navigation.core.params import ParamError, ParamSet
from navigation.global_planning.sampling.sst import SST
from navigation.maps.occupancy_grid import OccupancyGrid2D

_ALGO = "sst"


def _config(tmp_path: Path, **overrides: Any) -> ParamSet:
    """Real config with selected defaults overridden (smaller budgets keep tests fast)."""
    doc = yaml.safe_load((CONFIG_DIR / f"{_ALGO}.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] in overrides:
            entry["default"] = overrides[entry["name"]]
    out = tmp_path / f"{_ALGO}.yaml"
    out.write_text(yaml.safe_dump(doc), encoding="utf-8")
    return ParamSet.from_yaml(out)


def _collision_free(grid: OccupancyGrid2D, path: list[tuple[float, float]]) -> bool:
    return all(grid.is_state_valid(p) for p in path) and all(
        grid.is_motion_valid(a, b) for a, b in zip(path, path[1:], strict=False)
    )


def test_finds_dynamically_feasible_path(tmp_path: Path) -> None:
    grid = open_grid(10, 10, seed=1)
    start, goal = (0.5, 0.5), (8.5, 8.5)
    params = _config(tmp_path, max_iterations=20000)
    result = SST(params).plan(grid, start, goal)
    assert result.success
    assert result.path[0] == pytest.approx(start)
    # Goal reached within tolerance (position-only; the unicycle has no goal heading).
    assert grid.distance(result.path[-1], goal) <= params.get_float("goal_tolerance")
    assert _collision_free(grid, result.path)
    assert result.cost > 0.0
    # Dynamic feasibility: the path is a densely-sampled propagated trajectory, not
    # straight steer jumps — every consecutive chord stays within the arc spacing.
    max_step = max(grid.distance(a, b) for a, b in zip(result.path, result.path[1:], strict=False))
    assert max_step <= 0.25


def test_no_path_when_walled_off(tmp_path: Path) -> None:
    # Fully occupied middle column: no propagated arc can cross it.
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."], seed=3)
    params = _config(tmp_path, max_iterations=500)
    result = SST(params).plan(grid, (0.5, 0.5), (4.5, 4.5))
    assert not result.success
    assert result.path == []
    assert result.cost == 0.0


def test_out_of_range_default_rejected(tmp_path: Path) -> None:
    # goal_bias default above its declared max must fail validation at load time.
    doc = yaml.safe_load((CONFIG_DIR / f"{_ALGO}.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] == "goal_bias":
            entry["default"] = 2.0
    bad = tmp_path / f"{_ALGO}.yaml"
    bad.write_text(yaml.safe_dump(doc), encoding="utf-8")
    with pytest.raises(ParamError):
        ParamSet.from_yaml(bad)


def test_witness_set_bounds_active_growth(tmp_path: Path) -> None:
    # The witness set caps how many nodes stay active: over a large budget the active
    # tree stays sparse (far below the iteration count), while total propagations added
    # exceed it because dominated representatives are pruned. A naive RRT would keep
    # every added node, so its tree would grow ~linearly in iterations instead.
    grid = open_grid(12, 12, seed=2)
    result = SST(_config(tmp_path, max_iterations=20000)).plan(grid, (0.5, 0.5), (10.5, 10.5))
    assert result.success
    assert result.stats.tree_size < result.stats.iterations // 10  # sparse active set
    assert result.stats.expanded_nodes > result.stats.tree_size  # pruning happened
    assert result.stats.tree_size <= 800


def test_sst_star_shrinking_schedule_runs(tmp_path: Path) -> None:
    # SST* shrinks delta_bn / delta_s over iterations; it must run end-to-end without
    # error and grow a tree (finer sparsification keeps more active nodes than SST).
    grid = open_grid(10, 10, seed=1)
    result = SST(_config(tmp_path, sst_star=True, max_iterations=4000)).plan(
        grid, (0.5, 0.5), (8.5, 8.5)
    )
    assert isinstance(result.success, bool)
    assert result.stats.iterations == 4000
    assert result.stats.tree_size > 1
