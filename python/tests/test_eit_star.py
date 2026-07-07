"""EIT* — reaches the goal on open space, fails when walled off, near-optimal.

Exercises the effort-informed batch planner in isolation (imported directly from
its module so the suite does not depend on the package re-export).
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml
from conftest import CONFIG_DIR, grid_from, open_grid

from navigation.core.params import ParamError, ParamSet
from navigation.global_planning.sampling.eit_star import EITStar
from navigation.maps.occupancy_grid import OccupancyGrid2D


def _eit_config(tmp_path: Path, **overrides: int) -> ParamSet:
    """Real eit_star config with reduced batch budgets so the suite stays fast."""
    doc = yaml.safe_load((CONFIG_DIR / "eit_star.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] in overrides:
            entry["default"] = overrides[entry["name"]]
    out = tmp_path / "eit_star.yaml"
    out.write_text(yaml.safe_dump(doc), encoding="utf-8")
    return ParamSet.from_yaml(out)


def _motion_valid_path(grid: OccupancyGrid2D, path: list[tuple[float, float]]) -> bool:
    return all(grid.is_motion_valid(a, b) for a, b in zip(path, path[1:], strict=False))


def test_reaches_goal_on_open_grid(tmp_path: Path) -> None:
    grid = open_grid(10, 10, seed=7)
    start, goal = (0.5, 0.5), (9.0, 9.0)
    params = _eit_config(tmp_path, batch_size=150, max_batches=4)
    result = EITStar(params).plan(grid, start, goal)
    assert result.success
    # start/goal are permanent sample nodes, so the path pins to them exactly.
    assert result.path[0] == pytest.approx(start)
    assert result.path[-1] == pytest.approx(goal)
    assert _motion_valid_path(grid, result.path)
    assert result.cost > 0.0


def test_no_path_when_walled_off(tmp_path: Path) -> None:
    # Fully occupied middle column: no motion can cross it.
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."], seed=3)
    params = _eit_config(tmp_path, batch_size=80, max_batches=3)
    result = EITStar(params).plan(grid, (0.5, 0.5), (4.5, 4.5))
    assert not result.success
    assert result.path == []
    assert result.cost == 0.0


def test_near_straight_line_on_open_grid(tmp_path: Path) -> None:
    # As an asymptotically optimal batch planner, EIT* should return a cost close
    # to the straight-line lower bound on obstacle-free space.
    grid = open_grid(12, 12, seed=5)
    start, goal = (0.5, 0.5), (11.0, 11.0)
    lower_bound = grid.distance(start, goal)
    result = EITStar(_eit_config(tmp_path, batch_size=200, max_batches=4)).plan(grid, start, goal)
    assert result.success
    assert result.cost <= lower_bound * 1.15


def test_bad_default_param_throws(tmp_path: Path) -> None:
    # step_size default below its declared min must fail at load time.
    bad = _write_bad_step_size(tmp_path)
    with pytest.raises(ParamError):
        ParamSet.from_yaml(bad)


def _write_bad_step_size(tmp_path: Path) -> Path:
    doc = yaml.safe_load((CONFIG_DIR / "eit_star.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] == "step_size":
            entry["default"] = 0.0  # below declared min 0.01
    out = tmp_path / "eit_star_bad.yaml"
    out.write_text(yaml.safe_dump(doc), encoding="utf-8")
    return out
