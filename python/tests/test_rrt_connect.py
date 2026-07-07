"""RRT-Connect: bidirectional trees meet on open space, fail when walled off."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml
from conftest import CONFIG_DIR, grid_from, open_grid, write_config

from navigation.core.params import ParamError, ParamSet
from navigation.global_planning import RRTConnect
from navigation.maps.occupancy_grid import OccupancyGrid2D


def _capped_config(tmp_path: Path, max_iter: int) -> ParamSet:
    """Real rrt_connect config with a smaller budget so the no-path test stays fast."""
    doc = yaml.safe_load((CONFIG_DIR / "rrt_connect.yaml").read_text())
    for entry in doc["params"]:
        if entry["name"] == "max_iterations":
            entry["default"] = max_iter
    out = tmp_path / "rrt_connect.yaml"
    out.write_text(yaml.safe_dump(doc), encoding="utf-8")
    return ParamSet.from_yaml(out)


def _motion_valid_path(grid: OccupancyGrid2D, path: list[tuple[float, float]]) -> bool:
    return all(grid.is_motion_valid(a, b) for a, b in zip(path, path[1:], strict=False))


def test_reaches_goal_on_open_grid(tmp_path: Path) -> None:
    grid = open_grid(10, 10, seed=7)
    start, goal = (0.5, 0.5), (9.0, 9.0)
    params = _capped_config(tmp_path, max_iter=2000)
    result = RRTConnect(params).plan(grid, start, goal)
    assert result.success
    # Both roots are exact tree nodes, so the spliced path pins to start and goal.
    assert result.path[0] == pytest.approx(start)
    assert result.path[-1] == pytest.approx(goal)
    assert _motion_valid_path(grid, result.path)
    assert result.cost > 0.0


def test_no_path_when_walled_off(tmp_path: Path) -> None:
    # Fully occupied middle column: the two trees can never connect.
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."], seed=3)
    params = _capped_config(tmp_path, max_iter=400)
    result = RRTConnect(params).plan(grid, (0.5, 0.5), (4.5, 4.5))
    assert not result.success
    assert result.path == []
    assert result.cost == 0.0


def test_out_of_range_step_size_rejected(tmp_path: Path) -> None:
    # step_size above its declared max must fail at load time, not silently clamp.
    bad = write_config(
        tmp_path / "rrt_connect.yaml",
        "rrt_connect",
        [{"name": "step_size", "type": "float", "default": 200.0, "min": 0.01, "max": 100.0,
          "description": "over max"}],
    )
    with pytest.raises(ParamError):
        ParamSet.from_yaml(bad)
