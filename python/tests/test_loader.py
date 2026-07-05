"""Map / scenario loading: type dispatch, path resolution, capability wiring."""

from __future__ import annotations

from pathlib import Path

import pytest
from conftest import REPO_ROOT

from nav_study.core.capabilities import Capability
from nav_study.maps.loader import load_map, load_scenario
from nav_study.maps.occupancy_grid import OccupancyGrid2D

_MAZE = REPO_ROOT / "maps" / "grid" / "maze01.yaml"
_SCENARIO = REPO_ROOT / "maps" / "scenarios" / "maze01_s1.yaml"


def test_load_occupancy_grid() -> None:
    grid = load_map(_MAZE, connectivity=4)
    assert isinstance(grid, OccupancyGrid2D)
    assert grid.supports(Capability.DISCRETE_SPACE)
    assert (grid.height, grid.width) == (20, 20)
    # Manhattan heuristic proves the 4-connectivity flag reached the grid.
    assert grid.heuristic((0, 0), (2, 3)) == pytest.approx(5.0)


def test_load_scenario_resolves_map_path() -> None:
    scenario = load_scenario(_SCENARIO)
    assert Path(scenario.map_path) == _MAZE.resolve()
    assert scenario.start == (0.75, 0.75)
    assert scenario.goal == (9.25, 9.25)


def test_unsupported_map_type_raises(tmp_path: Path) -> None:
    p = tmp_path / "g.yaml"
    p.write_text("type: graph\nnodes: []\nedges: []\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_map(p)


def test_multiagent_scenario_rejected(tmp_path: Path) -> None:
    p = tmp_path / "s.yaml"
    p.write_text("map: ../grid/maze01.yaml\nagents: []\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_scenario(p)
