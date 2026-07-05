"""Planner wiring the demos/bench rely on: name matches config, correct caps."""

from __future__ import annotations

import pytest
from conftest import config

from nav_study.core.capabilities import Capability
from nav_study.global_planning.astar import AStar
from nav_study.global_planning.bfs import BFS
from nav_study.global_planning.dijkstra import Dijkstra
from nav_study.global_planning.fast_rrt import FastRRT
from nav_study.global_planning.rrt import RRT
from nav_study.global_planning.rrt_star import RRTStar

_DISCRETE = [("bfs", BFS), ("dijkstra", Dijkstra), ("astar", AStar)]
_SAMPLING = [("rrt", RRT), ("rrt_star", RRTStar), ("fast_rrt", FastRRT)]


@pytest.mark.parametrize("algo,cls", _DISCRETE)
def test_discrete_contract(algo: str, cls: type) -> None:
    planner = cls(config(algo))
    assert planner.name == config(algo).algorithm == algo
    assert planner.required_capabilities() == {Capability.DISCRETE_SPACE}


@pytest.mark.parametrize("algo,cls", _SAMPLING)
def test_sampling_contract(algo: str, cls: type) -> None:
    planner = cls(config(algo))
    assert planner.name == config(algo).algorithm == algo
    assert planner.required_capabilities() == {Capability.SAMPLING_SPACE}
