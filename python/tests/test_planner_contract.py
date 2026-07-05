"""Planner wiring the demos/bench rely on: name matches config, correct caps."""

from __future__ import annotations

import pytest
from conftest import config

from navigation.core.capabilities import Capability
from navigation.global_planning import BFS, RRT, AStar, Dijkstra, FastRRT, RRTStar

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
