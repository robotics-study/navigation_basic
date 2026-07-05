"""Abstract planner base — the common surface every algorithm implements.

Mirrors the C++ `core/planner.hpp` (`GlobalPlanner<State, Space>`). Two type
params (state + space) let one base serve both discrete and sampling planners
while keeping `space` strongly typed, matching the C++ template signature.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Generic, TypeVar

from .capabilities import Capability, DiscreteSpace, SamplingSpace
from .params import ParamSet
from .trace import TraceRecorder
from .types import Cell, PlanResult, Point, StateT

SpaceT = TypeVar("SpaceT")


class GlobalPlanner(ABC, Generic[StateT, SpaceT]):
    def __init__(self, params: ParamSet) -> None:
        self.params = params

    @property
    @abstractmethod
    def name(self) -> str:
        """Algorithm id; matches the config filename and trace `algorithm` field."""
        ...

    @abstractmethod
    def required_capabilities(self) -> set[Capability]: ...

    @abstractmethod
    def plan(
        self,
        space: SpaceT,
        start: StateT,
        goal: StateT,
        recorder: TraceRecorder | None = None,
    ) -> PlanResult[StateT]: ...


# Convenience aliases mirroring the C++ `DiscretePlanner` / `SamplingPlanner`.
DiscretePlanner = GlobalPlanner[Cell, DiscreteSpace[Cell]]
SamplingPlanner = GlobalPlanner[Point, SamplingSpace[Point]]
