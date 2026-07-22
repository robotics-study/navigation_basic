"""Abstract planner base — the common surface every algorithm implements.

Mirrors the C++ `core/planner.hpp` (`GlobalPlanner<State, Space>`). Two type
params (state + space) let one base serve both discrete and sampling planners
while keeping `space` strongly typed, matching the C++ template signature.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Generic, TypeVar

from .capabilities import (
    Capability,
    DiscreteSpace,
    DynamicGridSpace,
    LineOfSightSpace,
    ObstacleQuery,
    SamplingSpace,
    SE2CollisionSpace,
)
from .params import ParamSet
from .trace import TraceRecorder
from .types import Cell, LocalTask, PlanResult, Point, Pose, RobotState, StateT, VelocityCommand

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


class LocalPlanner(ABC, Generic[SpaceT]):
    """One control tick: (robot state, task, map capability) -> velocity command.

    Single type param (unlike GlobalPlanner's state+space): every local planner's
    state is the same SE(2) `RobotState`, so a second type var would just be dead
    weight on every implementation.
    """

    def __init__(self, params: ParamSet) -> None:
        self.params = params

    @property
    @abstractmethod
    def name(self) -> str:
        """Algorithm id; matches the config filename and trace `algorithm` field."""
        ...

    @abstractmethod
    def required_capabilities(self) -> set[Capability]: ...

    def reset(self) -> None:
        """Reset internal state at the start of an episode. Stateless planners
        (no internal progress cursor / band) can leave this as a no-op."""
        return None

    def requires_reference_path(self) -> bool:
        """True if the planner cannot run without a reference_path. hot-path
        `compute_command` never raises (C++ convention), so callers check this
        before the first tick and reject a missing path at assembly time."""
        return False

    @abstractmethod
    def compute_command(
        self,
        space: SpaceT,
        state: RobotState,
        task: LocalTask,
        dt: float,
        recorder: TraceRecorder | None = None,
    ) -> VelocityCommand: ...


# Convenience aliases mirroring the C++ `DiscretePlanner` / `SamplingPlanner`.
DiscretePlanner = GlobalPlanner[Cell, DiscreteSpace[Cell]]
SamplingPlanner = GlobalPlanner[Point, SamplingSpace[Point]]
LineOfSightPlanner = GlobalPlanner[Cell, LineOfSightSpace[Cell]]
DynamicGridPlanner = GlobalPlanner[Cell, DynamicGridSpace[Cell]]
SE2CollisionPlanner = GlobalPlanner[Pose, SE2CollisionSpace[Pose]]
ObstacleLocalPlanner = LocalPlanner[ObstacleQuery]
