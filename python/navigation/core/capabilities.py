"""Capability model: algorithms require capabilities, map types provide them.

Mirrors the C++ `core/capabilities.hpp`. Capabilities are structural Protocols
so one concrete map can satisfy several without a class hierarchy, and a planner
depends only on the capability it needs (never on a concrete map class).
"""

from __future__ import annotations

import enum
from abc import ABC, abstractmethod
from typing import Protocol, TypeVar

StateT = TypeVar("StateT")


class Capability(enum.Enum):
    DISCRETE_SPACE = "discrete_space"
    SAMPLING_SPACE = "sampling_space"
    # Declared for future local planners; no map here implements it (no dead impl).
    OBSTACLE_QUERY = "obstacle_query"
    LINE_OF_SIGHT_SPACE = "line_of_sight_space"


class DiscreteSpace(Protocol[StateT]):
    """Graph-search view: enumerable successors + admissible heuristic."""

    def neighbors(self, s: StateT) -> list[tuple[StateT, float]]:
        """Return (successor, edge_cost) pairs reachable from ``s``."""
        ...

    def heuristic(self, a: StateT, b: StateT) -> float: ...


class LineOfSightSpace(DiscreteSpace[StateT], Protocol[StateT]):
    """DiscreteSpace that also answers collision-free straight-segment queries
    (any-angle Theta*, Nash et al. 2007). Structural, so one concrete grid
    satisfies DiscreteSpace and this without a class hierarchy."""

    def line_of_sight(self, a: StateT, b: StateT) -> bool: ...


class SamplingSpace(Protocol[StateT]):
    """Sampling-based view: draw states and test local motions."""

    def sample(self) -> StateT: ...

    def is_state_valid(self, s: StateT) -> bool: ...

    def is_motion_valid(self, a: StateT, b: StateT) -> bool: ...

    def distance(self, a: StateT, b: StateT) -> float: ...

    def steer(self, a: StateT, b: StateT, eta: float) -> StateT: ...


class MapBase(ABC):
    """Base for concrete maps. Owns the single `supports` implementation."""

    @abstractmethod
    def capabilities(self) -> set[Capability]: ...

    def supports(self, c: Capability) -> bool:
        return c in self.capabilities()
