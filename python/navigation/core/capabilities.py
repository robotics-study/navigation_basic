"""Capability model: algorithms require capabilities, map types provide them.

Mirrors the C++ `core/capabilities.hpp`. Capabilities are structural Protocols
so one concrete map can satisfy several without a class hierarchy, and a planner
depends only on the capability it needs (never on a concrete map class).
"""

from __future__ import annotations

import enum
from abc import ABC, abstractmethod
from typing import Protocol, TypeVar

from .types import Footprint  # same core layer; types.py does not import capabilities.py

StateT = TypeVar("StateT")
# SE2CollisionSpace consumes its state only as a method argument (never returns it), so
# its type var must be contravariant — an invariant one trips mypy's protocol check.
StateT_contra = TypeVar("StateT_contra", contravariant=True)


class Capability(enum.Enum):
    DISCRETE_SPACE = "discrete_space"
    SAMPLING_SPACE = "sampling_space"
    # Declared for future local planners; no map here implements it (no dead impl).
    OBSTACLE_QUERY = "obstacle_query"
    LINE_OF_SIGHT_SPACE = "line_of_sight_space"
    DYNAMIC_GRID_SPACE = "dynamic_grid_space"
    SE2_COLLISION_SPACE = "se2_collision_space"


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


class DynamicGridSpace(Protocol[StateT]):
    """Dynamic-replanning search view for D* Lite (Koenig & Likhachev 2002).

    Standalone (NOT a DiscreteSpace): neighbor enumeration takes a *belief* — the
    planner's own set of known blocked cells — instead of reading ground truth, so
    it cannot share the truth-baked ``neighbors()``. Structural, so one concrete
    grid satisfies DiscreteSpace and this without a class hierarchy."""

    def passable_neighbors(
        self, s: StateT, blocked: set[StateT]
    ) -> list[tuple[StateT, float]]:
        """(successor, edge_cost) pairs traversable under the belief ``blocked``:
        in bounds and not in ``blocked``, same corner-cut rule as ``neighbors``."""
        ...

    def is_blocked(self, s: StateT) -> bool:
        """Ground-truth sensor: True iff ``s`` is occupied or out of bounds."""
        ...


class SE2CollisionSpace(Protocol[StateT_contra]):
    """Continuous SE(2) collision view for kinodynamic planners (Hybrid A*, Dolgov
    et al. 2008). Standalone: the planner owns its motion model + heuristic and needs
    ONLY a footprint collision test at a world pose. Structural, so one concrete grid
    satisfies this and the discrete/sampling views without a class hierarchy."""

    def is_collision(self, footprint: Footprint, pose: StateT_contra) -> bool: ...


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
