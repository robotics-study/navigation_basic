"""Language-neutral state / result types shared by every planner.

Mirrors the C++ `core/types.hpp`. Discrete state is a grid cell (row, col) of
ints; sampling state is a world point (x, y) of floats. Never blur the two.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Generic, TypeVar

# Discrete state: grid index (row, col), row 0 = top image row.
Cell = tuple[int, int]
# Sampling state: world point (x, y) in meters.
Point = tuple[float, float]
# SE(2) planning state: world pose (x,y meters, theta radians). A 3-tuple so it
# serializes to trace as [x, y, theta] via list(); distinct from the 2-tuple Point.
Pose = tuple[float, float, float]

StateT = TypeVar("StateT")


@dataclass(frozen=True)
class Footprint:
    # Robot collision footprint — inscribed disc only (orientation-invariant, so
    # collision depends only on (x,y)). A polygon variant can extend this later.
    inscribed_radius: float  # meters


@dataclass
class PlanStats:
    expanded_nodes: int = 0
    samples: int = 0
    iterations: int = 0
    tree_size: int = 0


@dataclass
class PlanResult(Generic[StateT]):
    success: bool
    path: list[StateT] = field(default_factory=list)
    cost: float = 0.0
    stats: PlanStats = field(default_factory=PlanStats)
