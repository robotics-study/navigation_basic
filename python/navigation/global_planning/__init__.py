"""Global planning algorithms. Depend on `core` abstractions only.

Organised by lineage into two sub-packages:
- `search` — graph-search planners (`DiscreteSpace`): BFS, Dijkstra, A*.
- `sampling` — sampling-based planners (`SamplingSpace`): RRT family, PRM
  family, FMT*, BIT*.

No algorithm module imports another; shared skeletons live in each family's
private `_discrete` / `_bestfirst` (search) and `_sampling` / `_roadmap`
(sampling) helpers. The flat public names below are re-exported for stable
`from navigation.global_planning import AStar, RRTStar, ...` access.
"""

from .sampling import PRM, RRT, BITStar, FastRRT, FMTStar, PRMStar, RRTStar
from .search import BFS, AStar, Dijkstra

__all__ = [
    "BFS",
    "Dijkstra",
    "AStar",
    "RRT",
    "RRTStar",
    "FastRRT",
    "PRM",
    "PRMStar",
    "FMTStar",
    "BITStar",
]
