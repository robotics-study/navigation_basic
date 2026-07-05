"""A* search — f = g + w*h with an admissible grid heuristic.

Hart, Nilsson & Raphael (1968). w = 1 is optimal; w > 1 is weighted A* (Pohl
1970): faster, bounded-suboptimal. w is read from the `heuristic_weight` param.
"""

from __future__ import annotations

from nav_study.core.params import ParamSet

from ._bestfirst import _BestFirstSearch


class AStar(_BestFirstSearch):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._heuristic_weight = params.get_float("heuristic_weight")

    @property
    def name(self) -> str:
        return "astar"

    def _uses_heuristic(self) -> bool:
        return True

    def _weight(self) -> float:
        return self._heuristic_weight
