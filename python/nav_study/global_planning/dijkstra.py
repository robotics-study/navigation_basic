"""Dijkstra's algorithm — uniform-cost search, no heuristic (Dijkstra 1959)."""

from __future__ import annotations

from ._bestfirst import _BestFirstSearch


class Dijkstra(_BestFirstSearch):
    @property
    def name(self) -> str:
        return "dijkstra"

    def _uses_heuristic(self) -> bool:
        return False
