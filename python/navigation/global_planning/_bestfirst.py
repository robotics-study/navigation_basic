"""Best-first search skeleton for Dijkstra / A*.

Both settle nodes from a g-ordered priority queue with edge relaxation and differ
only in the priority key: Dijkstra uses f = g (Dijkstra 1959); A* uses
f = g + w*h with an admissible heuristic (Hart, Nilsson & Raphael 1968; the
weight w generalizes to weighted A*, Pohl 1970). A lazy queue (stale entries
skipped on pop) avoids a decrease-key structure.
"""

from __future__ import annotations

import heapq
import itertools
import time
from abc import abstractmethod

from navigation.core.capabilities import Capability, DiscreteSpace
from navigation.core.planner import GlobalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import Cell, PlanResult, PlanStats

from ._discrete import emit_finish, reconstruct


class _BestFirstSearch(GlobalPlanner[Cell, "DiscreteSpace[Cell]"]):
    def required_capabilities(self) -> set[Capability]:
        return {Capability.DISCRETE_SPACE}

    @abstractmethod
    def _uses_heuristic(self) -> bool: ...

    def _weight(self) -> float:
        return 1.0

    def plan(
        self,
        space: DiscreteSpace[Cell],
        start: Cell,
        goal: Cell,
        recorder: TraceRecorder | None = None,
    ) -> PlanResult[Cell]:
        t0 = time.monotonic()
        use_h = self._uses_heuristic()
        weight = self._weight()
        g: dict[Cell, float] = {start: 0.0}
        parent: dict[Cell, Cell] = {}
        closed: set[Cell] = set()
        counter = itertools.count()  # stable tie-break, keeps heap entries comparable
        h0 = weight * space.heuristic(start, goal) if use_h else 0.0
        frontier: list[tuple[float, int, Cell]] = [(h0, next(counter), start)]
        expanded = 0
        found = False
        while frontier:
            _, _, cell = heapq.heappop(frontier)
            if cell in closed:
                continue
            closed.add(cell)
            expanded += 1
            if recorder is not None:
                recorder.node_expanded(cell, g[cell])
            if cell == goal:
                found = True
                break
            for nb, edge_cost in space.neighbors(cell):
                if nb in closed:
                    continue
                tentative = g[cell] + edge_cost
                if nb not in g or tentative < g[nb]:
                    g[nb] = tentative
                    parent[nb] = cell
                    if recorder is not None:
                        # emit only on a successful relaxation: state = relaxed node,
                        # cost = its improved tentative g.
                        recorder.candidate_evaluated(nb, tentative)
                        recorder.edge_added(nb, cell, edge_cost)
                    f = tentative + (weight * space.heuristic(nb, goal) if use_h else 0.0)
                    heapq.heappush(frontier, (f, next(counter), nb))

        runtime = time.monotonic() - t0
        if not found:
            emit_finish(recorder, False, [], 0.0, expanded, runtime)
            return PlanResult(success=False, stats=PlanStats(expanded_nodes=expanded))
        path = reconstruct(parent, start, goal)
        cost = g[goal]
        emit_finish(recorder, True, path, cost, expanded, runtime)
        return PlanResult(True, path, cost, PlanStats(expanded_nodes=expanded))
