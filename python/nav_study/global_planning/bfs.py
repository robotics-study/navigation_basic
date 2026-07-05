"""Breadth-first search over a DiscreteSpace (Cormen et al.; fewest-edge path).

Each node is enqueued once at first discovery and its parent fixed to the
discoverer, which is what yields the fewest-edge path.
"""

from __future__ import annotations

import time
from collections import deque

from nav_study.core.capabilities import Capability, DiscreteSpace
from nav_study.core.planner import GlobalPlanner
from nav_study.core.trace import TraceRecorder
from nav_study.core.types import Cell, PlanResult, PlanStats

from ._discrete import emit_finish, reconstruct


class BFS(GlobalPlanner[Cell, "DiscreteSpace[Cell]"]):
    @property
    def name(self) -> str:
        return "bfs"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.DISCRETE_SPACE}

    def plan(
        self,
        space: DiscreteSpace[Cell],
        start: Cell,
        goal: Cell,
        recorder: TraceRecorder | None = None,
    ) -> PlanResult[Cell]:
        t0 = time.monotonic()
        parent: dict[Cell, Cell] = {}
        cost_to: dict[Cell, float] = {start: 0.0}
        discovered: set[Cell] = {start}
        frontier: deque[Cell] = deque([start])
        expanded = 0
        found = False
        while frontier:
            cell = frontier.popleft()
            expanded += 1
            if recorder is not None:
                recorder.node_expanded(cell, cost_to[cell])
            if cell == goal:
                found = True
                break
            for nb, edge_cost in space.neighbors(cell):
                if nb in discovered:
                    continue
                discovered.add(nb)
                parent[nb] = cell
                cost_to[nb] = cost_to[cell] + edge_cost
                if recorder is not None:
                    recorder.edge_added(nb, cell, edge_cost)
                frontier.append(nb)

        runtime = time.monotonic() - t0
        if not found:
            emit_finish(recorder, False, [], 0.0, expanded, runtime)
            return PlanResult(success=False, stats=PlanStats(expanded_nodes=expanded))
        path = reconstruct(parent, start, goal)
        cost = cost_to[goal]
        emit_finish(recorder, True, path, cost, expanded, runtime)
        return PlanResult(True, path, cost, PlanStats(expanded_nodes=expanded))
