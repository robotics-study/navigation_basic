"""Theta* — any-angle path planning on grids.

Nash, Daniel, Koenig & Felner (2007). Like A*, but on each relaxation it tries a
straight line-of-sight shortcut from the expanded node's parent (path 2), so the
returned path follows true straight segments instead of being locked to grid
edges. The heuristic is Euclidean (straight-line), consistent with the any-angle
g-values; w > 1 is weighted Theta* (Pohl 1970).

Does NOT subclass ``_BestFirstSearch``: that skeleton hardcodes the octile
heuristic and standard grid relaxation, both wrong for any-angle. Reconstruction
and finish-emit are shared via ``_discrete``.
"""

from __future__ import annotations

import heapq
import itertools
import math
import time

from navigation.core.capabilities import Capability, LineOfSightSpace
from navigation.core.params import ParamSet
from navigation.core.planner import GlobalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import Cell, PlanResult, PlanStats

from ._discrete import emit_finish, reconstruct


def _euclid(a: Cell, b: Cell) -> float:
    # Euclidean distance on integer cell-index deltas. math.sqrt — NOT math.hypot:
    # hypot is relaxed-accuracy and can differ 1 ULP from the C++ run, diverging
    # f-values / the trace stream the bench compares across languages. sqrt is
    # correctly-rounded, and sqrt(2.0) exactly equals the diagonal edge cost from
    # neighbors(), so a diagonal shortcut (path 2) and a diagonal step (path 1)
    # are bit-equal.
    dr = float(a[0] - b[0])
    dc = float(a[1] - b[1])
    return math.sqrt(dr * dr + dc * dc)


class ThetaStar(GlobalPlanner[Cell, "LineOfSightSpace[Cell]"]):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._heuristic_weight = params.get_float("heuristic_weight")

    @property
    def name(self) -> str:
        return "theta_star"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.DISCRETE_SPACE, Capability.LINE_OF_SIGHT_SPACE}

    def plan(
        self,
        space: LineOfSightSpace[Cell],
        start: Cell,
        goal: Cell,
        recorder: TraceRecorder | None = None,
    ) -> PlanResult[Cell]:
        t0 = time.monotonic()
        w = self._heuristic_weight

        def h(c: Cell) -> float:
            return w * _euclid(c, goal)

        g: dict[Cell, float] = {start: 0.0}
        parent: dict[Cell, Cell] = {start: start}  # self-parent: grandparent + reconstruct stop
        closed: set[Cell] = set()
        counter = itertools.count()  # stable tie-break, keeps heap entries comparable
        frontier: list[tuple[float, int, Cell]] = [(h(start), next(counter), start)]
        expanded = 0
        found = False
        while frontier:
            _, _, s = heapq.heappop(frontier)
            if s in closed:
                continue
            closed.add(s)
            expanded += 1
            if recorder is not None:
                recorder.node_expanded(s, g[s])
            if s == goal:
                found = True
                break
            p = parent[s]
            for s2, edge_cost in space.neighbors(s):
                if s2 in closed:
                    continue
                if space.line_of_sight(p, s2):
                    # Path 2 — any-angle shortcut straight from the grandparent.
                    ecost = _euclid(p, s2)
                    cand = g[p] + ecost
                    par = p
                else:
                    # Path 1 — standard grid step through s.
                    ecost = edge_cost
                    cand = g[s] + ecost
                    par = s
                if s2 not in g or cand < g[s2]:
                    g[s2] = cand
                    parent[s2] = par
                    if recorder is not None:
                        recorder.candidate_evaluated(s2, cand)
                        recorder.edge_added(s2, par, ecost)
                    heapq.heappush(frontier, (cand + h(s2), next(counter), s2))

        runtime = time.monotonic() - t0
        if not found:
            emit_finish(recorder, False, [], 0.0, expanded, runtime)
            return PlanResult(success=False, stats=PlanStats(expanded_nodes=expanded))
        path = reconstruct(parent, start, goal)
        cost = g[goal]  # any-angle path cost; adjacent-edge summing would be wrong for jumps.
        emit_finish(recorder, True, path, cost, expanded, runtime)
        return PlanResult(True, path, cost, PlanStats(expanded_nodes=expanded))
