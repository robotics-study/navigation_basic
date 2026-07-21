"""Lazy Theta* — any-angle path planning with deferred line-of-sight.

Nash & Koenig (2010). Structurally identical to Theta* (Nash et al. 2007) but it
changes *when* line of sight is checked. On generating a successor it
OPTIMISTICALLY assumes the grandparent is visible (Path 2), setting
parent(s2)=parent(s) and g(s2)=g(parent(s))+euclid without any check. The single
line-of-sight query is deferred to ``set_vertex`` when the vertex is popped for
expansion: if the assumed parent is not actually visible, the parent is repaired
to the cheapest already-settled grid neighbour (Path 1 is a valid adjacent move,
so the generator is always a visible fallback). This yields one line-of-sight
check per *expanded vertex* instead of one per *edge*. Per instance the returned
path can differ slightly from Theta*'s in either direction (the optimistic
assumption changes tie-breaking), but path lengths stay comparable in aggregate
("without an increase in path length", Nash & Koenig 2010) — and far fewer checks.

Does NOT subclass ``_BestFirstSearch`` (its octile heuristic + grid relaxation are
wrong for any-angle) and does NOT import Theta* (algorithm modules must not depend
on each other). Reconstruction and finish-emit are shared via ``_discrete``.
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
    # are bit-equal. Defined locally, not imported from theta_star, because
    # algorithm modules must not depend on one another.
    dr = float(a[0] - b[0])
    dc = float(a[1] - b[1])
    return math.sqrt(dr * dr + dc * dc)


class LazyThetaStar(GlobalPlanner[Cell, "LineOfSightSpace[Cell]"]):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._heuristic_weight = params.get_float("heuristic_weight")

    @property
    def name(self) -> str:
        return "lazy_theta_star"

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
            # set_vertex — the deferred line-of-sight check (Nash & Koenig 2010). The
            # parent was assumed visible when s was generated; verify it only now, once
            # per expanded vertex.
            p = parent[s]
            if p != s and not space.line_of_sight(p, s):
                # Repair: the optimistic grandparent is not actually visible, so adopt
                # the cheapest already-settled grid neighbour as parent. Path 1 between
                # adjacent cells is always a valid move, so the generator (in `closed`)
                # is a guaranteed visible fallback.
                best_g = math.inf
                best_par = p
                best_cost = 0.0
                for s3, edge_cost in space.neighbors(s):
                    if s3 in closed:
                        cand = g[s3] + edge_cost
                        if cand < best_g:
                            best_g = cand
                            best_par = s3
                            best_cost = edge_cost
                g[s] = best_g
                parent[s] = best_par
                p = best_par
                if recorder is not None:
                    # Surface the lazy repair to the visualizer: the deferred check
                    # rejected the optimistic parent, so re-emit s with its real
                    # (grid-neighbour) parent via the existing relaxation events.
                    recorder.candidate_evaluated(s, best_g)
                    recorder.edge_added(s, best_par, best_cost)
            closed.add(s)
            expanded += 1
            if recorder is not None:
                recorder.node_expanded(s, g[s])
            if s == goal:
                found = True
                break
            for s2, _edge_cost in space.neighbors(s):
                if s2 in closed:
                    continue
                # Lazy Path 2: OPTIMISTICALLY assume line_of_sight(p, s2) instead of
                # checking it here — the check is deferred to set_vertex when s2 pops
                # (Nash & Koenig 2010).
                ecost = _euclid(p, s2)
                cand = g[p] + ecost
                if s2 not in g or cand < g[s2]:
                    g[s2] = cand
                    parent[s2] = p
                    if recorder is not None:
                        recorder.candidate_evaluated(s2, cand)
                        recorder.edge_added(s2, p, ecost)
                    heapq.heappush(frontier, (cand + h(s2), next(counter), s2))

        runtime = time.monotonic() - t0
        if not found:
            emit_finish(recorder, False, [], 0.0, expanded, runtime)
            return PlanResult(success=False, stats=PlanStats(expanded_nodes=expanded))
        path = reconstruct(parent, start, goal)
        cost = g[goal]  # any-angle path cost; adjacent-edge summing would be wrong for jumps.
        emit_finish(recorder, True, path, cost, expanded, runtime)
        return PlanResult(True, path, cost, PlanStats(expanded_nodes=expanded))
