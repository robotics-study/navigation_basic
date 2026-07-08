"""ARA*: Anytime Repairing A* — a sequence of weighted-A* searches that reuse
work to converge on the optimum while keeping a proven suboptimality bound.

Likhachev, Gordon & Thrun (2003). Run ImprovePath with an inflated heuristic
f = g + ε·h to get a first ε-suboptimal solution fast, then repeatedly shrink ε
and repair the previous search instead of restarting: states whose g dropped
while already expanded (CLOSED) go to INCONS rather than back into OPEN, and each
ε-iteration reopens INCONS∪OPEN with recomputed keys. Every iteration emits an
improved path (anytime); the last (ε → eps_final) is optimal when eps_final == 1.
"""

from __future__ import annotations

import heapq
import itertools
import math
import time

from navigation.core.capabilities import Capability, DiscreteSpace
from navigation.core.params import ParamSet
from navigation.core.planner import GlobalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import Cell, PlanResult, PlanStats

from ._discrete import reconstruct


class ARAStar(GlobalPlanner[Cell, "DiscreteSpace[Cell]"]):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._eps_start = params.get_float("eps_start")
        self._eps_final = params.get_float("eps_final")
        self._eps_step = params.get_float("eps_step")
        self._max_expansions = params.get_int("max_expansions")

    @property
    def name(self) -> str:
        return "ara_star"

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
        inf = math.inf
        # ε0 must not fall below the target ε, else no anytime repair happens.
        eps = max(self._eps_start, self._eps_final)
        eps_final = self._eps_final
        eps_step = self._eps_step
        max_expansions = self._max_expansions

        g: dict[Cell, float] = {start: 0.0}
        parent: dict[Cell, Cell] = {}
        closed: set[Cell] = set()
        incons: set[Cell] = set()
        open_set: set[Cell] = {start}
        counter = itertools.count()  # stable FIFO tie-break, mirrors the C++ seq order
        heap: list[tuple[float, int, Cell]] = [
            (eps * space.heuristic(start, goal), next(counter), start)
        ]
        expanded = 0

        def improve_path(cur_eps: float) -> None:
            # Expand until the goal's key is no larger than OPEN's minimum key: at
            # that point g(goal) is provably within cur_eps of optimal (the ARA*
            # termination criterion). Lazy heap — stale entries (state no longer in
            # OPEN) are skipped on pop rather than decrease-keyed.
            nonlocal expanded
            while heap:
                while heap and heap[0][2] not in open_set:
                    heapq.heappop(heap)
                if not heap:
                    break
                if g.get(goal, inf) <= heap[0][0]:
                    break
                _, _, s = heapq.heappop(heap)
                open_set.discard(s)
                closed.add(s)
                expanded += 1
                if expanded > max_expansions:
                    return
                if recorder is not None:
                    recorder.node_expanded(s, g[s])
                for nb, edge_cost in space.neighbors(s):
                    tentative = g[s] + edge_cost
                    if nb not in g or tentative < g[nb]:
                        g[nb] = tentative
                        parent[nb] = s
                        if recorder is not None:
                            recorder.candidate_evaluated(nb, tentative)
                            recorder.edge_added(nb, s, edge_cost)
                        if nb not in closed:
                            open_set.add(nb)
                            key = tentative + cur_eps * space.heuristic(nb, goal)
                            heapq.heappush(heap, (key, next(counter), nb))
                        else:
                            # Already expanded but improved: defer to the next, smaller-ε
                            # iteration instead of re-expanding now (the ARA* core trick).
                            incons.add(nb)

        best_path: list[Cell] = []
        best_cost = inf
        success = False
        while True:
            improve_path(eps)
            if goal in g and g[goal] < inf:
                best_cost = g[goal]
                best_path = reconstruct(parent, start, goal)
                success = True
                if recorder is not None:
                    # Anytime: publish the current solution (suboptimality bound = eps).
                    recorder.path_found(best_path)
            else:
                break  # OPEN exhausted without reaching the goal
            if eps <= eps_final or expanded >= max_expansions:
                break
            eps = max(eps_final, eps - eps_step)
            # Reopen INCONS∪OPEN with keys recomputed under the tightened ε and clear
            # CLOSED so improved states can be re-expanded.
            open_set |= incons
            incons = set()
            closed = set()
            heap = [(g[s] + eps * space.heuristic(s, goal), next(counter), s) for s in open_set]
            heapq.heapify(heap)

        runtime = time.monotonic() - t0
        if recorder is not None:
            recorder.planning_finished(
                success,
                {
                    "runtime_sec": runtime,
                    "path_cost": best_cost if success else 0.0,
                    "expanded_nodes": float(expanded),
                },
            )
        if not success:
            return PlanResult(success=False, stats=PlanStats(expanded_nodes=expanded))
        return PlanResult(True, best_path, best_cost, PlanStats(expanded_nodes=expanded))
