"""LPA* — Lifelong Planning A*: incremental replanning on a KNOWN map whose edge costs change.

Koenig & Likhachev (2002); journal version Koenig, Likhachev & Furcy (2004). The
simple version of the paper, verbatim: g(s)/rhs(s) estimate the start distance of s
(FORWARD search — D* Lite's backward twin), U holds exactly the locally-inconsistent
vertices keyed by [min(g,rhs)+h(goal,s); min(g,rhs)], and a cost change re-seeds only
the affected vertices' rhs. The heuristic reference is the FIXED goal, so keys stay
comparable across replans — no k_m offset (that exists only because D* Lite's search
root moves with the robot).

The planner is stateful ACROSS plan() calls: round 0 seeds its model from the map's
full ground truth (occupied_cells) and runs one full burst; each scenario revision
applied via apply_revision() flips cells and UpdateVertex's exactly the vertices whose
predecessor set changed, so a later plan() is a repair burst, not a restart. The demo
driver feeds it scenario `revisions:` between plan() calls — LPA* is TOLD about cost
changes (the paper's Main loop), unlike D* Lite which senses them.

Does NOT subclass `_BestFirstSearch`: that skeleton is a one-shot forward search with
no cross-call state — none of which fits. Path extraction walks the parent chain
backward from the goal: at u, the first neighbor p whose `cost + g(p)` equals rhs(u)
is exactly the predecessor whose relaxation SET rhs(u) (same operands and operation
order as _update_vertex, so the equality is bit-exact, not a tolerance). Every chain
vertex is locally consistent when the burst ended — an inconsistent one would sit in
U with key < key(goal), contradicting termination — so each hop satisfies
c = g(u_next) - g(u) and the costs telescope to exactly g(goal).
"""

from __future__ import annotations

import heapq
import itertools
import math
import time

from navigation.core.capabilities import Capability, DynamicGridSpace
from navigation.core.params import ParamSet
from navigation.core.planner import GlobalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import Cell, PlanResult, PlanStats

_INF = float("inf")
_Key = tuple[float, float]


def _octile(a: Cell, b: Cell) -> float:
    # Octile distance on integer cell deltas, in exactly the same operation order —
    # (hi - lo) + sqrt(2)*lo, sqrt NOT hypot — as OccupancyGrid2D.heuristic, so keys
    # are bit-identical to the C++ mirror and the two traces stay in lock-step.
    # Admissible for 8-connected unit/sqrt2 moves; consistent (a metric), which is
    # what bounds a key below by g(goal) at termination.
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    lo = min(dr, dc)
    hi = max(dr, dc)
    return float(hi - lo) + math.sqrt(2.0) * float(lo)


class LpaStar(GlobalPlanner[Cell, "DynamicGridSpace[Cell]"]):
    """LPA* (Koenig, Likhachev & Furcy 2004), simple version — forward search from a
    fixed start whose edge costs change between plan() calls. No params: the paper's
    simplest variant has none."""

    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        # Search state persists ACROSS plan() calls — that IS the algorithm. Seeded
        # on the first plan(); apply_revision() before it is a driver bug (no space
        # to enumerate neighbors from yet), so it raises.
        self._space: DynamicGridSpace[Cell] | None = None
        self._recorder: TraceRecorder | None = None
        self._start: Cell = (0, 0)
        self._goal: Cell = (0, 0)
        # The model: every occupied cell. Seeded from ground truth at the first
        # plan(); revisions flip single cells out of/in it. Out-of-bounds is never
        # in the set and never needs to be — passable_neighbors bounds-checks anyway.
        self._blocked: set[Cell] = set()
        self._g: dict[Cell, float] = {}  # start-distance estimate (absent = inf)
        self._rhs: dict[Cell, float] = {}  # one-step lookahead over predecessors
        self._key_of: dict[Cell, _Key] = {}  # current key of each vertex in the queue
        self._open: list[tuple[_Key, int, Cell]] = []
        self._counter = itertools.count()  # stable tie-break, keeps heap entries comparable
        self._expanded = 0
        self._replans = 0
        self._runtime = 0.0  # cumulative burst time across every plan() round

    @property
    def name(self) -> str:
        return "lpa_star"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.DYNAMIC_GRID_SPACE}

    # --- paper primitives (Figure 3, simple version) -------------------------
    def _g_of(self, c: Cell) -> float:
        return self._g.get(c, _INF)

    def _rhs_of(self, c: Cell) -> float:
        return self._rhs.get(c, _INF)

    def _calc_key(self, s: Cell) -> _Key:
        # CalculateKey(u) = [min(g,rhs) + h(goal,u); min(g,rhs)]. No k_m term: the
        # heuristic's reference (the goal) never moves, so keys stay comparable
        # across replans — the exact contrast with D* Lite's moving-root key.
        m = min(self._g_of(s), self._rhs_of(s))
        return (m + _octile(s, self._goal), m)

    def _queue_insert(self, u: Cell, key: _Key) -> None:
        self._key_of[u] = key
        heapq.heappush(self._open, (key, next(self._counter), u))

    def _peek_top(self) -> tuple[_Key, Cell] | None:
        # Drop stale entries (whose stored key no longer matches key_of — a vertex
        # removed from U when it became consistent or blocked, or re-inserted under
        # a new key) and report the smallest live one.
        while self._open:
            key, _, u = self._open[0]
            if self._key_of.get(u) != key:
                heapq.heappop(self._open)
                continue
            return key, u
        return None

    def _update_vertex(self, space: DynamicGridSpace[Cell], u: Cell) -> None:
        # UpdateVertex(u): rhs = min over predecessors (the grid is undirected, so
        # passable neighbours of u ARE its predecessors — same symmetry argument as
        # D* Lite's belief search). The start keeps rhs=0 forever; a blocked cell
        # never re-enters U while blocked (apply_revision pops it on the flip).
        if u != self._start:
            best = _INF
            sbest: Cell | None = None
            best_edge = 0.0
            for s2, cost in space.passable_neighbors(u, self._blocked):
                v = cost + self._g_of(s2)
                if v < best:
                    best = v
                    sbest = s2
                    best_edge = cost
            old = self._rhs_of(u)
            if best != old:  # a real relaxation of the start-distance estimate
                self._rhs[u] = best
                rec = self._recorder
                if rec is not None and sbest is not None and best < _INF:
                    rec.candidate_evaluated(u, best)
                    rec.edge_added(u, sbest, best_edge)  # sbest = predecessor toward start
        self._key_of.pop(u, None)  # remove u from the queue; reinsert below iff inconsistent
        if self._g_of(u) != self._rhs_of(u):
            self._queue_insert(u, self._calc_key(u))

    def _compute_shortest_path(self, space: DynamicGridSpace[Cell]) -> None:
        # ComputeShortestPath(): expand in key order until the goal is locally
        # consistent AND no live key undercuts the goal's.
        while True:
            top = self._peek_top()
            if top is None:
                break
            ktop, u = top
            if not (ktop < self._calc_key(self._goal)
                    or self._rhs_of(self._goal) != self._g_of(self._goal)):
                break
            self._key_of.pop(u)  # pop u (its heap entry is now stale)
            self._expanded += 1
            rec = self._recorder
            if rec is not None:
                rec.node_expanded(u, min(self._g_of(u), self._rhs_of(u)))
            knew = self._calc_key(u)
            if ktop < knew:
                # Key-invariant (no k_m here): a popped key can only be stale from
                # the pop itself. Re-inserting it is therefore dead code by proof —
                # kept verbatim from the paper's pseudocode, and provably inert.
                self._queue_insert(u, knew)
            elif self._g_of(u) > self._rhs_of(u):
                self._g[u] = self._rhs_of(u)  # over-consistent: accept it, relax successors
                for s2, _ in space.passable_neighbors(u, self._blocked):
                    self._update_vertex(space, s2)
            else:
                self._g[u] = _INF  # under-consistent: raise, re-evaluate u and successors
                self._update_vertex(space, u)
                for s2, _ in space.passable_neighbors(u, self._blocked):
                    self._update_vertex(space, s2)

    def _extract_path(self, space: DynamicGridSpace[Cell]) -> tuple[list[Cell], float]:
        # Walk the parent chain BACKWARD from the goal and reverse it. At u, the
        # first neighbor p with cost + g(p) == rhs(u) is exactly the predecessor
        # whose relaxation SET rhs(u) (same operands and operation order as
        # _update_vertex — so the equality is bit-exact, not a tolerance). Every
        # chain vertex is locally consistent when the burst ended: an inconsistent
        # one would sit in U with key < key(goal), contradicting termination — so
        # each hop satisfies c = g(child) - g(u) and the costs telescope to g(goal).
        if self._g_of(self._goal) == _INF:
            return [], 0.0
        path_rev: list[Cell] = [self._goal]
        cost = 0.0
        cur = self._goal
        while cur != self._start:
            rhs_cur = self._rhs_of(cur)
            nxt: Cell | None = None
            step_cost = 0.0
            for s2, edge in space.passable_neighbors(cur, self._blocked):
                cand = edge + self._g_of(s2)
                if cand == rhs_cur:
                    nxt = s2
                    step_cost = edge
                    break
            if nxt is None:  # unreachable in a correct run; fail honestly, never loop
                return [], 0.0
            cost += step_cost
            path_rev.append(nxt)
            cur = nxt
        path_rev.reverse()
        return path_rev, cost

    # --- planner surface ------------------------------------------------------
    def plan(
        self,
        space: DynamicGridSpace[Cell],
        start: Cell,
        goal: Cell,
        recorder: TraceRecorder | None = None,
    ) -> PlanResult[Cell]:
        t0 = time.monotonic()
        first_round = self._space is None
        if first_round:
            # Initialize(): every g/rhs starts infinite (dicts stay empty; _g_of/
            # _rhs_of default to inf), the start's rhs pins to 0, and the model
            # seeds from ground truth — LPA* plans on what it KNOWS, not senses.
            self._space = space
            self._recorder = recorder
            self._start = start
            self._goal = goal
            self._blocked.update(space.occupied_cells())
            self._rhs[start] = 0.0
            self._queue_insert(start, self._calc_key(start))
        else:
            # Fixed endpoints are the algorithm: re-rooting is D* Lite's job.
            if start != self._start or goal != self._goal:
                raise ValueError(
                    "LpaStar is fixed-start/fixed-goal; re-planning with a moved "
                    "root is D* Lite, not LPA*"
                )
            if recorder is not None:
                self._recorder = recorder  # later rounds may carry a fresh recorder

        self._compute_shortest_path(space)
        if not first_round:
            self._replans += 1

        path, cost = self._extract_path(space)
        reached = bool(path)
        self._runtime += time.monotonic() - t0
        stats = PlanStats(expanded_nodes=self._expanded, iterations=self._replans)
        rec = self._recorder
        if rec is not None:
            metrics = {
                "runtime_sec": self._runtime,
                "path_cost": cost,
                "expanded_nodes": float(self._expanded),
                "replan_count": float(self._replans),
            }
            if reached:
                rec.path_found(path)
            rec.planning_finished(reached, metrics)
        if not reached:
            return PlanResult(success=False, stats=stats)
        return PlanResult(True, path, cost, stats)

    def apply_revision(self, cells: list[Cell], blocked: bool) -> None:
        """The paper's Main(): 'wait for changes in edge costs; for all directed edges
        (u,v) with changed edge costs, UpdateVertex(v)'. Flipping a cell flips every
        incident edge (a blocked cell is an impassable one), so the vertices whose
        predecessor set changed are the flipped cell itself and its passable
        neighbours — exactly what gets UpdateVertex'd. A newly-blocked cell leaves U
        instead: a blocked vertex must never pop, or it would propagate g through a
        wall (its stale finite g/rhs stay stored but are unreachable — every rhs is
        computed over passable predecessors only). No-op flips (already in that state
        in the MODEL) change no edge cost, so they emit nothing and update nothing."""
        space = self._space
        if space is None:
            raise RuntimeError(
                "apply_revision before the first plan(): LPA* seeds its model in plan()"
            )
        rec = self._recorder
        for c in cells:
            # The MODEL's state before the flip decides what changed, not ground
            # truth: revisions are reversible cost changes (free -> blocked -> free
            # on the same cell is legal), and only model membership tracks that.
            # Out-of-bounds cells are never members, so freeing one is a no-op; an
            # out-of-bounds block adds a member that passable_neighbors bounds-checks
            # away anyway — a recorded no change to any real edge cost.
            was_blocked = c in self._blocked
            if was_blocked == blocked:
                continue
            if rec is not None:
                rec.obstacle_changed(c, blocked)
            if blocked:
                self._blocked.add(c)
                self._key_of.pop(c, None)  # a blocked vertex must not sit in U
            else:
                self._blocked.discard(c)
                self._update_vertex(space, c)  # c became a vertex again: recompute rhs
            for s2, _ in space.passable_neighbors(c, self._blocked):
                self._update_vertex(space, s2)
