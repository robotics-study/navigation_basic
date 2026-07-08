"""AD* — Anytime Dynamic A*: anytime *and* incremental replanning at once.

Likhachev, Ferguson, Gordon, Stentz & Thrun (2005). AD* fuses ARA*'s inflated
heuristic (fast, bounded-suboptimal first solution that is repaired as ε shrinks)
with D* Lite's backward, incrementally repaired search (goal -> start, g/rhs values
+ the k_m key offset so a moving robot reuses the previous search instead of
restarting). A robot starts with no map (freespace assumption) and a local sensor;
``plan()`` simulates the whole improve -> move -> sense -> repair loop and returns
the *executed trajectory* (NOT a from-start plan).

Two mechanisms are layered on the D* Lite skeleton:

- **ε-inflated keys** (ARA*): the priority of an *over-consistent* vertex inflates
  its heuristic by ε; *under-consistent* vertices keep an un-inflated key (paper's
  key(s), so a raised cost still propagates correctly). ComputeOrImprovePath expands
  under this key with a CLOSED set, giving an ε-suboptimal solution quickly.
- **INCONS list** (ARA*): a vertex that becomes inconsistent *after* it was already
  expanded (is in CLOSED) is parked in INCONS instead of re-entering OPEN this pass.
  When ε is lowered — or an edge cost changes — INCONS ∪ OPEN is reopened with keys
  recomputed under the new ε and CLOSED is cleared, so prior work is reused.

The robot moves one step only once ε has reached ``eps_final`` (the plan is optimal
for the current belief), so the executed trajectory matches D* Lite's. Each improved
plan is published via ``recorder.path_found`` (anytime); a sensed edge-cost change
re-inflates ε to fetch a new suboptimal plan fast, then repairs it back down.

Does NOT subclass the static ``_discrete`` skeletons: this is a backward, belief-based
replanner whose result is a trajectory, not a from-start parent chain.
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
    # are bit-identical to the C++ mirror. The DynamicGridSpace capability exposes no
    # heuristic(), so the planner carries its own (admissible for 8-connected moves).
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    lo = min(dr, dc)
    hi = max(dr, dc)
    return float(hi - lo) + math.sqrt(2.0) * float(lo)


class _ADStarSearch:
    """One improve -> move -> sense -> repair simulation. Holds g/rhs, the belief
    (blocked set), the k_m offset, and the ARA* CLOSED/INCONS bookkeeping across the
    whole run."""

    def __init__(
        self,
        space: DynamicGridSpace[Cell],
        start: Cell,
        goal: Cell,
        eps_start: float,
        eps_final: float,
        eps_step: float,
        sensor_radius: int,
        max_expansions: int,
        recorder: TraceRecorder | None,
    ) -> None:
        self._space = space
        self._start = start
        self._goal = goal
        # ε0 must not fall below the target ε, else no anytime repair happens.
        self._eps = max(eps_start, eps_final)
        self._eps_start = max(eps_start, eps_final)
        self._eps_final = eps_final
        self._eps_step = eps_step
        self._radius = sensor_radius
        self._max_expansions = max_expansions
        self._rec = recorder
        self._s_start = start
        self._s_last = start
        self._k_m = 0.0
        self._blocked: set[Cell] = set()  # belief: known blocked cells (empty = freespace)
        self._g: dict[Cell, float] = {}
        self._rhs: dict[Cell, float] = {}
        self._key_of: dict[Cell, _Key] = {}  # OPEN membership + current key of each vertex
        self._open: list[tuple[_Key, int, Cell]] = []
        self._closed: set[Cell] = set()  # expanded (over-consistent) since the last reopen
        self._incons: set[Cell] = set()  # inconsistent again after expansion; reopened on ε change
        self._counter = itertools.count()
        self._expanded = 0
        self._replans = 0
        self._sensed_cells = 0

    def _g_of(self, c: Cell) -> float:
        return self._g.get(c, _INF)

    def _rhs_of(self, c: Cell) -> float:
        return self._rhs.get(c, _INF)

    def _calc_key(self, s: Cell) -> _Key:
        g = self._g_of(s)
        rhs = self._rhs_of(s)
        if g > rhs:  # over-consistent: inflate the heuristic by ε (ARA* weighting)
            return (rhs + self._eps * _octile(self._s_start, s) + self._k_m, rhs)
        # under-consistent / consistent: NO inflation (Likhachev et al. 2005 key), so a
        # raised cost still propagates on an admissible key.
        return (g + _octile(self._s_start, s) + self._k_m, g)

    def _queue_insert(self, u: Cell, key: _Key) -> None:
        self._key_of[u] = key
        heapq.heappush(self._open, (key, next(self._counter), u))

    def _peek_top(self) -> tuple[_Key, Cell] | None:
        # Drop stale entries (whose stored key no longer matches key_of) and report the
        # smallest live one.
        while self._open:
            key, _, u = self._open[0]
            if self._key_of.get(u) != key:
                heapq.heappop(self._open)
                continue
            return key, u
        return None

    def _update_state(self, u: Cell) -> None:
        if u != self._goal:
            best = _INF
            sbest: Cell | None = None
            best_edge = 0.0
            for s2, cost in self._space.passable_neighbors(u, self._blocked):
                v = cost + self._g_of(s2)
                if v < best:
                    best = v
                    sbest = s2
                    best_edge = cost
            if best != self._rhs_of(u):  # a real change to the cost-to-goal look-ahead
                self._rhs[u] = best
                if self._rec is not None and sbest is not None and best < _INF:
                    self._rec.candidate_evaluated(u, best)
                    self._rec.edge_added(u, sbest, best_edge)  # sbest = successor toward goal
        # Take u out of OPEN/INCONS, then re-file it by (in)consistency + CLOSED status.
        self._key_of.pop(u, None)
        self._incons.discard(u)
        if self._g_of(u) != self._rhs_of(u):
            if u not in self._closed:
                self._queue_insert(u, self._calc_key(u))
            else:
                # Already expanded this pass: defer to the next reopen instead of
                # re-expanding now (the ARA* INCONS trick, carried into AD*).
                self._incons.add(u)

    def _reopen(self) -> None:
        # Move INCONS into OPEN and recompute every key under the current ε and k_m,
        # then clear CLOSED so improved / cost-changed states can be re-expanded.
        # Reinsert in a fixed (row, col) order so the queue tie-break counter is
        # assigned identically to the C++ mirror — hash-iteration order would diverge.
        states = sorted(set(self._key_of.keys()) | self._incons)
        self._incons.clear()
        self._closed.clear()
        self._key_of.clear()
        self._open.clear()
        for s in states:
            self._queue_insert(s, self._calc_key(s))

    def _compute_or_improve_path(self) -> bool:
        # Expand until s_start is consistent and no OPEN key beats its key: g(s_start) is
        # then ε-suboptimal (ARA*/AD* termination). Returns False if the expansion cap
        # tripped (diverging), so the caller stops with the best trajectory so far.
        while True:
            top = self._peek_top()
            if top is None:
                break
            ktop, u = top
            if not (ktop < self._calc_key(self._s_start)
                    or self._rhs_of(self._s_start) != self._g_of(self._s_start)):
                break
            self._key_of.pop(u, None)  # pop u from OPEN
            self._expanded += 1
            if self._expanded > self._max_expansions:
                return False
            if self._rec is not None:
                self._rec.node_expanded(u, min(self._g_of(u), self._rhs_of(u)))
            if self._g_of(u) > self._rhs_of(u):
                self._g[u] = self._rhs_of(u)  # over-consistent: accept it, settle into CLOSED
                self._closed.add(u)
                for s2, _ in self._space.passable_neighbors(u, self._blocked):
                    self._update_state(s2)
            else:
                self._g[u] = _INF  # under-consistent: raise, re-evaluate u and predecessors
                self._update_state(u)
                for s2, _ in self._space.passable_neighbors(u, self._blocked):
                    self._update_state(s2)
        return True

    def _extract_plan(self) -> list[Cell] | None:
        # The current plan: greedily follow argmin over successors of edge + g from
        # s_start to goal (AD* solution extraction). Valid + finite after
        # ComputeOrImprovePath; the visited guard only defends against a malformed g.
        if self._g_of(self._s_start) == _INF:
            return None
        path: list[Cell] = [self._s_start]
        seen: set[Cell] = {self._s_start}
        cur = self._s_start
        while cur != self._goal:
            best = _INF
            nxt: Cell | None = None
            for s2, cost in self._space.passable_neighbors(cur, self._blocked):
                v = cost + self._g_of(s2)
                if v < best:
                    best = v
                    nxt = s2
            if nxt is None or best == _INF or nxt in seen:
                return None
            cur = nxt
            seen.add(cur)
            path.append(cur)
        return path

    def _publish(self) -> None:
        if self._rec is None:
            return
        plan = self._extract_plan()
        if plan is not None:
            self._rec.path_found(plan)  # anytime: bound = current ε

    def _predecessors_if_in_bounds(self, c: Cell) -> list[tuple[Cell, float]] | None:
        # In-bounds predecessors of c (cells that currently plan to move INTO c), or
        # None when c is out of bounds so revealing it changes nothing. The grid is
        # undirected, so an in-bounds c's forward passable neighbours are exactly its
        # predecessors; an out-of-bounds c has forward neighbours but none list c back —
        # that symmetry check drops it without an in-bounds accessor on the capability.
        fwd = self._space.passable_neighbors(c, self._blocked)
        if not fwd:
            return None
        pivot = fwd[0][0]
        if any(s2 == c for s2, _ in self._space.passable_neighbors(pivot, self._blocked)):
            return fwd
        return None

    def _sense(self, robot: Cell) -> list[Cell]:
        # Sense the Euclidean disk of radius `self._radius` around `robot`, reveal newly
        # blocked in-bounds cells into the belief, and return the vertices whose rhs must
        # be repaired. Deterministic scan order (dr outer, dc inner). The immediate
        # 8-neighbourhood is ALWAYS sensed even at radius 1 (whose disk omits diagonals):
        # the robot may step diagonally next, so every reachable cell must be detectable.
        to_update: list[Cell] = []
        r = self._radius
        for dr in range(-r, r + 1):
            for dc in range(-r, r + 1):
                if dr * dr + dc * dc > r * r and max(abs(dr), abs(dc)) > 1:
                    continue
                c = (robot[0] + dr, robot[1] + dc)
                if c in self._blocked:
                    continue
                if not self._space.is_blocked(c):
                    continue
                preds = self._predecessors_if_in_bounds(c)
                if preds is None:  # out of bounds: already impassable, nothing to repair
                    continue
                self._blocked.add(c)
                self._sensed_cells += 1
                if self._rec is not None:
                    self._rec.obstacle_revealed(c)
                to_update.extend(n for n, _ in preds)
        return to_update

    def _greedy_step(self) -> tuple[Cell | None, float]:
        # One belief-optimal move: argmin over successors of edge + g (first-min tie-break).
        best = _INF
        nxt: Cell | None = None
        step_cost = 0.0
        for s2, cost in self._space.passable_neighbors(self._s_start, self._blocked):
            cand = cost + self._g_of(s2)
            if cand < best:
                best = cand
                nxt = s2
                step_cost = cost
        if nxt is None or best == _INF:
            return None, 0.0
        return nxt, step_cost

    def run(self) -> PlanResult[Cell]:
        t0 = time.monotonic()
        self._rhs[self._goal] = 0.0  # goal is the backward-search root
        self._queue_insert(self._goal, self._calc_key(self._goal))

        # Fold obstacles visible from the spawn cell into the belief before the first
        # plan (setup — not a replan).
        for v in self._sense(self._s_start):
            self._update_state(v)
        capped = not self._compute_or_improve_path()
        self._publish()  # first ε_start-suboptimal solution from start

        trajectory: list[Cell] = [self._s_start]
        realized_cost = 0.0
        if self._rec is not None:
            self._rec.robot_moved(self._s_start)

        reached = self._s_start == self._goal
        while not reached and not capped:
            if self._eps > self._eps_final:
                # Anytime improvement: tighten ε, reopen INCONS∪OPEN, repair. No motion —
                # the robot waits for a belief-optimal plan before stepping.
                self._eps = max(self._eps_final, self._eps - self._eps_step)
                self._reopen()
                capped = not self._compute_or_improve_path()
                self._publish()
                continue

            # ε == eps_final: the plan is optimal for the current belief → take one step.
            if self._g_of(self._s_start) == _INF:
                break  # goal unreachable under the current belief
            nxt, step_cost = self._greedy_step()
            if nxt is None:
                break  # boxed in
            self._s_start = nxt
            realized_cost += step_cost
            trajectory.append(self._s_start)
            if self._rec is not None:
                self._rec.robot_moved(self._s_start)
            if self._s_start == self._goal:
                reached = True
                break

            changed = self._sense(self._s_start)
            if changed:
                self._k_m += _octile(self._s_last, self._s_start)  # keep keys monotone
                self._s_last = self._s_start
                for v in changed:
                    self._update_state(v)
                # A sensed cost change is treated as significant: re-inflate ε to fetch a
                # new suboptimal plan fast, then the loop repairs it back to eps_final.
                self._eps = self._eps_start
                self._replans += 1
                self._reopen()
                capped = not self._compute_or_improve_path()
                self._publish()

        runtime = time.monotonic() - t0
        stats = PlanStats(expanded_nodes=self._expanded, iterations=self._replans)
        if self._rec is not None:
            metrics = {
                "runtime_sec": runtime,
                "path_cost": realized_cost if reached else 0.0,
                "expanded_nodes": float(self._expanded),
                "replan_count": float(self._replans),
                "sensed_cells": float(self._sensed_cells),
                "final_eps": self._eps,
            }
            if reached:
                self._rec.path_found(trajectory)
            self._rec.planning_finished(reached, metrics)
        if not reached:
            return PlanResult(success=False, stats=stats)
        return PlanResult(True, trajectory, realized_cost, stats)


class ADStar(GlobalPlanner[Cell, "DynamicGridSpace[Cell]"]):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._eps_start = params.get_float("eps_start")
        self._eps_final = params.get_float("eps_final")
        self._eps_step = params.get_float("eps_step")
        self._sensor_radius = params.get_int("sensor_radius")
        self._max_expansions = params.get_int("max_expansions")

    @property
    def name(self) -> str:
        return "ad_star"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.DYNAMIC_GRID_SPACE}

    def plan(
        self,
        space: DynamicGridSpace[Cell],
        start: Cell,
        goal: Cell,
        recorder: TraceRecorder | None = None,
    ) -> PlanResult[Cell]:
        return _ADStarSearch(
            space,
            start,
            goal,
            self._eps_start,
            self._eps_final,
            self._eps_step,
            self._sensor_radius,
            self._max_expansions,
            recorder,
        ).run()
