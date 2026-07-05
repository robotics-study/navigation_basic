"""D* Lite — incremental replanning under a locally sensed, initially unknown map.

Koenig & Likhachev (2002). A robot starts with no map (every in-bounds cell assumed
free) and a local sensor. ``plan()`` simulates the whole move -> sense -> repair loop
and returns the *executed trajectory* (NOT a from-start plan): a backward A*-like
search from the goal maintains g/rhs values that are cheaply repaired whenever a
sensor reading contradicts the belief, so each replan reuses the previous search
instead of restarting. The ``k_m`` key offset keeps queue keys monotone as the robot
(the heuristic's reference point) advances.

Does NOT subclass ``_BestFirstSearch``: that skeleton is a forward, ground-truth,
one-shot grid search — none of which fits D* Lite. Path is the trajectory, so
``_discrete.reconstruct`` (a parent chain) does not apply either.
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
    # are bit-identical to the C++ mirror and the two traces the bench compares stay
    # in lock-step. Admissible for 8-connected unit/sqrt2 moves.
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    lo = min(dr, dc)
    hi = max(dr, dc)
    return float(hi - lo) + math.sqrt(2.0) * float(lo)


class _DStarLiteSearch:
    """One move -> sense -> repair simulation. Holds g/rhs, the belief (blocked set),
    and the k_m offset across replans."""

    def __init__(
        self,
        space: DynamicGridSpace[Cell],
        start: Cell,
        goal: Cell,
        sensor_radius: int,
        recorder: TraceRecorder | None,
    ) -> None:
        self._space = space
        self._start = start
        self._goal = goal
        self._radius = sensor_radius
        self._rec = recorder
        self._s_start = start
        self._s_last = start
        self._k_m = 0.0
        self._blocked: set[Cell] = set()  # belief: known blocked cells (empty = freespace)
        self._g: dict[Cell, float] = {}
        self._rhs: dict[Cell, float] = {}
        self._key_of: dict[Cell, _Key] = {}  # current key of each vertex in the queue
        self._open: list[tuple[_Key, int, Cell]] = []
        self._counter = itertools.count()  # stable tie-break, keeps heap entries comparable
        self._expanded = 0
        self._replans = 0
        self._sensed_cells = 0

    def _g_of(self, c: Cell) -> float:
        return self._g.get(c, _INF)

    def _rhs_of(self, c: Cell) -> float:
        return self._rhs.get(c, _INF)

    def _calc_key(self, s: Cell) -> _Key:
        m = min(self._g_of(s), self._rhs_of(s))
        return (m + _octile(self._s_start, s) + self._k_m, m)

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

    def _update_vertex(self, u: Cell) -> None:
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
            old = self._rhs_of(u)
            if best != old:  # a real relaxation of the cost-to-goal estimate
                self._rhs[u] = best
                if self._rec is not None and sbest is not None and best < _INF:
                    self._rec.candidate_evaluated(u, best)
                    self._rec.edge_added(u, sbest, best_edge)  # sbest = successor toward goal
        self._key_of.pop(u, None)  # remove u from the queue; reinsert below iff inconsistent
        if self._g_of(u) != self._rhs_of(u):
            self._queue_insert(u, self._calc_key(u))

    def _compute_shortest_path(self) -> None:
        while True:
            top = self._peek_top()
            if top is None:
                break
            ktop, u = top
            if not (ktop < self._calc_key(self._s_start)
                    or self._rhs_of(self._s_start) != self._g_of(self._s_start)):
                break
            self._key_of.pop(u, None)  # pop u (its heap entry is now stale)
            self._expanded += 1
            if self._rec is not None:
                self._rec.node_expanded(u, min(self._g_of(u), self._rhs_of(u)))
            knew = self._calc_key(u)
            if ktop < knew:
                self._queue_insert(u, knew)  # stale key: reinsert with the up-to-date one
            elif self._g_of(u) > self._rhs_of(u):
                self._g[u] = self._rhs_of(u)  # over-consistent: accept it, relax predecessors
                for s2, _ in self._space.passable_neighbors(u, self._blocked):
                    self._update_vertex(s2)
            else:
                self._g[u] = _INF  # under-consistent: raise, re-evaluate u and predecessors
                self._update_vertex(u)
                for s2, _ in self._space.passable_neighbors(u, self._blocked):
                    self._update_vertex(s2)

    def _predecessors_if_in_bounds(self, c: Cell) -> list[tuple[Cell, float]] | None:
        # In-bounds predecessors of c (cells that currently plan to move INTO c), or
        # None when c is out of bounds so revealing it changes nothing. The grid is
        # undirected, so for an in-bounds c its forward passable neighbours are exactly
        # its predecessors; an out-of-bounds c still has forward neighbours but none of
        # them list c back — that symmetry check drops it without an in-bounds accessor
        # on the capability.
        fwd = self._space.passable_neighbors(c, self._blocked)
        if not fwd:
            return None
        pivot = fwd[0][0]
        if any(s2 == c for s2, _ in self._space.passable_neighbors(pivot, self._blocked)):
            return fwd
        return None

    def _sense(self, robot: Cell) -> list[Cell]:
        # Sense the Euclidean disk of radius `self._radius` cells around `robot`, reveal
        # newly blocked in-bounds cells into the belief, and return the vertices whose
        # rhs must be repaired. Deterministic scan order (dr outer, dc inner).
        # The immediate 8-neighbourhood is ALWAYS sensed even when radius=1 (whose
        # Euclidean disk omits the diagonals): the robot may step diagonally next, so
        # every cell it can move into must be detectable or it could walk into a real
        # obstacle believed free.
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

    def run(self) -> PlanResult[Cell]:
        t0 = time.monotonic()
        self._rhs[self._goal] = 0.0  # Initialize(): goal is the backward-search root
        self._queue_insert(self._goal, self._calc_key(self._goal))

        # Fold obstacles visible from the spawn cell into the initial belief before the
        # first plan (setup — not a replan).
        for v in self._sense(self._s_start):
            self._update_vertex(v)
        self._compute_shortest_path()

        trajectory: list[Cell] = [self._s_start]
        realized_cost = 0.0
        if self._rec is not None:
            self._rec.robot_moved(self._s_start)

        reached = self._s_start == self._goal
        while not reached:
            if self._g_of(self._s_start) == _INF:
                break  # goal unreachable under the current belief
            # Greedy step: argmin over successors of edge + g (first-min tie-break).
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
                    self._update_vertex(v)
                self._compute_shortest_path()
                self._replans += 1

        runtime = time.monotonic() - t0
        stats = PlanStats(expanded_nodes=self._expanded, iterations=self._replans)
        if self._rec is not None:
            metrics = {
                "runtime_sec": runtime,
                "path_cost": realized_cost if reached else 0.0,
                "expanded_nodes": float(self._expanded),
                "replan_count": float(self._replans),
                "sensed_cells": float(self._sensed_cells),
            }
            if reached:
                self._rec.path_found(trajectory)
            self._rec.planning_finished(reached, metrics)
        if not reached:
            return PlanResult(success=False, stats=stats)
        return PlanResult(True, trajectory, realized_cost, stats)


class DStarLite(GlobalPlanner[Cell, "DynamicGridSpace[Cell]"]):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._sensor_radius = params.get_int("sensor_radius")

    @property
    def name(self) -> str:
        return "dstar_lite"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.DYNAMIC_GRID_SPACE}

    def plan(
        self,
        space: DynamicGridSpace[Cell],
        start: Cell,
        goal: Cell,
        recorder: TraceRecorder | None = None,
    ) -> PlanResult[Cell]:
        return _DStarLiteSearch(space, start, goal, self._sensor_radius, recorder).run()
