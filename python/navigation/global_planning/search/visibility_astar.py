"""Visibility A* — any-angle path planning over the cell-centre visibility graph.

Plain A* whose successor relation is *line-of-sight visibility* rather than grid
adjacency: from an expanded cell it relaxes every free cell it can see along an
obstacle-free straight line, at Euclidean cost. The result is the shortest path
over the cell-centre visibility graph (V = reachable free cells, E = mutually
LOS-visible pairs, weight = straight-line length), found with the admissible +
consistent Euclidean heuristic. Weighting h by w > 1 trades that optimality for
speed (Pohl 1970).

This is a cell-centre approximation, NOT a true Euclidean any-angle optimum:
turning points are restricted to cell centres, whereas the genuinely shortest
any-angle route may turn at obstacle corners that no cell centre lands on. It is
therefore not an interval-based optimal any-angle search (which would place turns
at those corners). What it does give is a valid any-angle path whose every leg is
a LOS-clear straight segment, and — because any Theta* output
is itself a cell-centre LOS polyline, i.e. one path in this same graph — a cost
no worse than Theta* on the same instance.

Depends only on the ``LineOfSightSpace`` capability (``neighbors`` +
``line_of_sight``); it never references a concrete map. The candidate vertex set
is discovered as the start's connected free component via ``neighbors()``.
"""

from __future__ import annotations

import heapq
import itertools
import math
import time
from collections.abc import Callable

from navigation.core.capabilities import Capability, LineOfSightSpace
from navigation.core.params import ParamSet
from navigation.core.planner import GlobalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import Cell, PlanResult, PlanStats

from ._discrete import emit_finish, reconstruct


def _euclid(a: Cell, b: Cell) -> float:
    # Euclidean distance on integer cell-index deltas. math.sqrt (correctly
    # rounded, and sqrt(2.0) exactly equals the grid diagonal cost) keeps
    # f-values / the emitted trace bit-identical with the C++ mirror, matching
    # Theta*'s any-angle cost model.
    dr = float(a[0] - b[0])
    dc = float(a[1] - b[1])
    return math.sqrt(dr * dr + dc * dc)


class VisibilityAStarPlanner(GlobalPlanner[Cell, "LineOfSightSpace[Cell]"]):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._heuristic_weight = params.get_float("heuristic_weight")

    @property
    def name(self) -> str:
        return "visibility_astar"

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

        # Candidate vertex set = the start's connected free component, discovered
        # through the capability's neighbors() alone (no concrete-map access).
        # Any feasible path stays inside this component, so restricting roots and
        # interval members to it preserves the visibility-graph optimum; a goal
        # outside it is genuinely unreachable.
        by_row = self._reachable_rows(space, start)

        g: dict[Cell, float] = {start: 0.0}
        parent: dict[Cell, Cell] = {start: start}  # self-parent stops reconstruct
        closed: set[Cell] = set()
        counter = itertools.count()  # stable tie-break, keeps heap entries comparable
        frontier: list[tuple[float, int, Cell]] = [(h(start), next(counter), start)]
        expanded = 0
        found = False
        goal_reachable = goal[0] in by_row and goal[1] in by_row[goal[0]]
        while frontier and goal_reachable:
            _, _, root = heapq.heappop(frontier)
            if root in closed:
                continue
            closed.add(root)
            expanded += 1
            if recorder is not None:
                recorder.node_expanded(root, g[root])
            if root == goal:
                found = True
                break
            self._expand_intervals(
                space, root, by_row, g, parent, closed, frontier, counter, h, recorder
            )

        runtime = time.monotonic() - t0
        if not found:
            emit_finish(recorder, False, [], 0.0, expanded, runtime)
            return PlanResult(success=False, stats=PlanStats(expanded_nodes=expanded))
        path = reconstruct(parent, start, goal)
        cost = g[goal]  # any-angle polyline length; adjacent-edge summing is wrong for jumps.
        emit_finish(recorder, True, path, cost, expanded, runtime)
        return PlanResult(True, path, cost, PlanStats(expanded_nodes=expanded))

    def _reachable_rows(
        self, space: LineOfSightSpace[Cell], start: Cell
    ) -> dict[int, list[int]]:
        """Free cells reachable from ``start`` (via ``neighbors()``), bucketed by
        row with each row's columns sorted ascending — the per-row layout the
        interval projection scans."""
        seen: set[Cell] = {start}
        stack: list[Cell] = [start]
        while stack:
            cell = stack.pop()
            for nb, _cost in space.neighbors(cell):
                if nb not in seen:
                    seen.add(nb)
                    stack.append(nb)
        by_row: dict[int, list[int]] = {}
        for row, col in seen:
            by_row.setdefault(row, []).append(col)
        for cols in by_row.values():
            cols.sort()
        return by_row

    def _expand_intervals(
        self,
        space: LineOfSightSpace[Cell],
        root: Cell,
        by_row: dict[int, list[int]],
        g: dict[Cell, float],
        parent: dict[Cell, Cell],
        closed: set[Cell],
        frontier: list[tuple[float, int, Cell]],
        counter: itertools.count[int],
        h: Callable[[Cell], float],
        recorder: TraceRecorder | None,
    ) -> None:
        """Project ``root``'s visibility into per-row successor intervals and relax
        every cell they contain. An interval is a maximal run of row-adjacent free
        cells all LOS-visible from ``root``; relaxing the whole visible set makes
        the search the cell-centre visibility-graph optimum. Each improving
        relaxation is emitted as an any-angle
        ``root -> cell`` edge, so the replayer draws the fan of visible intervals
        (its parent-link straight-line rendering already handles non-adjacent
        edges, as with Theta*'s grandparent shortcut)."""
        g_root = g[root]
        for row in sorted(by_row):
            cols = by_row[row]
            n = len(cols)
            i = 0
            while i < n:
                if not space.line_of_sight(root, (row, cols[i])):
                    i += 1
                    continue
                # Extend the interval while columns stay contiguous and visible.
                j = i
                while (
                    j + 1 < n
                    and cols[j + 1] == cols[j] + 1
                    and space.line_of_sight(root, (row, cols[j + 1]))
                ):
                    j += 1
                lo, hi = cols[i], cols[j]
                for col in range(lo, hi + 1):
                    cell = (row, col)
                    if cell == root or cell in closed:
                        continue
                    ecost = _euclid(root, cell)
                    cand = g_root + ecost
                    if cell not in g or cand < g[cell]:
                        g[cell] = cand
                        parent[cell] = root
                        if recorder is not None:
                            # Expose the (root, interval) node so viz can draw the
                            # visible column run [lo, hi] this cell was relaxed from.
                            interval = {"row": row, "col_lo": lo, "col_hi": hi}
                            recorder.candidate_evaluated(cell, cand)
                            recorder.edge_added(cell, root, ecost, data=interval)
                        heapq.heappush(frontier, (cand + h(cell), next(counter), cell))
                i = j + 1
