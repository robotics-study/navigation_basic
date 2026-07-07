"""Jump Point Search (JPS) — A* over grid "jump points".

Harabor & Grastien (2011). On a uniform-cost 8-connected grid, most of A*'s
expansions are wasted on symmetric permutations of the same path. JPS breaks that
symmetry: instead of every neighbour, a node's successors are the *jump points*
reached by scanning each canonical direction until an obstacle / the goal / a cell
that has a *forced neighbour* (a neighbour reachable optimally only through the
current cell because of an adjacent obstacle). It returns exactly the same optimal
paths as 8-connected A* while expanding far fewer nodes.

This repository's grid forbids corner-cutting (a diagonal step needs both shared
orthogonal cells free — see ``OccupancyGrid2D.neighbors``), so the pruning uses the
no-corner-cutting variant of the rules in Harabor & Grastien (2011): a straight run
has a forced neighbour when an obstacle sits diagonally *behind* the travel
direction, opening a side cell that can only be reached by turning at the current
cell; a diagonal run cuts no corners, so its side openings are already covered by
its two orthogonal scans and it needs no separate forced neighbour.

Capabilities: JPS is a discrete 8-connected grid search (``DISCRETE_SPACE``) whose
jump primitive is a single-cell occupancy/bounds oracle — exactly
``DynamicGridSpace.is_blocked`` (occupied OR out of bounds), so it binds to
``DynamicGridSpace`` and reads no ground-truth neighbour list. Octile costs are
computed here (like Theta* computes its own Euclidean distance) because the bound
space exposes no ``heuristic``; the formula matches ``OccupancyGrid2D.heuristic`` so
the returned cost equals the 8-connected A* optimum bit-for-bit.
"""

from __future__ import annotations

import heapq
import itertools
import math
import time

from navigation.core.capabilities import Capability, DynamicGridSpace
from navigation.core.planner import GlobalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import Cell, PlanResult, PlanStats

from ._discrete import emit_finish

_SQRT2 = math.sqrt(2.0)
# The eight canonical directions, orthogonals before diagonals — the same order as
# OccupancyGrid2D.neighbors, so the start node (which explores all directions)
# settles ties identically to A*.
_DIRS_8 = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]


def _octile(a: Cell, b: Cell) -> float:
    # Octile distance on integer cell deltas, in exactly the same operation order —
    # (hi - lo) + sqrt(2)*lo, sqrt NOT hypot — as OccupancyGrid2D.heuristic. A pure
    # straight/diagonal jump line has this as its true traversed cost, so successor
    # edge costs and f-values match 8-connected A* exactly. Admissible.
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    lo = min(dr, dc)
    hi = max(dr, dc)
    return float(hi - lo) + _SQRT2 * float(lo)


def _sign(x: int) -> int:
    return (x > 0) - (x < 0)


class JPS(GlobalPlanner[Cell, "DynamicGridSpace[Cell]"]):
    @property
    def name(self) -> str:
        return "jps"

    def required_capabilities(self) -> set[Capability]:
        # DISCRETE_SPACE: JPS is 8-connected grid graph search (same optimal paths as
        # A*). DYNAMIC_GRID_SPACE: is_blocked is the single-cell oracle the jumps need.
        return {Capability.DISCRETE_SPACE, Capability.DYNAMIC_GRID_SPACE}

    def plan(
        self,
        space: DynamicGridSpace[Cell],
        start: Cell,
        goal: Cell,
        recorder: TraceRecorder | None = None,
    ) -> PlanResult[Cell]:
        t0 = time.monotonic()

        def blocked(r: int, c: int) -> bool:
            return space.is_blocked((r, c))

        def free(r: int, c: int) -> bool:
            return not space.is_blocked((r, c))

        def scan(r: int, c: int, dr: int, dc: int) -> Cell | None:
            # Advance from (r,c) along (dr,dc) one legal grid step at a time; return the
            # first jump point (goal, or a cell with a forced neighbour) or None if the
            # run dead-ends at an obstacle / boundary. Recursion depth is at most 1: a
            # diagonal run's orthogonal probes are straight scans that never recurse.
            diagonal = dr != 0 and dc != 0
            while True:
                # Step legality mirrors the map's corner rule: a diagonal needs the
                # target and both shared orthogonal cells free (Harabor & Grastien 2011,
                # no-corner-cutting variant).
                if blocked(r + dr, c + dc):
                    return None
                if diagonal and (blocked(r + dr, c) or blocked(r, c + dc)):
                    return None
                r, c = r + dr, c + dc
                if (r, c) == goal:
                    return (r, c)
                if not diagonal:
                    if dc != 0:  # horizontal: obstacle diagonally behind opens a side cell
                        if (free(r - 1, c) and blocked(r - 1, c - dc)) or (
                            free(r + 1, c) and blocked(r + 1, c - dc)
                        ):
                            return (r, c)
                    else:  # vertical
                        if (free(r, c - 1) and blocked(r - dr, c - 1)) or (
                            free(r, c + 1) and blocked(r - dr, c + 1)
                        ):
                            return (r, c)
                elif (
                    scan(r, c, dr, 0) is not None or scan(r, c, 0, dc) is not None
                ):
                    # A diagonal cell whose orthogonal scan finds a jump point is itself a
                    # jump point: the optimal path must branch here.
                    return (r, c)

        def successor_dirs(u: Cell, parent: Cell | None) -> list[tuple[int, int]]:
            # Directions to jump from u given how it was reached. Start (no parent)
            # explores all eight. Otherwise: natural continuation + forced-neighbour
            # branches (Harabor & Grastien 2011). Illegal branches simply yield no jump
            # point, so a slightly liberal set never costs correctness.
            if parent is None:
                return _DIRS_8
            r, c = u
            pdr, pdc = _sign(r - parent[0]), _sign(c - parent[1])
            if pdr != 0 and pdc != 0:  # diagonal: continuation + both orthogonal legs
                return [(pdr, 0), (0, pdc), (pdr, pdc)]
            dirs: list[tuple[int, int]] = [(pdr, pdc)]  # natural continuation
            if pdc != 0:  # horizontal
                if free(r - 1, c) and blocked(r - 1, c - pdc):
                    dirs += [(-1, pdc), (-1, 0)]
                if free(r + 1, c) and blocked(r + 1, c - pdc):
                    dirs += [(1, pdc), (1, 0)]
            else:  # vertical
                if free(r, c - 1) and blocked(r - pdr, c - 1):
                    dirs += [(pdr, -1), (0, -1)]
                if free(r, c + 1) and blocked(r - pdr, c + 1):
                    dirs += [(pdr, 1), (0, 1)]
            return dirs

        g: dict[Cell, float] = {start: 0.0}
        parent: dict[Cell, Cell] = {}
        closed: set[Cell] = set()
        counter = itertools.count()  # stable tie-break, keeps heap entries comparable
        frontier: list[tuple[float, int, Cell]] = [(_octile(start, goal), next(counter), start)]
        expanded = 0
        found = False
        while frontier:
            _, _, u = heapq.heappop(frontier)
            if u in closed:
                continue
            closed.add(u)
            expanded += 1
            if recorder is not None:
                recorder.node_expanded(u, g[u])
            if u == goal:
                found = True
                break
            for dr, dc in successor_dirs(u, parent.get(u)):
                jp = scan(u[0], u[1], dr, dc)
                if jp is None or jp in closed:
                    continue
                tentative = g[u] + _octile(u, jp)
                if jp not in g or tentative < g[jp]:
                    g[jp] = tentative
                    parent[jp] = u
                    if recorder is not None:
                        recorder.candidate_evaluated(jp, tentative)
                        recorder.edge_added(jp, u, _octile(u, jp))
                    heapq.heappush(frontier, (tentative + _octile(jp, goal), next(counter), jp))

        runtime = time.monotonic() - t0
        if not found:
            emit_finish(recorder, False, [], 0.0, expanded, runtime)
            return PlanResult(success=False, stats=PlanStats(expanded_nodes=expanded))
        path = _reconstruct_full(parent, start, goal)
        cost = g[goal]
        emit_finish(recorder, True, path, cost, expanded, runtime)
        return PlanResult(True, path, cost, PlanStats(expanded_nodes=expanded))


def _reconstruct_full(parent: dict[Cell, Cell], start: Cell, goal: Cell) -> list[Cell]:
    # JPS's parent chain is sparse (jump points only). Fill the straight/diagonal grid
    # cells between consecutive jump points so the reported path is the full staircase,
    # matching how A* reports every cell (viz + path-connectivity checks rely on it).
    jumps = [goal]
    node = goal
    while node != start:
        node = parent[node]
        jumps.append(node)
    jumps.reverse()
    path = [start]
    for a, b in zip(jumps, jumps[1:], strict=False):
        dr, dc = _sign(b[0] - a[0]), _sign(b[1] - a[1])
        cur = a
        while cur != b:
            cur = (cur[0] + dr, cur[1] + dc)
            path.append(cur)
    return path
