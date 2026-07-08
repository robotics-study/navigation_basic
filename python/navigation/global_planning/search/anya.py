"""Anya — optimal Euclidean any-angle pathfinding via interval search.

Harabor, Grastien, Öz & Aksakalli, "Optimal Any-Angle Pathfinding In Practice",
JAIR 56 (2016), 89-118. Unlike Theta*/Visibility A* (which pin turning points to
cell CENTRES and are therefore only any-angle *approximations*), Anya searches
over (root, interval) nodes whose turning points are grid **corners (vertices)**.
Because a shortest Euclidean path in a polygonal domain is a taut string that
bends only at convex obstacle corners, allowing roots to sit exactly on those
corners lets Anya return the **true continuous Euclidean shortest any-angle path**,
not a cell-centre approximation.

Geometry frame (x, y): x = col + 0.5, y = row + 0.5. A cell (r, c) is the unit
square x in [c, c+1], y in [r, r+1]; grid corners are the integer (x, y) lattice;
cell centres (start / goal) are half-integer. Distance is Euclidean on cell-index
deltas, so costs are directly comparable with Theta*/Visibility A* and identical
across the C++ mirror.

A search node is a corner ROOT plus the visibility INTERVALS it projects across
the grid. Expanding a root sweeps its visibility row by row (cone successors) and
along its own row (flat successors); every convex obstacle corner the sweep
reaches is a successor turning point. g(root) is the accumulated Euclidean start->
root distance; the frontier is ordered by f = g(root) + w * ||root - goal|| (an
admissible straight-line bound). The taut root->corner edges are exactly those an
optimal path can use, so best-first expansion settles the Euclidean optimum.

Occupancy is observed only through the ``LineOfSightSpace`` capability's
``neighbors()`` (the reachable free component); cells outside it are treated as
blocked. No concrete map class is referenced, and the corner-level line-of-sight /
projection is computed here (the capability's ``line_of_sight`` answers only
cell-centre pairs, which cannot express corner-hugging turns).
"""

from __future__ import annotations

import heapq
import itertools
import math
import time
from collections.abc import Iterable

from navigation.core.capabilities import Capability, LineOfSightSpace
from navigation.core.params import ParamSet
from navigation.core.planner import GlobalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import Cell, PlanResult, PlanStats

from ._discrete import emit_finish

# A continuous point in the geometry frame (x = col + 0.5, y = row + 0.5).
_Point = tuple[float, float]


def _euclid(ax: float, ay: float, bx: float, by: float) -> float:
    # sqrt(dx^2 + dy^2), NOT math.hypot: hypot is relaxed-accuracy and can differ
    # 1 ULP from the C++ mirror, diverging the cost the bench compares across
    # languages. sqrt is correctly rounded, matching the Theta*/Visibility cost model.
    dx = ax - bx
    dy = ay - by
    return math.sqrt(dx * dx + dy * dy)


def _cell_free(free: frozenset[Cell], cx: int, cy: int) -> bool:
    # Square with integer min-corner (x=cx, y=cy) == cell (row=cy, col=cx).
    return (cy, cx) in free


def _is_corner(free: frozenset[Cell], x: int, y: int) -> bool:
    # A grid vertex is a convex/reflex obstacle corner (a candidate turning point)
    # iff its four incident cells mix free and blocked.
    cells = ((x - 1, y - 1), (x, y - 1), (x - 1, y), (x, y))
    blocked = [not _cell_free(free, cx, cy) for cx, cy in cells]
    return any(blocked) and not all(blocked)


def _proj_x(rx: float, ry: float, x0: float, y0: float, yn: float) -> float:
    # x-coordinate on row yn of the ray from root (rx, ry) through (x0, y0).
    return rx + (x0 - rx) * (yn - ry) / (y0 - ry)


def _seg_clear(free: frozenset[Cell], p: _Point, q: _Point, eps: float) -> bool:
    """True iff the open segment p->q is a valid any-angle move: it enters no blocked
    cell interior, does not travel along a grid edge whose two sides are both blocked,
    and does not squeeze through a blocked pinch corner (corner-cutting forbidden).

    A segment lying exactly on an integer grid line only grazes cell boundaries (zero
    interior overlap), so it stays valid as long as ONE adjacent side is free — an
    obstacle corner may be hugged (Harabor et al. 2016, "taut" any-angle paths bend
    at convex corners). Snapping the on-line sample to one side with ``floor`` would
    wrongly forbid such edge-grazing legs, breaking Euclidean optimality; instead the
    two straddling cells are tested and a free side suffices. Corner-level visibility
    primitive used by the interval sweep — the capability's cell-centre
    ``line_of_sight`` cannot answer corner endpoints."""
    px, py = p
    qx, qy = q
    if px == qx and py == qy:
        return True
    dx = qx - px
    dy = qy - py
    # Parameters t where the segment crosses an integer grid line; between two
    # consecutive crossings the segment lies within one cell (interior) or, when the
    # whole segment is axis-aligned on an integer line, along one cell edge.
    ts = {0.0, 1.0}
    if dx != 0.0:
        lo, hi = (px, qx) if px < qx else (qx, px)
        for xi in range(math.ceil(lo), math.floor(hi) + 1):
            ts.add((xi - px) / dx)
    if dy != 0.0:
        lo, hi = (py, qy) if py < qy else (qy, py)
        for yi in range(math.ceil(lo), math.floor(hi) + 1):
            ts.add((yi - py) / dy)
    ordered = sorted(t for t in ts if 0.0 <= t <= 1.0)
    for a, b in zip(ordered, ordered[1:], strict=False):
        if b - a < 1e-12:
            continue
        tm = 0.5 * (a + b)
        mx = px + tm * dx
        my = py + tm * dy
        if dx == 0.0 and abs(mx - round(mx)) < eps:
            # Vertical segment on grid column x=round(mx): edge-graze the two cells it
            # straddles; blocked only if BOTH are solid.
            xi = round(mx)
            row = math.floor(my)
            if not _cell_free(free, xi - 1, row) and not _cell_free(free, xi, row):
                return False
        elif dy == 0.0 and abs(my - round(my)) < eps:
            # Horizontal segment on grid row y=round(my): same edge-graze rule.
            yi = round(my)
            col = math.floor(mx)
            if not _cell_free(free, col, yi - 1) and not _cell_free(free, col, yi):
                return False
        elif not _cell_free(free, math.floor(mx), math.floor(my)):
            return False
    # Pinch: a diagonal segment passing exactly through an interior lattice corner
    # is blocked when the two cells it would squeeze between are both blocked.
    if dx != 0.0 and dy != 0.0:
        for t in ordered:
            if t <= eps or t >= 1.0 - eps:
                continue
            x = px + t * dx
            y = py + t * dy
            if abs(x - round(x)) < eps and abs(y - round(y)) < eps:
                ix, iy = round(x), round(y)
                if (dx > 0.0) == (dy > 0.0):
                    if not _cell_free(free, ix - 1, iy) and not _cell_free(free, ix, iy - 1):
                        return False
                elif not _cell_free(free, ix - 1, iy - 1) and not _cell_free(free, ix, iy):
                    return False
    return True


def _merge(pieces: list[tuple[float, float]], eps: float) -> list[tuple[float, float]]:
    if not pieces:
        return []
    pieces = sorted(pieces)
    out: list[list[float]] = [list(pieces[0])]
    for a, b in pieces[1:]:
        if a <= out[-1][1] + eps:
            out[-1][1] = max(out[-1][1], b)
        else:
            out.append([a, b])
    return [(a, b) for a, b in out]


def _clear_pieces(
    free: frozenset[Cell],
    root: _Point,
    yn: float,
    lo: float,
    hi: float,
    splits: Iterable[float],
    eps: float,
) -> list[tuple[float, float]]:
    """Maximal x-subintervals of [lo, hi] on row yn fully visible from root. Split
    candidates mark every possible visibility transition (grid columns + the
    back-projected columns of the previous row); each homogeneous piece is verified
    exactly with a single ``_seg_clear`` sample."""
    pts = {lo, hi}
    for s in splits:
        if lo - eps <= s <= hi + eps:
            pts.add(min(max(s, lo), hi))
    ordered = sorted(pts)
    pieces: list[tuple[float, float]] = []
    for a, b in zip(ordered, ordered[1:], strict=False):
        if b - a < 1e-7:
            continue
        mid = 0.5 * (a + b)
        if _seg_clear(free, root, (mid, yn), eps):
            pieces.append((a, b))
    return _merge(pieces, eps)


class Anya(GlobalPlanner[Cell, "LineOfSightSpace[Cell]"]):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        # Float tolerance for treating a projected coordinate as landing on a grid
        # vertex / for the pinch test. Anya has no behavioural tuning knob (it is
        # exact); this is the only declared parameter.
        self._eps = params.get_float("vertex_epsilon")

    @property
    def name(self) -> str:
        return "anya"

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
        eps = self._eps
        free = self._reachable(space, start)
        r0, r1, c0, c1 = self._bounds(free)

        s_pt: _Point = (start[1] + 0.5, start[0] + 0.5)
        g_pt: _Point = (goal[1] + 0.5, goal[0] + 0.5)

        def h(p: _Point) -> float:
            return _euclid(p[0], p[1], g_pt[0], g_pt[1])

        g: dict[_Point, float] = {s_pt: 0.0}
        parent: dict[_Point, _Point] = {s_pt: s_pt}
        # Interval a corner was first reached through, in cell-index coords, for the
        # trace edge data (mirrors Visibility A*'s {row, col_lo, col_hi}).
        via_interval: dict[_Point, tuple[float, float, float]] = {}
        settled: set[_Point] = set()
        counter = itertools.count()
        frontier: list[tuple[float, int, _Point]] = [(h(s_pt), next(counter), s_pt)]
        goal_cost = math.inf
        goal_root: _Point | None = None
        expanded = 0

        while frontier:
            f, _, root = heapq.heappop(frontier)
            if root in settled:
                continue
            if f >= goal_cost - eps:
                break
            settled.add(root)
            expanded += 1
            if recorder is not None:
                recorder.node_expanded(self._cell_of(free, root), g[root])
            # Final leg: any corner that can see the goal directly closes a path.
            if _seg_clear(free, root, g_pt, eps):
                cand = g[root] + _euclid(root[0], root[1], g_pt[0], g_pt[1])
                if cand < goal_cost:
                    goal_cost = cand
                    goal_root = root
            for corner, (yrow, lo, hi) in self._successors(free, root, r0, r1, c0, c1, eps):
                nd = g[root] + _euclid(root[0], root[1], corner[0], corner[1])
                if corner not in g or nd < g[corner] - eps:
                    g[corner] = nd
                    parent[corner] = root
                    via_interval[corner] = (yrow - 0.5, lo - 0.5, hi - 0.5)
                    if recorder is not None:
                        c_cell = self._cell_of(free, corner)
                        recorder.candidate_evaluated(c_cell, nd)
                        recorder.edge_added(
                            c_cell,
                            self._cell_of(free, root),
                            _euclid(root[0], root[1], corner[0], corner[1]),
                            data={
                                "row": via_interval[corner][0],
                                "col_lo": via_interval[corner][1],
                                "col_hi": via_interval[corner][2],
                            },
                        )
                    heapq.heappush(frontier, (nd + h(corner), next(counter), corner))

        runtime = time.monotonic() - t0
        if goal_root is None:
            emit_finish(recorder, False, [], 0.0, expanded, runtime)
            return PlanResult(success=False, stats=PlanStats(expanded_nodes=expanded))
        path = self._reconstruct(free, parent, s_pt, goal_root, start, goal)
        emit_finish(recorder, True, path, goal_cost, expanded, runtime)
        return PlanResult(True, path, goal_cost, PlanStats(expanded_nodes=expanded))

    # --- occupancy observed via the capability -----------------------------
    def _reachable(self, space: LineOfSightSpace[Cell], start: Cell) -> frozenset[Cell]:
        seen: set[Cell] = {start}
        stack: list[Cell] = [start]
        while stack:
            cell = stack.pop()
            for nb, _cost in space.neighbors(cell):
                if nb not in seen:
                    seen.add(nb)
                    stack.append(nb)
        return frozenset(seen)

    def _bounds(self, free: frozenset[Cell]) -> tuple[int, int, int, int]:
        rows = [r for r, _c in free]
        cols = [c for _r, c in free]
        return min(rows), max(rows), min(cols), max(cols)

    # --- corner <-> cell for the shared list[Cell] / viz contract ----------
    def _cell_of(self, free: frozenset[Cell], p: _Point) -> Cell:
        # start / goal are cell centres (half-integer) -> their own cell.
        row = p[1] - 0.5
        col = p[0] - 0.5
        if abs(row - round(row)) < 1e-6 and abs(col - round(col)) < 1e-6:
            return (int(round(row)), int(round(col)))
        # A corner (integer x, y) has no cell of its own; render it at the first
        # incident FREE cell (fixed order) so the list[Cell] path and the replay
        # stay on free space. cost is the exact corner geometry, not this snap.
        x, y = int(round(p[0])), int(round(p[1]))
        for cx, cy in ((x - 1, y - 1), (x - 1, y), (x, y - 1), (x, y)):
            if _cell_free(free, cx, cy):
                return (cy, cx)
        return (y, x)

    def _reconstruct(
        self,
        free: frozenset[Cell],
        parent: dict[_Point, _Point],
        s_pt: _Point,
        goal_root: _Point,
        start: Cell,
        goal: Cell,
    ) -> list[Cell]:
        chain: list[_Point] = [goal_root]
        node = goal_root
        while node != s_pt:
            node = parent[node]
            chain.append(node)
        chain.reverse()
        path: list[Cell] = [start]
        for pt in chain:
            cell = self._cell_of(free, pt)
            if cell != path[-1]:
                path.append(cell)
        if path[-1] != goal:
            path.append(goal)
        return path

    # --- successor generation: interval projection (cone + flat) -----------
    def _successors(
        self,
        free: frozenset[Cell],
        root: _Point,
        r0: int,
        r1: int,
        c0: int,
        c1: int,
        eps: float,
    ) -> list[tuple[_Point, tuple[float, float, float]]]:
        rx, ry = root
        found: dict[_Point, tuple[float, float, float]] = {}
        span_lo, span_hi = float(c0) - 2.0, float(c1) + 3.0

        # Cone successors: sweep the root's visibility away row by row in both
        # directions, projecting the interval and re-splitting at obstacle walls.
        for direction in (1, -1):
            y = math.floor(ry) + 1 if direction > 0 else math.ceil(ry) - 1
            splits = [float(e) for e in range(int(span_lo) - 1, int(span_hi) + 2)]
            beams = _clear_pieces(free, root, float(y), span_lo, span_hi, splits, eps)
            self._emit(free, root, float(y), beams, found, eps)
            steps = 0
            while beams and r0 - 2 <= y <= r1 + 2 and steps < 400:
                yn = y + direction
                child: list[tuple[float, float]] = []
                for a, b in beams:
                    an = _proj_x(rx, ry, a, float(y), float(yn))
                    bn = _proj_x(rx, ry, b, float(y), float(yn))
                    lo, hi = (an, bn) if an < bn else (bn, an)
                    cand_splits: list[float] = []
                    for e in range(math.floor(lo) - 1, math.floor(hi) + 2):
                        cand_splits.append(float(e))
                        cand_splits.append(_proj_x(rx, ry, float(e), float(y), float(yn)))
                    child.extend(_clear_pieces(free, root, float(yn), lo, hi, cand_splits, eps))
                y = yn
                beams = _merge(child, eps)
                self._emit(free, root, float(y), beams, found, eps)
                steps += 1

        # Flat successors: walk along the root's own row (corner roots only) to the
        # obstacle corners it can reach without leaving the row.
        if abs(ry - round(ry)) < eps:
            yr = int(round(ry))
            for direction in (1, -1):
                x = int(round(rx))
                steps = 0
                while c0 - 2 <= x <= c1 + 2 and steps < 400:
                    col = x if direction > 0 else x - 1
                    if not _cell_free(free, col, yr - 1) and not _cell_free(free, col, yr):
                        break
                    x += direction
                    steps += 1
                    corner = (float(x), float(yr))
                    if _is_corner(free, x, yr) and _seg_clear(free, root, corner, eps):
                        found.setdefault(corner, (float(yr), float(x), float(x)))

        found.pop(root, None)
        # Deterministic order (row, col) so both languages expand identically.
        return sorted(found.items(), key=lambda kv: (kv[0][1], kv[0][0]))

    def _emit(
        self,
        free: frozenset[Cell],
        root: _Point,
        y: float,
        beams: list[tuple[float, float]],
        found: dict[_Point, tuple[float, float, float]],
        eps: float,
    ) -> None:
        iy = int(round(y))
        for a, b in beams:
            for x in range(math.ceil(a - 1e-6), math.floor(b + 1e-6) + 1):
                if _is_corner(free, x, iy) and _seg_clear(free, root, (float(x), y), eps):
                    found.setdefault((float(x), y), (y, a, b))
