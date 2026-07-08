"""Anya — the TRUE Euclidean any-angle optimum (turns at grid corners, not cell
centres): equals the corner-visibility optimum, never exceeds Visibility A*, and
the open-space / no-path / parameter-validation contracts."""

from __future__ import annotations

import heapq
import io
import json
import math
import random
from pathlib import Path

import pytest
from conftest import config, grid_from, open_grid, write_config

from navigation.core.params import ParamError, ParamSet
from navigation.core.trace import TraceRecorder
from navigation.core.types import Cell
from navigation.global_planning.search.anya import Anya
from navigation.global_planning.search.theta_star import ThetaStar
from navigation.global_planning.search.visibility_astar import VisibilityAStarPlanner

# --- independent ground truth: corner visibility graph ----------------------
# Turning points of a shortest Euclidean path in a polygonal (blocked-cell) domain
# lie only at convex obstacle corners (grid vertices), so the optimum is a Dijkstra
# over {start, goal, all obstacle corners} weighted by straight-line length. The
# line-of-sight oracle below is computed by a DIFFERENT method than the planner
# (analytic Liang-Barsky clipping against open cell squares, not midpoint sampling),
# so agreement is a real correctness check, not a restatement of the planner.

_Pt = tuple[float, float]


def _reachable(grid: object, start: Cell) -> set[Cell]:
    seen: set[Cell] = {start}
    stack = [start]
    while stack:
        cur = stack.pop()
        for nb, _c in grid.neighbors(cur):  # type: ignore[attr-defined]
            if nb not in seen:
                seen.add(nb)
                stack.append(nb)
    return seen


def _free(cells: set[Cell], cx: int, cy: int) -> bool:
    return (cy, cx) in cells


def _open_slab(p: float, d: float, lo: float, hi: float) -> tuple[float, float] | None:
    # t-interval where p + t*d lies strictly inside the open interval (lo, hi); None
    # if empty. An axis-parallel component that stays exactly on a boundary (d == 0
    # and p == lo/hi) yields None, so a segment grazing a cell edge is NOT counted as
    # crossing that cell's interior.
    if d == 0.0:
        return (-math.inf, math.inf) if lo < p < hi else None
    t0, t1 = (lo - p) / d, (hi - p) / d
    return (t0, t1) if t0 <= t1 else (t1, t0)


def _seg_clear(cells: set[Cell], p: _Pt, q: _Pt, eps: float = 1e-9) -> bool:
    """Exact grazing-aware any-angle line of sight, computed independently of the
    planner. A segment is valid iff (1) it enters no blocked cell interior (analytic
    Liang-Barsky clip against each cell's OPEN square — a positive-length overlap is a
    real interior crossing; touching an edge is not), (2) it does not run along a grid
    edge whose two sides are both blocked, and (3) it does not corner-cut through a
    pinch of two diagonally blocked cells. Edge-grazing an obstacle corner is allowed,
    matching the true continuous Euclidean model (Harabor et al. 2016)."""
    px, py = p
    qx, qy = q
    if p == q:
        return True
    dx, dy = qx - px, qy - py
    xlo, xhi = sorted((px, qx))
    ylo, yhi = sorted((py, qy))
    # (1) interior crossing
    for cy in range(math.floor(ylo), math.ceil(yhi)):
        for cx in range(math.floor(xlo), math.ceil(xhi)):
            if _free(cells, cx, cy):
                continue
            sx = _open_slab(px, dx, cx, cx + 1)
            sy = _open_slab(py, dy, cy, cy + 1)
            if sx is None or sy is None:
                continue
            if min(sx[1], sy[1], 1.0) - max(sx[0], sy[0], 0.0) > 1e-9:
                return False
    # (2) both-sides-blocked grid edge (axis segment exactly on an integer line)
    if dx == 0.0 and abs(px - round(px)) < eps:
        xi = round(px)
        for row in range(math.floor(ylo), math.ceil(yhi)):
            if ylo < row + 1 and yhi > row and not _free(cells, xi - 1, row) \
                    and not _free(cells, xi, row):
                return False
    if dy == 0.0 and abs(py - round(py)) < eps:
        yi = round(py)
        for col in range(math.floor(xlo), math.ceil(xhi)):
            if xlo < col + 1 and xhi > col and not _free(cells, col, yi - 1) \
                    and not _free(cells, col, yi):
                return False
    # (3) pinch (corner-cutting) at an interior lattice corner of a diagonal segment
    if dx != 0.0 and dy != 0.0:
        for ix in range(math.ceil(xlo), math.floor(xhi) + 1):
            t = (ix - px) / dx
            if t <= eps or t >= 1.0 - eps:
                continue
            y = py + t * dy
            if abs(y - round(y)) < eps:
                iy = round(y)
                if (dx > 0.0) == (dy > 0.0):
                    if not _free(cells, ix - 1, iy) and not _free(cells, ix, iy - 1):
                        return False
                elif not _free(cells, ix - 1, iy - 1) and not _free(cells, ix, iy):
                    return False
    return True


def _corner_optimum(grid: object, start: Cell, goal: Cell) -> float:
    cells = _reachable(grid, start)  # same domain the planner observes
    if goal not in cells:
        return math.inf
    rows = [r for r, _c in cells]
    cols = [c for _r, c in cells]
    verts: list[_Pt] = []
    for y in range(min(rows), max(rows) + 2):
        for x in range(min(cols), max(cols) + 2):
            around = [not _free(cells, x - 1, y - 1), not _free(cells, x, y - 1),
                      not _free(cells, x - 1, y), not _free(cells, x, y)]
            if any(around) and not all(around):
                verts.append((float(x), float(y)))
    s: _Pt = (start[1] + 0.5, start[0] + 0.5)
    g: _Pt = (goal[1] + 0.5, goal[0] + 0.5)
    nodes = list(dict.fromkeys([s, g, *verts]))
    dist = {s: 0.0}
    pq: list[tuple[float, int]] = [(0.0, 0)]
    settled: set[int] = set()
    while pq:
        d, ui = heapq.heappop(pq)
        u = nodes[ui]
        if ui in settled:
            continue
        settled.add(ui)
        if u == g:
            return d
        for vi, v in enumerate(nodes):
            if vi in settled or v == u:
                continue
            if _seg_clear(cells, u, v):
                nd = d + math.dist(u, v)
                if nd < dist.get(v, math.inf):
                    dist[v] = nd
                    heapq.heappush(pq, (nd, vi))
    return math.inf


# (a) OPTIMALITY: corner turning beats cell-centre ---------------------------

def test_anya_turns_at_corner_and_beats_cell_centre() -> None:
    # A vertical bar hides the goal; the shortest route grazes the bar's top-left
    # grid corner (2,2). Turning THERE (not at any cell centre) gives the true
    # Euclidean optimum 2*sqrt(8.5); Visibility A* / Theta*, pinned to cell centres,
    # must be strictly longer. This is Anya's defining property.
    grid = grid_from([".....", ".....", "..#..", "..#..", "....."])
    start, goal = (4, 0), (0, 4)
    res = Anya(config("anya")).plan(grid, start, goal)
    assert res.success
    assert res.path[0] == start and res.path[-1] == goal
    # exact corner-turning optimum: start->corner(2,2)->goal, each leg sqrt(8.5)
    assert res.cost == pytest.approx(2.0 * math.sqrt(8.5))
    # equals the independent corner-visibility optimum
    assert res.cost == pytest.approx(_corner_optimum(grid, start, goal))
    # strictly shorter than the cell-centre any-angle planners
    vis = VisibilityAStarPlanner(config("visibility_astar")).plan(grid, start, goal)
    theta = ThetaStar(config("theta_star")).plan(grid, start, goal)
    assert res.cost < vis.cost - 1e-9
    assert res.cost < theta.cost - 1e-9


def test_anya_matches_corner_optimum_on_asymmetric_map() -> None:
    # A second, asymmetric instance: guards against a projection correct only on the
    # symmetric bend above. Anya must still equal the corner-visibility optimum and
    # never exceed Visibility A*.
    grid = grid_from([
        "........",
        "...##...",
        "...##...",
        ".....#..",
        "..#.....",
        "........",
    ])
    start, goal = (5, 0), (0, 7)
    res = Anya(config("anya")).plan(grid, start, goal)
    assert res.success
    assert res.cost == pytest.approx(_corner_optimum(grid, start, goal))
    vis = VisibilityAStarPlanner(config("visibility_astar")).plan(grid, start, goal)
    assert res.cost <= vis.cost + 1e-9


# (a') EDGE-GRAZING optimum: a taut leg that hugs an obstacle edge --------------
# These are the instances where snapping the on-grid-line line-of-sight sample to one
# side (floor) forbids a legal edge-grazing leg and inflates the cost. The optimum is
# only reachable if a segment that runs exactly along a blocked cell's boundary (but
# through no interior) is admitted.

def test_anya_optimum_hugs_blocked_cell_edge() -> None:
    # (3,1)->(2,3)->(3,3): the middle leg grazes the top edge of blocked cell (3,2)
    # without entering its interior. Optimum 1 + sqrt(2); a floor-snapped LOS instead
    # detours over (1,1) at cost 2*sqrt(8)/... > 3.7.
    grid = grid_from([".##.", "..#.", "....", "..#."])
    start, goal = (3, 1), (3, 3)
    res = Anya(config("anya")).plan(grid, start, goal)
    assert res.success
    assert res.cost == pytest.approx(1.0 + math.sqrt(2.0))
    assert res.cost == pytest.approx(_corner_optimum(grid, start, goal))


def test_anya_never_exceeds_visibility_astar_on_corner_turn() -> None:
    # A corner-turning instance where the floor-snapped LOS made Anya STRICTLY worse
    # than Visibility A* (cell-centre). The true any-angle optimum must be <= it.
    grid = grid_from([".......", "#.##...", "##...#.", ".....#.", "....#.."])
    start, goal = (2, 6), (2, 2)
    res = Anya(config("anya")).plan(grid, start, goal)
    assert res.success
    assert res.cost == pytest.approx(_corner_optimum(grid, start, goal))
    vis = VisibilityAStarPlanner(config("visibility_astar")).plan(grid, start, goal)
    assert res.cost <= vis.cost + 1e-9


# (a'') PROPERTY: Anya == exact grazing-aware corner optimum on random maps ------

@pytest.mark.parametrize("seed", range(600))
def test_anya_matches_exact_oracle_on_random_grids(seed: int) -> None:
    # Anya's cost must equal the independent, grazing-aware corner-visibility optimum
    # on every solvable random instance. The oracle uses analytic Liang-Barsky clips
    # (not the planner's sampled LOS), so this pins true continuous Euclidean
    # optimality, not a self-consistency check.
    rng = random.Random(seed)
    rows = rng.randint(3, 6)
    cols = rng.randint(3, 6)
    density = rng.uniform(0.12, 0.32)
    ascii_rows = [
        "".join("#" if rng.random() < density else "." for _ in range(cols))
        for _ in range(rows)
    ]
    free_cells = [(r, c) for r in range(rows) for c in range(cols) if ascii_rows[r][c] == "."]
    if len(free_cells) < 2:
        pytest.skip("degenerate map")
    start = rng.choice(free_cells)
    goal = rng.choice(free_cells)
    if start == goal:
        pytest.skip("start == goal")
    grid = grid_from(ascii_rows)
    if goal not in _reachable(grid, start):
        pytest.skip("goal unreachable")
    res = Anya(config("anya")).plan(grid, start, goal)
    assert res.success
    assert res.cost == pytest.approx(_corner_optimum(grid, start, goal), abs=1e-9)


# (b) open space: straight line ----------------------------------------------

def test_anya_returns_straight_line_when_visible() -> None:
    grid = open_grid(4, 4)
    start, goal = (3, 0), (0, 3)
    res = Anya(config("anya")).plan(grid, start, goal)
    assert res.success
    assert res.path[0] == start and res.path[-1] == goal
    assert res.cost == pytest.approx(math.hypot(3, 3))  # single unobstructed segment


# trace surfaces the (root, interval) node ------------------------------------

def test_anya_emits_root_interval_data() -> None:
    buf = io.StringIO()
    Anya(config("anya")).plan(grid_from([".....", ".....", "..#..", "..#..", "....."]),
                              (4, 0), (0, 4), TraceRecorder(buf))
    intervals = [
        json.loads(line)["data"]
        for line in buf.getvalue().splitlines()
        if json.loads(line)["event"] == "edge_added"
    ]
    assert intervals, "any-angle edges must be tagged with their (root, interval) node"
    for d in intervals:
        assert d.keys() == {"row", "col_lo", "col_hi"}
        assert d["col_lo"] <= d["col_hi"] + 1e-9


# (c) no-path case -----------------------------------------------------------

def test_anya_no_path_when_walled_off() -> None:
    grid = grid_from(["..#..", "..#..", "..#..", "..#..", "..#.."])
    res = Anya(config("anya")).plan(grid, (0, 0), (0, 4))
    assert not res.success
    assert res.path == []
    assert res.cost == 0.0


# (d) parameter validation failure -------------------------------------------

def test_anya_rejects_out_of_range_epsilon(tmp_path: Path) -> None:
    # anya declares vertex_epsilon in [1e-12, 1e-3]; an above-max default must fail
    # at load time so an invalid tolerance can never reach plan().
    cfg = write_config(
        tmp_path / "bad_anya.yaml",
        "anya",
        [{"name": "vertex_epsilon", "type": "float", "default": 1.0,
          "min": 1.0e-12, "max": 1.0e-3, "description": "above max"}],
    )
    with pytest.raises(ParamError):
        ParamSet.from_yaml(cfg)
