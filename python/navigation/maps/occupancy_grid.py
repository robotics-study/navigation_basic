"""OccupancyGrid2D — ROS-style grid map providing DiscreteSpace + SamplingSpace.

world<->grid conversion lives ONLY here (map layer owns the coordinate frames).
Discrete state is a Cell (row, col); sampling state is a world Point (x, y).
"""

from __future__ import annotations

import math
from collections.abc import Callable

import numpy as np

from navigation.core.capabilities import Capability, MapBase
from navigation.core.types import Cell, Footprint, Point, Pose

_SQRT2 = math.sqrt(2.0)
# Sentinel "infinity" for the 1D distance transform below. Must stay finite (an
# actual inf would produce a 0/0 -> nan in the lower-envelope intersection formula)
# but far larger than any squared distance a real grid can produce.
_EDT_INF = 1e15


def _dt_1d(f: np.ndarray) -> np.ndarray:
    """Exact 1D squared-distance lower envelope (Felzenszwalb & Huttenlocher 2004,
    "Distance Transforms of Sampled Functions", §2). `f[q]` is 0 at a source and
    `_EDT_INF` elsewhere; returns, for every q, min_{q'} (q-q')^2 + f[q'].
    """
    n = f.shape[0]
    d = np.empty(n, dtype=np.float64)
    v = np.zeros(n, dtype=np.int64)  # q-indices of parabolas kept in the envelope
    z = np.empty(n + 1, dtype=np.float64)  # envelope boundaries between kept parabolas
    k = 0
    v[0] = 0
    z[0] = -_EDT_INF
    z[1] = _EDT_INF
    for q in range(1, n):
        while True:
            s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
            if s <= z[k]:
                k -= 1
            else:
                break
        k += 1
        v[k] = q
        z[k] = s
        z[k + 1] = _EDT_INF
    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k += 1
        d[q] = float((q - v[k]) ** 2) + f[v[k]]
    return d


def _squared_edt(non_free: np.ndarray) -> np.ndarray:
    """2-pass exact squared Euclidean distance transform to the `non_free` mask
    (Felzenszwalb & Huttenlocher 2004): 1D transform along each column, then along
    each row of the result. Distances are in cell units (squared); the caller scales
    by resolution and takes sqrt once, so cross-language results agree bit-for-bit
    up to that single final sqrt.
    """
    h, w = non_free.shape
    f = np.where(non_free, 0.0, _EDT_INF)
    partial = np.empty_like(f)
    for col in range(w):
        partial[:, col] = _dt_1d(f[:, col])
    out = np.empty_like(f)
    for row in range(h):
        out[row, :] = _dt_1d(partial[row, :])
    return out
# 8-connected moves; diagonals cost sqrt(2), orthogonals 1.0.
_MOVES_8 = [
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, _SQRT2),
    (-1, 1, _SQRT2),
    (1, -1, _SQRT2),
    (1, 1, _SQRT2),
]
_MOVES_4 = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]


class OccupancyGrid2D(MapBase):
    def __init__(
        self,
        pixels: np.ndarray,
        resolution: float,
        origin: tuple[float, float, float],
        occupied_thresh: float = 0.65,
        free_thresh: float = 0.196,
        connectivity: int = 8,
        seed: int = 0,
    ) -> None:
        if connectivity not in (4, 8):
            raise ValueError(f"connectivity must be 4 or 8, got {connectivity}")
        self._height, self._width = int(pixels.shape[0]), int(pixels.shape[1])
        self._resolution = resolution
        self._origin_x, self._origin_y = origin[0], origin[1]
        self._connectivity = connectivity
        # occupied_thresh is accepted for ROS yaml signature compatibility but not
        # used: traversability is decided by free_thresh alone (occupied/unknown are
        # blocked), so a separate occupied cutoff would be redundant.
        # occ = 1 - p/255; free iff occ <= free_thresh; anything not clearly free
        # (occupied or unknown) is non-traversable.
        occ = 1.0 - pixels.astype(np.float64) / 255.0
        self._free = occ <= free_thresh
        self._moves = _MOVES_8 if connectivity == 8 else _MOVES_4
        self._rng = np.random.default_rng(seed)
        # Lazy EDT cache (cell-unit squared distances): computed on first
        # distance_to_nearest() call so the existing global_planning suite, which
        # never touches ObstacleQuery, pays nothing for it.
        self._edt_sq: np.ndarray | None = None

    # --- dimensions -------------------------------------------------------
    @property
    def height(self) -> int:
        return self._height

    @property
    def width(self) -> int:
        return self._width

    @property
    def resolution(self) -> float:
        return self._resolution

    def free_mask(self) -> np.ndarray:
        """Boolean [H, W] mask of traversable cells (read-only view for viz)."""
        return self._free

    # --- coordinate frames (owned here only) ------------------------------
    def cell_to_world(self, row: int, col: int) -> Point:
        x = self._origin_x + (col + 0.5) * self._resolution
        y = self._origin_y + ((self._height - 1 - row) + 0.5) * self._resolution
        return (x, y)

    def world_to_cell(self, x: float, y: float) -> Cell:
        col = int(math.floor((x - self._origin_x) / self._resolution))
        row = (self._height - 1) - int(math.floor((y - self._origin_y) / self._resolution))
        return (row, col)

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self._height and 0 <= col < self._width

    def is_free_cell(self, row: int, col: int) -> bool:
        return self.in_bounds(row, col) and bool(self._free[row, col])

    # --- capabilities -----------------------------------------------------
    def capabilities(self) -> set[Capability]:
        return {
            Capability.DISCRETE_SPACE,
            Capability.SAMPLING_SPACE,
            Capability.LINE_OF_SIGHT_SPACE,
            Capability.DYNAMIC_GRID_SPACE,
            Capability.SE2_COLLISION_SPACE,
            Capability.OBSTACLE_QUERY,
        }

    # --- DiscreteSpace[Cell] ---------------------------------------------
    def _neighbors_impl(
        self, s: Cell, is_free: Callable[[int, int], bool]
    ) -> list[tuple[Cell, float]]:
        # Single 8-move + corner-cut worker shared by neighbors() (truth predicate)
        # and passable_neighbors() (belief predicate). ``is_free`` decides both cell
        # entry and the diagonal corner rule so both callers forbid corner-cutting
        # identically.
        row, col = s
        out: list[tuple[Cell, float]] = []
        for dr, dc, cost in self._moves:
            nr, nc = row + dr, col + dc
            if not is_free(nr, nc):
                continue
            # Forbid corner-cutting: a diagonal needs both shared orthogonals free.
            if dr != 0 and dc != 0:
                if not (is_free(row + dr, col) and is_free(row, col + dc)):
                    continue
            out.append(((nr, nc), cost))
        return out

    def neighbors(self, s: Cell) -> list[tuple[Cell, float]]:
        # Ground-truth successors: enterable iff in bounds and actually free.
        return self._neighbors_impl(s, self.is_free_cell)

    # --- DynamicGridSpace[Cell] ------------------------------------------
    def passable_neighbors(self, s: Cell, blocked: set[Cell]) -> list[tuple[Cell, float]]:
        # Belief successors: enterable iff in bounds and not (yet) known blocked; real
        # occupancy is invisible here. Same worker + corner rule as neighbors().
        def believed_free(row: int, col: int) -> bool:
            return self.in_bounds(row, col) and (row, col) not in blocked

        return self._neighbors_impl(s, believed_free)

    def is_blocked(self, s: Cell) -> bool:
        # Occupied OR out of bounds — is_free_cell is already false for both.
        return not self.is_free_cell(*s)

    def heuristic(self, a: Cell, b: Cell) -> float:
        dr = abs(a[0] - b[0])
        dc = abs(a[1] - b[1])
        if self._connectivity == 8:
            # Octile distance: admissible for 8-connected unit/sqrt2 moves.
            # Evaluated as (hi - lo) + sqrt2*lo (identical operation order to the
            # C++ mirror) so f-values match bit-for-bit and tie-breaking, hence
            # trace event streams, stay identical across languages.
            lo = min(dr, dc)
            hi = max(dr, dc)
            return float(hi - lo) + _SQRT2 * float(lo)
        return float(dr + dc)  # Manhattan for 4-connected.

    # --- LineOfSightSpace[Cell] ------------------------------------------
    def line_of_sight(self, a: Cell, b: Cell) -> bool:
        # Straight segment between cell centres, tested by the same supercover
        # (Amanatides & Woo 1987) + corner-cut-forbidden rule as neighbors(), so a
        # LOS-visible pair is exactly a legal straight move (Nash et al. 2007).
        return self.is_motion_valid(self.cell_to_world(*a), self.cell_to_world(*b))

    # --- SamplingSpace[Point] --------------------------------------------
    def sample(self) -> Point:
        x = self._origin_x + float(self._rng.uniform(0.0, self._width * self._resolution))
        y = self._origin_y + float(self._rng.uniform(0.0, self._height * self._resolution))
        return (x, y)

    def is_state_valid(self, s: Point) -> bool:
        row, col = self.world_to_cell(s[0], s[1])
        return self.is_free_cell(row, col)

    # --- SE2CollisionSpace[Pose] -----------------------------------------
    def is_collision(self, footprint: Footprint, pose: Pose) -> bool:
        # Inscribed-disc footprint is orientation-invariant, so theta is unused (a
        # polygon footprint would use it) — Dolgov et al. 2008. Collision iff any
        # occupied or out-of-bounds cell overlaps the disc; exact disc–cell overlap
        # via squared distance to the cell rectangle (no sqrt, no trig → bit-identical
        # with the C++ mirror). world<->cell stays here.
        x, y, _theta = pose
        r = footprint.inscribed_radius
        r2 = r * r
        half = self._resolution * 0.5
        lo_row, lo_col = self.world_to_cell(x - r, y + r)  # y+r → smaller row
        hi_row, hi_col = self.world_to_cell(x + r, y - r)
        for row in range(lo_row, hi_row + 1):
            for col in range(lo_col, hi_col + 1):
                if self.is_free_cell(row, col):  # in-bounds & free → skip
                    continue
                cx, cy = self.cell_to_world(row, col)  # occupied OR out-of-bounds
                dx = x - min(max(x, cx - half), cx + half)
                dy = y - min(max(y, cy - half), cy + half)
                if dx * dx + dy * dy <= r2:
                    return True
        return False

    # --- ObstacleQuery (extends SE2CollisionSpace[Pose]) -------------------
    def _edt(self) -> np.ndarray:
        if self._edt_sq is None:
            h, w = self._height, self._width
            # 1-cell non-free border padding makes the map edge itself a source, so
            # a reactive planner near the boundary sees it as an obstacle (occupied
            # OR out-of-bounds already means "non-free" everywhere else in this class).
            padded = np.ones((h + 2, w + 2), dtype=bool)
            padded[1 : h + 1, 1 : w + 1] = ~self._free
            self._edt_sq = _squared_edt(padded)[1 : h + 1, 1 : w + 1]
        return self._edt_sq

    def distance_to_nearest(self, p: Point) -> float:
        row, col = self.world_to_cell(p[0], p[1])
        if not self.in_bounds(row, col):
            # p's own cell is itself non-free (out of bounds), so it is its own
            # nearest non-free cell.
            return 0.0
        # Squared distances are integer cell-unit values; sqrt is the only
        # floating-point step, so C++/Python agree up to that single rounding.
        return math.sqrt(self._edt()[row, col]) * self._resolution

    def occupied_within(self, center: Point, radius: float) -> list[Point]:
        # Same bounding-box convention as is_collision: world_to_cell(x-r, y+r) is
        # the lower row/col corner because +y maps to a smaller row index.
        x, y = center
        r2 = radius * radius
        lo_row, lo_col = self.world_to_cell(x - radius, y + radius)
        hi_row, hi_col = self.world_to_cell(x + radius, y - radius)
        out: list[Point] = []
        for row in range(lo_row, hi_row + 1):  # row asc, col asc: fixes summation
            for col in range(lo_col, hi_col + 1):  # order for PF/VFH force/bin sums
                if self.is_free_cell(row, col):  # in-bounds & free → not a source
                    continue
                cx, cy = self.cell_to_world(row, col)  # occupied OR out-of-bounds
                dx, dy = cx - x, cy - y
                if dx * dx + dy * dy <= r2:
                    out.append((cx, cy))
        return out

    def _is_free_uv(self, iu: int, iv: int) -> bool:
        # (u, v) grid coords count up from the origin (bottom-left); rows count
        # down from the top image row.
        return self.is_free_cell(self._height - 1 - iv, iu)

    def is_motion_valid(self, a: Point, b: Point) -> bool:
        # Supercover grid traversal (Amanatides & Woo, 1987): visits every cell
        # the segment crosses. Point sampling misses a corner clip whose in-cell
        # chord is shorter than the sample spacing, letting edges cut obstacle
        # corners.
        u0 = (a[0] - self._origin_x) / self._resolution
        v0 = (a[1] - self._origin_y) / self._resolution
        u1 = (b[0] - self._origin_x) / self._resolution
        v1 = (b[1] - self._origin_y) / self._resolution
        iu, iv = int(math.floor(u0)), int(math.floor(v0))
        ju, jv = int(math.floor(u1)), int(math.floor(v1))
        if not self._is_free_uv(iu, iv):
            return False
        du, dv = u1 - u0, v1 - v0
        step_u = 1 if du > 0.0 else -1
        step_v = 1 if dv > 0.0 else -1
        t_delta_u = abs(1.0 / du) if du != 0.0 else math.inf
        t_delta_v = abs(1.0 / dv) if dv != 0.0 else math.inf
        # Parametric distance (in units of t) from the start to the first grid
        # line crossed on each axis; an axis with no motion never crosses one
        # (explicit inf also avoids 0 * inf = nan when the start sits on a line).
        if du != 0.0:
            frac_u = (math.floor(u0) + 1.0 - u0) if du > 0.0 else (u0 - math.floor(u0))
            t_max_u = frac_u * t_delta_u
        else:
            t_max_u = math.inf
        if dv != 0.0:
            frac_v = (math.floor(v0) + 1.0 - v0) if dv > 0.0 else (v0 - math.floor(v0))
            t_max_v = frac_v * t_delta_v
        else:
            t_max_v = math.inf
        while iu != ju or iv != jv:
            # Clamp exhausted axes so float drift in t_max can never step past
            # the end cell (termination is by cell index, not by t).
            if iv == jv or t_max_u < t_max_v:
                iu += step_u
                t_max_u += t_delta_u
            elif iu == ju or t_max_v < t_max_u:
                iv += step_v
                t_max_v += t_delta_v
            else:
                # Exact corner crossing: same rule as neighbors() — passing
                # through a corner needs both shared orthogonal cells free.
                if not (self._is_free_uv(iu + step_u, iv) and self._is_free_uv(iu, iv + step_v)):
                    return False
                iu += step_u
                iv += step_v
                t_max_u += t_delta_u
                t_max_v += t_delta_v
            if not self._is_free_uv(iu, iv):
                return False
        return True

    def distance(self, a: Point, b: Point) -> float:
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def steer(self, a: Point, b: Point, eta: float) -> Point:
        dist = self.distance(a, b)
        if dist <= eta or dist == 0.0:
            return b
        scale = eta / dist
        return (a[0] + (b[0] - a[0]) * scale, a[1] + (b[1] - a[1]) * scale)
