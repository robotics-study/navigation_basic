"""OccupancyGrid2D — ROS-style grid map providing DiscreteSpace + SamplingSpace.

world<->grid conversion lives ONLY here (map layer owns the coordinate frames).
Discrete state is a Cell (row, col); sampling state is a world Point (x, y).
"""

from __future__ import annotations

import math

import numpy as np

from navigation.core.capabilities import Capability, MapBase
from navigation.core.types import Cell, Point

_SQRT2 = math.sqrt(2.0)
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
        return {Capability.DISCRETE_SPACE, Capability.SAMPLING_SPACE}

    # --- DiscreteSpace[Cell] ---------------------------------------------
    def neighbors(self, s: Cell) -> list[tuple[Cell, float]]:
        row, col = s
        out: list[tuple[Cell, float]] = []
        for dr, dc, cost in self._moves:
            nr, nc = row + dr, col + dc
            if not self.is_free_cell(nr, nc):
                continue
            # Forbid corner-cutting: a diagonal needs both shared orthogonals free.
            if dr != 0 and dc != 0:
                if not (self.is_free_cell(row + dr, col) and self.is_free_cell(row, col + dc)):
                    continue
            out.append(((nr, nc), cost))
        return out

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

    # --- SamplingSpace[Point] --------------------------------------------
    def sample(self) -> Point:
        x = self._origin_x + float(self._rng.uniform(0.0, self._width * self._resolution))
        y = self._origin_y + float(self._rng.uniform(0.0, self._height * self._resolution))
        return (x, y)

    def is_state_valid(self, s: Point) -> bool:
        row, col = self.world_to_cell(s[0], s[1])
        return self.is_free_cell(row, col)

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
