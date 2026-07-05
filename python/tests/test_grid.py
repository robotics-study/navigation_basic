"""OccupancyGrid2D geometry, occupancy, neighbors, capabilities."""

from __future__ import annotations

import math

import numpy as np
import pytest
from conftest import grid_from, open_grid

from nav_study.core.capabilities import Capability
from nav_study.maps.occupancy_grid import OccupancyGrid2D


def test_world_cell_round_trip() -> None:
    grid = open_grid(4, 4)
    for row in range(4):
        for col in range(4):
            x, y = grid.cell_to_world(row, col)
            assert grid.world_to_cell(x, y) == (row, col)


def test_known_geometry() -> None:
    # origin bottom-left, row 0 = top: top-left cell center sits high in world y.
    grid = open_grid(4, 4)
    assert grid.cell_to_world(0, 0) == pytest.approx((0.5, 3.5))
    assert grid.world_to_cell(0.5, 3.5) == (0, 0)


def test_occupancy_thresholding() -> None:
    # 255 -> free, 0 -> occupied, mid-gray -> unknown -> blocked.
    pixels = np.array([[255, 0, 128]], dtype=np.uint16)
    grid = OccupancyGrid2D(pixels, resolution=1.0, origin=(0.0, 0.0, 0.0))
    assert grid.is_free_cell(0, 0) is True
    assert grid.is_free_cell(0, 1) is False
    assert grid.is_free_cell(0, 2) is False


def test_corner_cut_forbidden() -> None:
    # Occupied orthogonal neighbor blocks the diagonal through the corner.
    grid = grid_from([".#.", "...", "..."])
    succ = {cell for cell, _ in grid.neighbors((1, 1))}
    assert (0, 0) not in succ  # blocked by occupied (0,1)
    assert (0, 2) not in succ  # blocked by occupied (0,1)
    assert (2, 0) in succ and (2, 2) in succ  # lower diagonals stay open


def test_diagonal_cost_and_octile_heuristic() -> None:
    grid = open_grid(5, 5, connectivity=8)
    costs = {cell: c for cell, c in grid.neighbors((2, 2))}
    assert costs[(1, 2)] == pytest.approx(1.0)
    assert costs[(1, 1)] == pytest.approx(math.sqrt(2))
    # Octile heuristic equals the true free-space diagonal cost (admissible, tight).
    assert grid.heuristic((0, 0), (3, 3)) == pytest.approx(3 * math.sqrt(2))
    assert grid.heuristic((0, 0), (0, 3)) == pytest.approx(3.0)


def test_manhattan_heuristic_4conn() -> None:
    grid = open_grid(5, 5, connectivity=4)
    assert grid.heuristic((0, 0), (3, 2)) == pytest.approx(5.0)
    assert all(len(cell) == 2 for cell, _ in grid.neighbors((2, 2)))
    assert len(grid.neighbors((2, 2))) == 4


def test_dimensions_and_free_mask() -> None:
    grid = grid_from(["..#", "..."])
    assert (grid.height, grid.width) == (2, 3)
    assert grid.resolution == 1.0
    assert grid.free_mask().tolist() == [[True, True, False], [True, True, True]]


def test_capabilities_and_supports() -> None:
    grid = open_grid(3, 3)
    assert grid.capabilities() == {Capability.DISCRETE_SPACE, Capability.SAMPLING_SPACE}
    assert grid.supports(Capability.DISCRETE_SPACE)
    assert grid.supports(Capability.SAMPLING_SPACE)
    assert not grid.supports(Capability.OBSTACLE_QUERY)


def test_sampling_validity_and_motion() -> None:
    grid = grid_from(["...", "###", "..."])
    assert grid.is_state_valid((0.5, 2.5))  # top row free
    assert not grid.is_state_valid((0.5, 1.5))  # middle wall
    # A motion crossing the wall is invalid; one within the free top row is valid.
    assert not grid.is_motion_valid((0.5, 2.5), (0.5, 0.5))
    assert grid.is_motion_valid((0.5, 2.5), (2.5, 2.5))


def test_motion_corner_clip_rejected() -> None:
    # Center cell spans [2,3)x[2,3). The segment y = x + 0.96 grazes its
    # top-left corner with an in-cell chord (~0.057) far shorter than any fixed
    # sample spacing — the traversal must still reject it, in both directions.
    grid = grid_from([".....", ".....", "..#..", ".....", "....."])
    assert not grid.is_motion_valid((1.1, 2.06), (3.1, 4.06))
    assert not grid.is_motion_valid((3.1, 4.06), (1.1, 2.06))
    # y = x + 1.04 passes just outside the same corner and must stay valid.
    assert grid.is_motion_valid((1.1, 2.14), (3.1, 4.14))


def test_motion_exact_corner_crossing_follows_corner_cut_rule() -> None:
    # Passing exactly through a corner point obeys the neighbors() rule: both
    # shared orthogonal cells must be free.
    blocked = grid_from([".#", "#."])
    assert not blocked.is_motion_valid((0.5, 1.5), (1.5, 0.5))
    assert open_grid(2, 2).is_motion_valid((0.5, 0.5), (1.5, 1.5))


def test_motion_degenerate_and_gridline_segments() -> None:
    grid = grid_from(["..", ".#"])
    assert grid.is_motion_valid((0.5, 1.5), (0.5, 1.5))  # zero-length, free cell
    assert not grid.is_motion_valid((1.5, 0.5), (1.5, 0.5))  # zero-length, occupied cell
    # Vertical run lying exactly on a grid line (zero delta on one axis).
    assert open_grid(2, 2).is_motion_valid((1.0, 0.2), (1.0, 1.8))


def test_invalid_connectivity_rejected() -> None:
    with pytest.raises(ValueError):
        open_grid(3, 3, connectivity=6)
