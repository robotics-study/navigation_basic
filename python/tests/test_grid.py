"""OccupancyGrid2D geometry, occupancy, neighbors, capabilities."""

from __future__ import annotations

import math

import numpy as np
import pytest
from conftest import grid_from, open_grid

from navigation.core.capabilities import Capability
from navigation.maps.occupancy_grid import OccupancyGrid2D


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
    assert grid.capabilities() == {
        Capability.DISCRETE_SPACE,
        Capability.SAMPLING_SPACE,
        Capability.LINE_OF_SIGHT_SPACE,
        Capability.DYNAMIC_GRID_SPACE,
        Capability.SE2_COLLISION_SPACE,
        Capability.OBSTACLE_QUERY,
    }
    assert grid.supports(Capability.DISCRETE_SPACE)
    assert grid.supports(Capability.SAMPLING_SPACE)
    assert grid.supports(Capability.LINE_OF_SIGHT_SPACE)
    assert grid.supports(Capability.DYNAMIC_GRID_SPACE)
    assert grid.supports(Capability.SE2_COLLISION_SPACE)
    assert grid.supports(Capability.OBSTACLE_QUERY)


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


# --- ObstacleQuery: distance_to_nearest ------------------------------------
# 7x7 grid, single obstacle at (3,3) -> world center (3.5, 3.5). Reused across
# cases so the hand-computed geometry (obstacle vs. border source) stays comparable.
_OBSTACLE_ROWS = [
    ".......",
    ".......",
    ".......",
    "...#...",
    ".......",
    ".......",
    ".......",
]


def test_distance_to_nearest_obstacle_adjacent() -> None:
    grid = grid_from(_OBSTACLE_ROWS)
    # (3,2) is one cell left of the obstacle (distance 1) and far from every
    # border (distance 3), so the obstacle -- not the map edge -- is nearest.
    query = grid.cell_to_world(3, 2)
    assert grid.distance_to_nearest(query) == pytest.approx(1.0)


def test_distance_to_nearest_far_from_obstacle() -> None:
    # 11x11 grid, obstacle at (5,9) (not itself on the edge). Query at the exact
    # center (5,5): obstacle distance 4.0 < border distance 6.0 on every side.
    rows = ["." * 11 for _ in range(11)]
    rows[5] = "." * 9 + "#" + "."
    grid = grid_from(rows)
    query = grid.cell_to_world(5, 5)
    assert grid.distance_to_nearest(query) == pytest.approx(4.0)


def test_distance_to_nearest_near_boundary_beats_farther_obstacle() -> None:
    # (0,3) sits on the top edge: the map boundary (distance 1, treated as a non-free
    # source) is nearer than the real obstacle at (3,3) (distance 3), so the border
    # branch -- not the obstacle branch -- decides the answer.
    grid = grid_from(_OBSTACLE_ROWS)
    query = grid.cell_to_world(0, 3)
    assert grid.distance_to_nearest(query) == pytest.approx(1.0)


def test_distance_to_nearest_point_outside_grid_is_zero() -> None:
    # A point outside the grid is itself in a non-free (out-of-bounds) cell, so it
    # is its own nearest non-free cell.
    grid = open_grid(3, 3)
    assert grid.distance_to_nearest((-5.0, -5.0)) == 0.0


def test_distance_to_nearest_inside_occupied_cell_is_zero() -> None:
    grid = grid_from(_OBSTACLE_ROWS)
    query = grid.cell_to_world(3, 3)  # the occupied cell's own center
    assert grid.distance_to_nearest(query) == 0.0


# --- ObstacleQuery: occupied_within ------------------------------------------
def test_occupied_within_includes_out_of_bounds_and_respects_radius_and_order() -> None:
    grid = grid_from(["...", ".#.", "..."])
    center = grid.cell_to_world(0, 0)  # top-left free cell
    # Axis-aligned neighbors of (0,0) sit at distance 1.0 (within radius 1.2); the
    # diagonal neighbors (including the occupied (1,1)) sit at sqrt(2) (excluded).
    # Only the out-of-bounds axis-aligned neighbors are non-free -> row asc, col asc.
    result = grid.occupied_within(center, radius=1.2)
    assert result == [grid.cell_to_world(-1, 0), grid.cell_to_world(0, -1)]


def test_occupied_within_empty_when_nothing_in_radius() -> None:
    grid = open_grid(7, 7)  # fully free, no obstacles
    center = grid.cell_to_world(3, 3)  # center cell, 4 cells from any border
    assert grid.occupied_within(center, radius=0.5) == []
