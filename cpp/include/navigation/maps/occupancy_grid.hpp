#pragma once

#include <cmath>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include "navigation/core/capabilities.hpp"
#include "navigation/core/types.hpp"
#include "navigation/maps/pgm.hpp"

namespace navigation::maps {

using core::Capability;
using core::Cell;
using core::Footprint;
using core::Point;
using core::Pose;

// 2D occupancy grid exposing both DiscreteSpace (grid search) and SamplingSpace
// (RRT) over the same cells. World<->cell conversion lives only here. Geometry
// follows ROS map_server: origin is the world pose of the bottom-left pixel and
// row 0 is the top image row.
class OccupancyGrid2D final : public core::MapBase,
                              public core::LineOfSightSpace<Cell>,  // was DiscreteSpace<Cell>
                              public core::DynamicGridSpace<Cell>,
                              public core::SamplingSpace<Point>,
                              // Sibling add (State = Pose is a third state type, shares no base
                              // subobject with the Cell/Point bases) → no diamond, no virtual base.
                              public core::SE2CollisionSpace<Pose> {
 public:
  OccupancyGrid2D(int rows, int cols, double resolution, double origin_x, double origin_y,
                  std::vector<bool> free_cells, int connectivity = 8, unsigned seed = 0);

  // Applies occupancy thresholds: occ = 1 - pixel/255; a cell is traversable
  // only when occ <= free_thresh (occupied or unknown cells are blocked).
  static OccupancyGrid2D from_image(const PgmImage& img, double resolution, double origin_x,
                                    double origin_y, double occupied_thresh, double free_thresh,
                                    int connectivity, unsigned seed);

  std::set<Capability> capabilities() const override;

  Point cell_to_world(const Cell& c) const;
  Cell world_to_cell(double x, double y) const;

  std::vector<std::pair<Cell, double>> neighbors(const Cell& s) const override;
  double heuristic(const Cell& a, const Cell& b) const override;
  bool line_of_sight(const Cell& a, const Cell& b) const override;

  std::vector<std::pair<Cell, double>> passable_neighbors(
      const Cell& s, const std::set<Cell>& blocked) const override;
  bool is_blocked(const Cell& s) const override;

  bool is_collision(const Footprint& footprint, const Pose& pose) const override;

  Point sample() override;
  bool is_state_valid(const Point& p) const override;
  bool is_motion_valid(const Point& a, const Point& b) const override;
  double distance(const Point& a, const Point& b) const override;
  Point steer(const Point& a, const Point& b, double eta) const override;

  int rows() const { return rows_; }
  int cols() const { return cols_; }
  int connectivity() const { return connectivity_; }
  bool is_free(int row, int col) const;

 private:
  bool in_bounds(int row, int col) const;
  bool is_free_uv(int iu, int iv) const;

  // Single 8-move + corner-cut worker shared by neighbors() (truth predicate) and
  // passable_neighbors() (belief predicate). `is_free(row, col)` decides whether a
  // cell may be entered; the diagonal corner rule reuses the same predicate so both
  // callers forbid corner-cutting identically. Templated (not std::function) to keep
  // the hot search path allocation-free.
  template <class FreePred>
  std::vector<std::pair<Cell, double>> neighbors_impl(const Cell& s, FreePred is_free) const {
    // Orthogonals first, then diagonals — a fixed cross-language emission order so
    // the deterministic searches (with a stable tie-break) settle the same path in
    // both C++ and Python.
    static const int kOrthoR[] = {-1, 1, 0, 0};
    static const int kOrthoC[] = {0, 0, -1, 1};
    static const int kDiagR[] = {-1, -1, 1, 1};
    static const int kDiagC[] = {-1, 1, -1, 1};
    const double kSqrt2 = std::sqrt(2.0);

    std::vector<std::pair<Cell, double>> out;
    for (int i = 0; i < 4; ++i) {
      int nr = s.row + kOrthoR[i], nc = s.col + kOrthoC[i];
      if (is_free(nr, nc)) out.push_back({Cell{nr, nc}, 1.0});
    }
    if (connectivity_ == 4) return out;
    for (int i = 0; i < 4; ++i) {
      int dr = kDiagR[i], dc = kDiagC[i];
      int nr = s.row + dr, nc = s.col + dc;
      if (!is_free(nr, nc)) continue;
      // No corner-cutting: a diagonal is blocked if either shared orthogonal cell
      // is not free (the robot would clip an obstacle corner).
      if (!is_free(s.row + dr, s.col) || !is_free(s.row, s.col + dc)) continue;
      out.push_back({Cell{nr, nc}, kSqrt2});
    }
    return out;
  }

  int rows_;
  int cols_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  std::vector<bool> free_;  // rows_*cols_, row-major, row 0 = top
  int connectivity_;
  std::mt19937 rng_;
};

}  // namespace navigation::maps
