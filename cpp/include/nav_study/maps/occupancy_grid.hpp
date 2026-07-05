#pragma once

#include <random>
#include <set>
#include <utility>
#include <vector>

#include "nav_study/core/capabilities.hpp"
#include "nav_study/core/types.hpp"
#include "nav_study/maps/pgm.hpp"

namespace nav_study::maps {

using core::Capability;
using core::Cell;
using core::Point;

// 2D occupancy grid exposing both DiscreteSpace (grid search) and SamplingSpace
// (RRT) over the same cells. World<->cell conversion lives only here. Geometry
// follows ROS map_server: origin is the world pose of the bottom-left pixel and
// row 0 is the top image row.
class OccupancyGrid2D final : public core::MapBase,
                              public core::DiscreteSpace<Cell>,
                              public core::SamplingSpace<Point> {
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

  int rows_;
  int cols_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  std::vector<bool> free_;  // rows_*cols_, row-major, row 0 = top
  int connectivity_;
  std::mt19937 rng_;
};

}  // namespace nav_study::maps
