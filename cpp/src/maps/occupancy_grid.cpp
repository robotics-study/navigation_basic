#include "navigation/maps/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace navigation::maps {
namespace {

// Sentinel for "no source seen yet" in the 1D transform below. Squared cell
// distances on any map we load are tiny by comparison, so this never collides
// with a real value.
constexpr double kEdtInf = 1e18;

// Exact 1D squared-distance transform, in place: f[i] = 0 marks a source, any
// other value is replaced by the squared distance (in index units) to the
// nearest source. Lower-envelope-of-parabolas algorithm (Felzenszwalb &
// Huttenlocher 2004, "Distance Transforms of Sampled Functions") — O(n), exact
// (integer-valued results, only the caller's final sqrt is inexact).
void edt_1d(std::vector<double>& f) {
  const int n = static_cast<int>(f.size());
  std::vector<int> v(n, 0);
  std::vector<double> z(static_cast<size_t>(n) + 1);
  std::vector<double> d(n);
  int k = 0;
  v[0] = 0;
  z[0] = -kEdtInf;
  z[1] = kEdtInf;
  for (int q = 1; q < n; ++q) {
    double s = ((f[q] + static_cast<double>(q) * q) -
                (f[v[k]] + static_cast<double>(v[k]) * v[k])) /
               (2.0 * (q - v[k]));
    while (s <= z[k]) {
      --k;
      s = ((f[q] + static_cast<double>(q) * q) -
           (f[v[k]] + static_cast<double>(v[k]) * v[k])) /
          (2.0 * (q - v[k]));
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kEdtInf;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) ++k;
    double diff = q - v[static_cast<size_t>(k)];
    d[q] = diff * diff + f[static_cast<size_t>(v[k])];
  }
  f = std::move(d);
}

// Squared-distance-to-nearest-source grid, in cell units, over a (rows+2) x
// (cols+2) padded grid: the outer 1-cell ring is entirely source (models "out
// of bounds" as immediately adjacent, matching is_collision's occupied-OR-out-
// of-bounds rule); interior cells are source iff occupied. Row-major, size
// (rows+2)*(cols+2). Two 1D passes (columns then rows) give the exact 2D
// Euclidean squared-distance transform.
std::vector<double> compute_edt(int rows, int cols, const std::vector<bool>& free) {
  const int pr = rows + 2, pc = cols + 2;
  std::vector<double> g(static_cast<size_t>(pr) * pc);
  for (int i = 0; i < pr; ++i) {
    for (int j = 0; j < pc; ++j) {
      bool source = i == 0 || i == pr - 1 || j == 0 || j == pc - 1 ||
                    !free[static_cast<size_t>(i - 1) * cols + (j - 1)];
      g[static_cast<size_t>(i) * pc + j] = source ? 0.0 : kEdtInf;
    }
  }
  std::vector<double> col(pr);
  for (int j = 0; j < pc; ++j) {
    for (int i = 0; i < pr; ++i) col[i] = g[static_cast<size_t>(i) * pc + j];
    edt_1d(col);
    for (int i = 0; i < pr; ++i) g[static_cast<size_t>(i) * pc + j] = col[i];
  }
  std::vector<double> row(pc);
  for (int i = 0; i < pr; ++i) {
    for (int j = 0; j < pc; ++j) row[j] = g[static_cast<size_t>(i) * pc + j];
    edt_1d(row);
    for (int j = 0; j < pc; ++j) g[static_cast<size_t>(i) * pc + j] = row[j];
  }
  return g;
}

}  // namespace

OccupancyGrid2D::OccupancyGrid2D(int rows, int cols, double resolution, double origin_x,
                                 double origin_y, std::vector<bool> free_cells, int connectivity,
                                 unsigned seed)
    : rows_(rows),
      cols_(cols),
      resolution_(resolution),
      origin_x_(origin_x),
      origin_y_(origin_y),
      free_(std::move(free_cells)),
      connectivity_(connectivity),
      rng_(seed) {
  if (rows_ <= 0 || cols_ <= 0) throw std::runtime_error("grid: non-positive size");
  if (free_.size() != static_cast<size_t>(rows_) * static_cast<size_t>(cols_)) {
    throw std::runtime_error("grid: free-cell count mismatch");
  }
  if (connectivity_ != 4 && connectivity_ != 8) {
    throw std::runtime_error("grid: connectivity must be 4 or 8");
  }
}

OccupancyGrid2D OccupancyGrid2D::from_image(const PgmImage& img, double resolution, double origin_x,
                                            double origin_y, double occupied_thresh,
                                            double free_thresh, int connectivity, unsigned seed) {
  (void)occupied_thresh;  // free_thresh alone decides traversability; occupied/unknown are blocked.
  std::vector<bool> free_cells(img.pixels.size());
  for (size_t i = 0; i < img.pixels.size(); ++i) {
    double occ = 1.0 - static_cast<double>(img.pixels[i]) / 255.0;
    free_cells[i] = occ <= free_thresh;
  }
  return OccupancyGrid2D(img.height, img.width, resolution, origin_x, origin_y,
                         std::move(free_cells), connectivity, seed);
}

std::set<Capability> OccupancyGrid2D::capabilities() const {
  return {Capability::DISCRETE_SPACE,      Capability::SAMPLING_SPACE,
          Capability::LINE_OF_SIGHT_SPACE, Capability::DYNAMIC_GRID_SPACE,
          Capability::SE2_COLLISION_SPACE, Capability::OBSTACLE_QUERY};
}

bool OccupancyGrid2D::in_bounds(int row, int col) const {
  return row >= 0 && row < rows_ && col >= 0 && col < cols_;
}

bool OccupancyGrid2D::is_free(int row, int col) const {
  return in_bounds(row, col) && free_[static_cast<size_t>(row) * cols_ + col];
}

Point OccupancyGrid2D::cell_to_world(const Cell& c) const {
  double x = origin_x_ + (c.col + 0.5) * resolution_;
  double y = origin_y_ + ((rows_ - 1 - c.row) + 0.5) * resolution_;
  return {x, y};
}

Cell OccupancyGrid2D::world_to_cell(double x, double y) const {
  int col = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  int row = (rows_ - 1) - static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return {row, col};
}

std::vector<std::pair<Cell, double>> OccupancyGrid2D::neighbors(const Cell& s) const {
  // Ground-truth successors: a cell is enterable iff in bounds and actually free.
  return neighbors_impl(s, [this](int row, int col) { return is_free(row, col); });
}

std::vector<std::pair<Cell, double>> OccupancyGrid2D::passable_neighbors(
    const Cell& s, const std::set<Cell>& blocked) const {
  // Belief successors: a cell is enterable iff in bounds and not (yet) known blocked;
  // real occupancy is invisible here (only is_blocked reads it). Same worker + corner
  // rule as neighbors(), so a discovered-free grid gives identical successors to truth.
  return neighbors_impl(s, [this, &blocked](int row, int col) {
    return in_bounds(row, col) && blocked.find(Cell{row, col}) == blocked.end();
  });
}

bool OccupancyGrid2D::is_blocked(const Cell& s) const {
  // Occupied OR out of bounds — is_free is already false for both.
  return !is_free(s.row, s.col);
}

double OccupancyGrid2D::heuristic(const Cell& a, const Cell& b) const {
  int dr = std::abs(a.row - b.row);
  int dc = std::abs(a.col - b.col);
  if (connectivity_ == 4) return dr + dc;  // Manhattan is admissible for 4-connected moves.
  // Octile distance: admissible lower bound for 8-connected moves with diagonal
  // cost sqrt(2) (D = 1, D2 = sqrt(2)).
  int lo = std::min(dr, dc);
  int hi = std::max(dr, dc);
  return (hi - lo) + std::sqrt(2.0) * lo;
}

bool OccupancyGrid2D::line_of_sight(const Cell& a, const Cell& b) const {
  // Any-angle LOS must mean "the straight segment is actually traversable" under
  // the SAME corner-cut-forbidden rule as neighbors()/is_motion_valid — else a
  // shortcut could clip an obstacle corner the grid edges forbid. Reuse the
  // verified supercover (Amanatides & Woo 1987) from cell centres; world
  // conversion stays inside the map (Nash, Daniel, Koenig & Felner 2007).
  return is_motion_valid(cell_to_world(a), cell_to_world(b));
}

Point OccupancyGrid2D::sample() {
  std::uniform_real_distribution<double> dx(origin_x_, origin_x_ + cols_ * resolution_);
  std::uniform_real_distribution<double> dy(origin_y_, origin_y_ + rows_ * resolution_);
  return {dx(rng_), dy(rng_)};
}

bool OccupancyGrid2D::is_state_valid(const Point& p) const {
  Cell c = world_to_cell(p.x, p.y);
  return is_free(c.row, c.col);
}

bool OccupancyGrid2D::is_collision(const Footprint& fp, const Pose& pose) const {
  // Inscribed-disc footprint is orientation-invariant, so pose.theta is unused (a
  // polygon footprint would use it) — Dolgov et al. 2008. Collision iff any occupied
  // or out-of-bounds cell overlaps the disc of radius r at (x, y). Exact disc–cell
  // overlap via squared distance to the cell rectangle (no sqrt, no trig → bit-
  // identical across languages). Fixed row-major scan; world<->cell stays here.
  const double r = fp.inscribed_radius, r2 = r * r, half = resolution_ * 0.5;
  const Cell lo = world_to_cell(pose.x - r, pose.y + r);  // y+r → smaller row
  const Cell hi = world_to_cell(pose.x + r, pose.y - r);
  for (int row = lo.row; row <= hi.row; ++row) {
    for (int col = lo.col; col <= hi.col; ++col) {
      if (is_free(row, col)) continue;               // in-bounds & free → skip
      const Point c = cell_to_world(Cell{row, col});  // occupied OR out-of-bounds
      const double dx = pose.x - std::clamp(pose.x, c.x - half, c.x + half);
      const double dy = pose.y - std::clamp(pose.y, c.y - half, c.y + half);
      if (dx * dx + dy * dy <= r2) return true;
    }
  }
  return false;
}

double OccupancyGrid2D::distance_to_nearest(const Point& p) const {
  if (!edt_) edt_ = compute_edt(rows_, cols_, free_);
  const Cell c = world_to_cell(p.x, p.y);
  // Padded index = grid index + 1 (the added border ring). is_free() already
  // treats every out-of-bounds cell as non-free regardless of how far out it
  // is, so a cell beyond the ring is itself a source — clamping into the ring
  // yields exactly that (distance 0), while also keeping the lookup in bounds.
  const int pr = std::clamp(c.row + 1, 0, rows_ + 1);
  const int pc = std::clamp(c.col + 1, 0, cols_ + 1);
  const double d2 = (*edt_)[static_cast<size_t>(pr) * (cols_ + 2) + pc];
  return std::sqrt(d2) * resolution_;
}

std::vector<Point> OccupancyGrid2D::occupied_within(const Point& center, double radius) const {
  // Same bounding-box-then-filter shape as is_collision: convert the world
  // radius to a cell-index box via world_to_cell, then keep non-free cells
  // whose center actually falls within radius. Row ascending, then column
  // ascending (outer/inner loop order) — enumeration order is part of the
  // capability contract (core/capabilities.hpp), not an implementation detail.
  const double r2 = radius * radius;
  const Cell lo = world_to_cell(center.x - radius, center.y + radius);  // y+r -> smaller row
  const Cell hi = world_to_cell(center.x + radius, center.y - radius);
  std::vector<Point> out;
  for (int row = lo.row; row <= hi.row; ++row) {
    for (int col = lo.col; col <= hi.col; ++col) {
      if (is_free(row, col)) continue;                 // in-bounds & free -> skip
      const Point c = cell_to_world(Cell{row, col});    // occupied OR out-of-bounds
      const double dx = c.x - center.x, dy = c.y - center.y;
      if (dx * dx + dy * dy <= r2) out.push_back(c);
    }
  }
  return out;
}

bool OccupancyGrid2D::is_free_uv(int iu, int iv) const {
  // (u, v) grid coords count up from the origin (bottom-left); rows count down
  // from the top image row.
  return is_free(rows_ - 1 - iv, iu);
}

bool OccupancyGrid2D::is_motion_valid(const Point& a, const Point& b) const {
  // Supercover grid traversal (Amanatides & Woo, 1987): visits every cell the
  // segment crosses. Point sampling misses a corner clip whose in-cell chord is
  // shorter than the sample spacing, letting edges cut obstacle corners.
  const double kInf = std::numeric_limits<double>::infinity();
  double u0 = (a.x - origin_x_) / resolution_, v0 = (a.y - origin_y_) / resolution_;
  double u1 = (b.x - origin_x_) / resolution_, v1 = (b.y - origin_y_) / resolution_;
  int iu = static_cast<int>(std::floor(u0)), iv = static_cast<int>(std::floor(v0));
  int ju = static_cast<int>(std::floor(u1)), jv = static_cast<int>(std::floor(v1));
  if (!is_free_uv(iu, iv)) return false;
  double du = u1 - u0, dv = v1 - v0;
  int step_u = du > 0.0 ? 1 : -1;
  int step_v = dv > 0.0 ? 1 : -1;
  double t_delta_u = du != 0.0 ? std::abs(1.0 / du) : kInf;
  double t_delta_v = dv != 0.0 ? std::abs(1.0 / dv) : kInf;
  // Parametric distance (in units of t) from the start to the first grid line
  // crossed on each axis; an axis with no motion never crosses one (explicit
  // inf also avoids 0 * inf = nan when the start sits on a line).
  double t_max_u =
      du != 0.0 ? (du > 0.0 ? std::floor(u0) + 1.0 - u0 : u0 - std::floor(u0)) * t_delta_u : kInf;
  double t_max_v =
      dv != 0.0 ? (dv > 0.0 ? std::floor(v0) + 1.0 - v0 : v0 - std::floor(v0)) * t_delta_v : kInf;
  while (iu != ju || iv != jv) {
    // Clamp exhausted axes so float drift in t_max can never step past the end
    // cell (termination is by cell index, not by t).
    if (iv == jv || t_max_u < t_max_v) {
      iu += step_u;
      t_max_u += t_delta_u;
    } else if (iu == ju || t_max_v < t_max_u) {
      iv += step_v;
      t_max_v += t_delta_v;
    } else {
      // Exact corner crossing: same rule as neighbors() — passing through a
      // corner needs both shared orthogonal cells free.
      if (!is_free_uv(iu + step_u, iv) || !is_free_uv(iu, iv + step_v)) return false;
      iu += step_u;
      iv += step_v;
      t_max_u += t_delta_u;
      t_max_v += t_delta_v;
    }
    if (!is_free_uv(iu, iv)) return false;
  }
  return true;
}

double OccupancyGrid2D::distance(const Point& a, const Point& b) const {
  return std::hypot(a.x - b.x, a.y - b.y);
}

Point OccupancyGrid2D::steer(const Point& a, const Point& b, double eta) const {
  double d = distance(a, b);
  if (d <= eta || d == 0.0) return b;
  double s = eta / d;
  return {a.x + s * (b.x - a.x), a.y + s * (b.y - a.y)};
}

}  // namespace navigation::maps
