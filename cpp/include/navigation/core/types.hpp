#pragma once

#include <cstddef>
#include <functional>
#include <vector>

namespace navigation::core {

// Discrete state: grid cell (row, col), integers. row 0 = top image row.
struct Cell {
  int row = 0;
  int col = 0;
  bool operator==(const Cell& o) const { return row == o.row && col == o.col; }
  // Row-major total order so a Cell can key an ordered std::set (the belief's
  // blocked-cell set in dynamic replanning). Deterministic across runs/languages.
  bool operator<(const Cell& o) const { return row < o.row || (row == o.row && col < o.col); }
};

// Sampling state: world point (x, y), meters.
struct Point {
  double x = 0.0;
  double y = 0.0;
};

// SE(2) planning state: world pose (x,y meters, theta radians). Distinct from
// Point (no heading) — never blur them. No operator</hash/==: the kinodynamic
// closed set keys on a planner-internal discretized bin, not on a raw float Pose.
struct Pose {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;  // radians, world frame; not normalized here
};

// Robot collision footprint — inscribed disc only (orientation-invariant, so
// collision depends only on (x,y): no per-vertex swept-polygon trig). A polygon
// variant can extend this later without changing the capability signature.
struct Footprint {
  double inscribed_radius = 0.0;  // meters
};

struct PlanStats {
  int expanded_nodes = 0;
  int samples = 0;
  int iterations = 0;
  int tree_size = 0;
};

template <class State>
struct PlanResult {
  bool success = false;
  std::vector<State> path;
  double cost = 0.0;
  PlanStats stats;
};

// A state serializes to trace as a numeric JSON array; these free functions
// give the coordinate tuple per state type (cell -> row,col; point -> x,y).
inline std::vector<double> to_trace(const Cell& c) {
  return {static_cast<double>(c.row), static_cast<double>(c.col)};
}
inline std::vector<double> to_trace(const Point& p) { return {p.x, p.y}; }
// A Pose serializes to [x, y, theta] (schema $defs/state already allows maxItems 3).
inline std::vector<double> to_trace(const Pose& p) { return {p.x, p.y, p.theta}; }

}  // namespace navigation::core

namespace std {
template <>
struct hash<navigation::core::Cell> {
  size_t operator()(const navigation::core::Cell& c) const noexcept {
    size_t h1 = std::hash<int>()(c.row);
    size_t h2 = std::hash<int>()(c.col);
    return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
  }
};
}  // namespace std
