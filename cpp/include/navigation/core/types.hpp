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
};

// Sampling state: world point (x, y), meters.
struct Point {
  double x = 0.0;
  double y = 0.0;
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
