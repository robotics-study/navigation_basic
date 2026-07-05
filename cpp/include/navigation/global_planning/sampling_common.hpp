#pragma once

#include <map>
#include <vector>

#include "navigation/core/capabilities.hpp"
#include "navigation/core/trace.hpp"
#include "navigation/core/types.hpp"

namespace navigation::global_planning {

using core::Point;
using core::SamplingSpace;
using core::TraceRecorder;

// Incremental search tree shared by the RRT family. Each node keeps its parent
// index and cumulative cost from the root; children lists let rewiring propagate
// cost updates to a rewired node's subtree.
struct Tree {
  std::vector<Point> nodes;
  std::vector<int> parent;
  std::vector<double> cost;
  std::vector<std::vector<int>> children;

  int add(const Point& p, int parent_idx, double cumulative_cost);
  // Re-attaches `child` under `new_parent` with the given cost and propagates the
  // new cost to all descendants.
  void reparent(int child, int new_parent, double new_cost, const SamplingSpace<Point>& space);
};

int nearest(const Tree& tree, const SamplingSpace<Point>& space, const Point& q);
std::vector<int> near(const Tree& tree, const SamplingSpace<Point>& space, const Point& q,
                      double radius);
std::vector<Point> extract_path(const Tree& tree, int goal_idx);

// Ground-truth geometric length of a polyline. Recomputed from the returned path
// so rewiring that changes an ancestor cannot leave a stale cumulative cost.
double path_length(const SamplingSpace<Point>& space, const std::vector<Point>& path);

// RRT* choose-parent: among `nbrs` (plus the nearest fallback), pick the
// collision-free parent minimising cumulative cost to q_new; out_cost receives
// that cost. Emits candidate_evaluated per considered neighbor.
int choose_parent(const Tree& tree, const SamplingSpace<Point>& space, const Point& q_new,
                  const std::vector<int>& nbrs, int nearest_idx, double& out_cost,
                  TraceRecorder* recorder);

// RRT* rewire: for each neighbor whose cost improves by routing through new_idx
// (with a valid motion), reparent it and emit rewire.
void rewire_neighbors(Tree& tree, const SamplingSpace<Point>& space, int new_idx,
                      const std::vector<int>& nbrs, TraceRecorder* recorder);

// Sampling metrics for planning_finished.
void emit_finished_sampling(TraceRecorder* recorder, bool success, double cost, int expanded_nodes,
                            int samples, int tree_size, int iterations, double runtime_sec);

}  // namespace navigation::global_planning
