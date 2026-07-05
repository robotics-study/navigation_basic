#pragma once

#include <utility>
#include <vector>

#include "navigation/core/capabilities.hpp"
#include "navigation/core/trace.hpp"
#include "navigation/core/types.hpp"

namespace navigation::global_planning {

using core::Point;
using core::SamplingSpace;
using core::TraceRecorder;

// Undirected graph over world Points shared by the PRM family (PRM / PRM*). Both
// build a roadmap over sampled free states, differing only in their connection
// radius policy, then answer a start->goal query with Dijkstra over the graph.
struct Roadmap {
  std::vector<Point> nodes;
  std::vector<std::vector<std::pair<int, double>>> adj;

  int add_node(const Point& p);
  void add_edge(int a, int b, double cost);
};

// Wire `idx` to every earlier node within `radius` via a valid motion. Only
// earlier nodes are considered so each undirected edge is added once as the
// roadmap grows incrementally.
void connect(const SamplingSpace<Point>& space, Roadmap& roadmap, int idx, double radius,
             TraceRecorder* recorder);

// Shortest path start->goal over the roadmap. out_cost / out_expanded receive the
// path cost and the number of expanded nodes; returns an empty path on failure.
std::vector<Point> dijkstra(const Roadmap& roadmap, int start, int goal, TraceRecorder* recorder,
                            double& out_cost, int& out_expanded);

// Draw `num_samples` collision-free nodes into the roadmap, emitting sample_drawn.
void sample_free(SamplingSpace<Point>& space, Roadmap& roadmap, int num_samples,
                 TraceRecorder* recorder);

}  // namespace navigation::global_planning
