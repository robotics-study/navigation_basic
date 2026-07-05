#pragma once

#include <chrono>
#include <queue>
#include <unordered_map>
#include <vector>

#include "navigation/core/capabilities.hpp"
#include "navigation/core/trace.hpp"
#include "navigation/core/types.hpp"

namespace navigation::global_planning {

using core::Cell;
using core::DiscreteSpace;
using core::PlanResult;
using core::PlanStats;
using core::TraceRecorder;

// Follows the came-from links from goal back to start (inclusive), returning the
// path in start->goal order.
std::vector<Cell> reconstruct_path(const std::unordered_map<Cell, Cell>& came_from,
                                   const Cell& start, const Cell& goal);

// True edge cost of a found path, summed from the space's own neighbor costs so
// diagonal (sqrt2) vs orthogonal costs stay consistent with the map.
double path_cost(const DiscreteSpace<Cell>& space, const std::vector<Cell>& path);

// Discrete metrics for planning_finished: runtime_sec, path_cost, expanded_nodes.
void emit_finished_discrete(TraceRecorder* recorder, bool success, double cost,
                            const PlanStats& stats, double runtime_sec);

// Shared best-first search for Dijkstra (h == 0) and A* (h == weighted
// heuristic). heuristic_to_goal(cell) already includes any weight. Nodes settle
// on pop (node_expanded); neighbor relaxations emit candidate_evaluated +
// edge_added. Dijkstra 1959; A* Hart-Nilsson-Raphael 1968.
template <class Heuristic>
PlanResult<Cell> best_first_search(DiscreteSpace<Cell>& space, const Cell& start, const Cell& goal,
                                   TraceRecorder* recorder, Heuristic heuristic_to_goal) {
  struct QItem {
    double f;
    unsigned long long seq;  // insertion order; FIFO tie-break keeps ties stable
    Cell cell;
  };
  struct Greater {
    bool operator()(const QItem& a, const QItem& b) const {
      // Ties on f resolve by earliest insertion (smaller seq pops first), matching
      // the Python (f, counter) ordering so both languages settle the same path.
      return a.f > b.f || (a.f == b.f && a.seq > b.seq);
    }
  };

  auto t0 = std::chrono::steady_clock::now();
  std::priority_queue<QItem, std::vector<QItem>, Greater> open;
  std::unordered_map<Cell, double> g;
  std::unordered_map<Cell, Cell> came_from;
  std::unordered_map<Cell, bool> settled;
  unsigned long long seq = 0;

  g[start] = 0.0;
  open.push({heuristic_to_goal(start), seq++, start});

  PlanResult<Cell> result;
  PlanStats stats;
  bool found = false;

  while (!open.empty()) {
    Cell u = open.top().cell;
    open.pop();
    if (settled[u]) continue;  // stale duplicate with a worse key
    settled[u] = true;
    if (recorder) recorder->node_expanded(u, g[u]);
    ++stats.expanded_nodes;
    if (u == goal) {
      found = true;
      break;
    }
    for (const auto& [v, w] : space.neighbors(u)) {
      if (settled[v]) continue;
      double ng = g[u] + w;
      auto it = g.find(v);
      if (it == g.end() || ng < it->second) {
        g[v] = ng;
        came_from[v] = u;
        if (recorder) recorder->candidate_evaluated(v, ng);
        if (recorder) recorder->edge_added(v, u, w);
        open.push({ng + heuristic_to_goal(v), seq++, v});
      }
    }
  }

  if (found) {
    result.success = true;
    result.path = reconstruct_path(came_from, start, goal);
    result.cost = path_cost(space, result.path);
    if (recorder) recorder->path_found(result.path);
  }
  result.stats = stats;
  double rt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  emit_finished_discrete(recorder, result.success, result.cost, stats, rt);
  return result;
}

}  // namespace navigation::global_planning
