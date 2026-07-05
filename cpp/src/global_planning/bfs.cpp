#include "nav_study/global_planning/bfs.hpp"

#include <chrono>
#include <deque>
#include <unordered_map>
#include <unordered_set>

#include "nav_study/global_planning/discrete_search.hpp"

namespace nav_study::global_planning {

core::PlanResult<Cell> BfsPlanner::plan(DiscreteSpace<Cell>& space, const Cell& start,
                                        const Cell& goal, TraceRecorder* recorder) {
  auto t0 = std::chrono::steady_clock::now();
  std::deque<Cell> queue{start};
  std::unordered_set<Cell> discovered{start};
  std::unordered_map<Cell, Cell> came_from;
  std::unordered_map<Cell, double> cost_to{{start, 0.0}};
  PlanStats stats;
  bool found = false;

  while (!queue.empty()) {
    Cell u = queue.front();
    queue.pop_front();
    if (recorder) recorder->node_expanded(u, cost_to[u]);
    ++stats.expanded_nodes;
    if (u == goal) {
      found = true;
      break;
    }
    for (const auto& [v, w] : space.neighbors(u)) {
      if (discovered.count(v)) continue;  // first discovery gives the shortest-in-moves parent
      discovered.insert(v);
      came_from[v] = u;
      cost_to[v] = cost_to[u] + w;
      if (recorder) recorder->edge_added(v, u, w);
      queue.push_back(v);
    }
  }

  core::PlanResult<Cell> result;
  if (found) {
    result.success = true;
    result.path = reconstruct_path(came_from, start, goal);
    result.cost = cost_to[goal];  // true edge-cost sum along the fewest-edge path
    if (recorder) recorder->path_found(result.path);
  }
  result.stats = stats;
  double rt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  emit_finished_discrete(recorder, result.success, result.cost, stats, rt);
  return result;
}

}  // namespace nav_study::global_planning
