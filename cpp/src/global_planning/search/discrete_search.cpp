#include "navigation/global_planning/search/discrete_search.hpp"

#include <algorithm>
#include <map>

namespace navigation::global_planning {

std::vector<Cell> reconstruct_path(const std::unordered_map<Cell, Cell>& came_from,
                                   const Cell& start, const Cell& goal) {
  std::vector<Cell> path;
  Cell cur = goal;
  path.push_back(cur);
  while (!(cur == start)) {
    auto it = came_from.find(cur);
    if (it == came_from.end()) return {};  // goal not connected to start
    cur = it->second;
    path.push_back(cur);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

double path_cost(const DiscreteSpace<Cell>& space, const std::vector<Cell>& path) {
  double total = 0.0;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    for (const auto& [v, w] : space.neighbors(path[i])) {
      if (v == path[i + 1]) {
        total += w;
        break;
      }
    }
  }
  return total;
}

void emit_finished_discrete(TraceRecorder* recorder, bool success, double cost,
                            const PlanStats& stats, double runtime_sec) {
  if (!recorder) return;
  std::map<std::string, double> metrics{{"runtime_sec", runtime_sec},
                                        {"path_cost", success ? cost : 0.0},
                                        {"expanded_nodes", static_cast<double>(stats.expanded_nodes)}};
  recorder->planning_finished(success, metrics);
}

}  // namespace navigation::global_planning
