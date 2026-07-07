#include "navigation/global_planning/search/ara_star.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "navigation/global_planning/search/discrete_search.hpp"

namespace navigation::global_planning {

namespace {

struct QItem {
  double key;
  unsigned long long seq;  // insertion order; FIFO tie-break keeps ties stable
  Cell cell;
};
struct Greater {
  bool operator()(const QItem& a, const QItem& b) const {
    // Ties on key resolve by earliest insertion, matching the Python (key, counter)
    // ordering so both languages settle the same path.
    return a.key > b.key || (a.key == b.key && a.seq > b.seq);
  }
};
using MinHeap = std::priority_queue<QItem, std::vector<QItem>, Greater>;

}  // namespace

core::PlanResult<Cell> AraStar::plan(DiscreteSpace<Cell>& space, const Cell& start, const Cell& goal,
                                     TraceRecorder* recorder) {
  const double INF = std::numeric_limits<double>::infinity();
  // eps0 must not fall below the target eps, else no anytime repair happens.
  double eps = std::max(params_.get_float("eps_start"), params_.get_float("eps_final"));
  const double eps_final = params_.get_float("eps_final");
  const double eps_step = params_.get_float("eps_step");
  const long long max_expansions = params_.get_int("max_expansions");

  auto t0 = std::chrono::steady_clock::now();
  std::unordered_map<Cell, double> g;
  std::unordered_map<Cell, Cell> came_from;
  std::unordered_set<Cell> closed;
  std::unordered_set<Cell> incons;
  std::unordered_set<Cell> open_set;
  unsigned long long seq = 0;
  MinHeap open;

  g[start] = 0.0;
  open_set.insert(start);
  open.push({eps * space.heuristic(start, goal), seq++, start});
  long long expanded = 0;

  auto gval = [&](const Cell& s) {
    auto it = g.find(s);
    return it == g.end() ? INF : it->second;
  };

  // Expand until the goal's key is no larger than OPEN's minimum key: at that point
  // g(goal) is provably within cur_eps of optimal (the ARA* termination criterion).
  // Lazy heap — stale entries (state no longer in OPEN) are skipped on pop.
  auto improve_path = [&](double cur_eps) {
    while (!open.empty()) {
      while (!open.empty() && open_set.find(open.top().cell) == open_set.end()) open.pop();
      if (open.empty()) break;
      if (gval(goal) <= open.top().key) break;
      Cell s = open.top().cell;
      open.pop();
      open_set.erase(s);
      closed.insert(s);
      ++expanded;
      if (expanded > max_expansions) return;
      if (recorder) recorder->node_expanded(s, g[s]);
      for (const auto& [nb, edge_cost] : space.neighbors(s)) {
        double tentative = g[s] + edge_cost;
        auto it = g.find(nb);
        if (it == g.end() || tentative < it->second) {
          g[nb] = tentative;
          came_from[nb] = s;
          if (recorder) {
            recorder->candidate_evaluated(nb, tentative);
            recorder->edge_added(nb, s, edge_cost);
          }
          if (closed.find(nb) == closed.end()) {
            open_set.insert(nb);
            open.push({tentative + cur_eps * space.heuristic(nb, goal), seq++, nb});
          } else {
            // Already expanded but improved: defer to the next, smaller-eps iteration
            // instead of re-expanding now (the ARA* core trick).
            incons.insert(nb);
          }
        }
      }
    }
  };

  std::vector<Cell> best_path;
  double best_cost = INF;
  bool success = false;
  while (true) {
    improve_path(eps);
    if (gval(goal) < INF) {
      best_path = reconstruct_path(came_from, start, goal);
      best_cost = path_cost(space, best_path);
      success = true;
      if (recorder) recorder->path_found(best_path);  // anytime: bound = eps
    } else {
      break;  // OPEN exhausted without reaching the goal
    }
    if (eps <= eps_final || expanded >= max_expansions) break;
    eps = std::max(eps_final, eps - eps_step);
    // Reopen INCONS union OPEN with keys recomputed under the tightened eps and clear
    // CLOSED so improved states can be re-expanded.
    for (const Cell& s : incons) open_set.insert(s);
    incons.clear();
    closed.clear();
    MinHeap rebuilt;
    for (const Cell& s : open_set) rebuilt.push({g[s] + eps * space.heuristic(s, goal), seq++, s});
    open = std::move(rebuilt);
  }

  PlanResult<Cell> result;
  PlanStats stats;
  stats.expanded_nodes = static_cast<int>(expanded);
  result.stats = stats;
  if (success) {
    result.success = true;
    result.path = best_path;
    result.cost = best_cost;
  }
  double rt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  emit_finished_discrete(recorder, success, best_cost, stats, rt);
  return result;
}

}  // namespace navigation::global_planning
