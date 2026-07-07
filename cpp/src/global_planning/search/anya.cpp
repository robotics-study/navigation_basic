#include "navigation/global_planning/search/anya.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "navigation/global_planning/search/discrete_search.hpp"

namespace navigation::global_planning {

using core::LineOfSightSpace;

namespace {

// Euclidean distance on integer cell-index deltas (row, col). sqrt — NOT hypot:
// hypot is relaxed-accuracy and can differ 1 ULP between C++/Python, which would
// diverge f-values and the emitted trace the bench compares across languages.
// sqrt is correctly-rounded, and sqrt(2.0) exactly equals the diagonal grid cost,
// matching Theta*'s any-angle cost model.
double euclid(const Cell& a, const Cell& b) {
  double dr = static_cast<double>(a.row - b.row);
  double dc = static_cast<double>(a.col - b.col);
  return std::sqrt(dr * dr + dc * dc);
}

}  // namespace

core::PlanResult<Cell> AnyaPlanner::plan(LineOfSightSpace<Cell>& space, const Cell& start,
                                         const Cell& goal, TraceRecorder* recorder) {
  // f = g + w*h, h Euclidean (straight-line). w == 1 is optimal Anya (Harabor,
  // Grastien, Öz & Aksakalli 2016); w > 1 trades optimality for speed (Pohl 1970).
  const double w = params_.get_float("heuristic_weight");
  auto h = [&](const Cell& c) { return w * euclid(c, goal); };

  struct QItem {
    double f;
    unsigned long long seq;  // insertion order; FIFO tie-break keeps ties stable
    Cell cell;
  };
  struct Greater {
    bool operator()(const QItem& a, const QItem& b) const {
      // Same (f, seq) tie-break as A*/Theta* so both languages settle one path.
      return a.f > b.f || (a.f == b.f && a.seq > b.seq);
    }
  };

  auto t0 = std::chrono::steady_clock::now();

  // Candidate vertex set = the start's connected free component, discovered
  // through the capability's neighbors() alone (no concrete-map access). Any
  // feasible path stays inside it, so restricting roots / interval members here
  // preserves optimality; a goal outside it is genuinely unreachable. Bucketed by
  // row with sorted columns — the per-row layout the interval projection scans.
  std::unordered_set<Cell> reachable{start};
  std::vector<Cell> stack{start};
  while (!stack.empty()) {
    Cell cur = stack.back();
    stack.pop_back();
    for (const auto& [nb, cost] : space.neighbors(cur)) {
      (void)cost;
      if (reachable.insert(nb).second) stack.push_back(nb);
    }
  }
  std::map<int, std::vector<int>> by_row;
  for (const Cell& c : reachable) by_row[c.row].push_back(c.col);
  for (auto& [row, cols] : by_row) std::sort(cols.begin(), cols.end());

  std::priority_queue<QItem, std::vector<QItem>, Greater> open;
  std::unordered_map<Cell, double> g;
  std::unordered_map<Cell, Cell> parent;
  std::unordered_map<Cell, bool> settled;
  unsigned long long seq = 0;

  g[start] = 0.0;
  parent[start] = start;  // self-parent: reconstruct terminates here
  open.push({h(start), seq++, start});

  PlanResult<Cell> result;
  PlanStats stats;
  bool found = false;
  const bool goal_reachable = reachable.count(goal) > 0;

  while (goal_reachable && !open.empty()) {
    Cell root = open.top().cell;
    open.pop();
    if (settled[root]) continue;  // stale duplicate with a worse key
    settled[root] = true;
    if (recorder) recorder->node_expanded(root, g[root]);
    ++stats.expanded_nodes;
    if (root == goal) {
      found = true;
      break;
    }
    const double g_root = g[root];
    // Project the root's visibility into per-row successor intervals and relax
    // every cell they contain (Harabor et al. 2016). An interval is a maximal run
    // of row-adjacent free cells all LOS-visible from the root; relaxing the whole
    // visible set makes the search the cell-centre visibility-graph optimum.
    for (const auto& [row, cols] : by_row) {
      const int n = static_cast<int>(cols.size());
      int i = 0;
      while (i < n) {
        if (!space.line_of_sight(root, Cell{row, cols[i]})) {
          ++i;
          continue;
        }
        int j = i;  // extend while columns stay contiguous and visible
        while (j + 1 < n && cols[j + 1] == cols[j] + 1 &&
               space.line_of_sight(root, Cell{row, cols[j + 1]})) {
          ++j;
        }
        for (int col = cols[i]; col <= cols[j]; ++col) {
          Cell cell{row, col};
          if (cell == root || settled[cell]) continue;
          double ecost = euclid(root, cell);
          double cand = g_root + ecost;
          auto it = g.find(cell);
          if (it == g.end() || cand < it->second) {
            g[cell] = cand;
            parent[cell] = root;
            if (recorder) recorder->candidate_evaluated(cell, cand);
            if (recorder) recorder->edge_added(cell, root, ecost);  // any-angle edge
            open.push({cand + h(cell), seq++, cell});
          }
        }
        i = j + 1;
      }
    }
  }

  if (found) {
    result.success = true;
    result.path = reconstruct_path(parent, start, goal);
    result.cost = g[goal];  // any-angle polyline length; NOT path_cost() (adjacent-edge sum).
    if (recorder) recorder->path_found(result.path);
  }
  result.stats = stats;
  double rt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  emit_finished_discrete(recorder, result.success, result.cost, stats, rt);
  return result;
}

}  // namespace navigation::global_planning
