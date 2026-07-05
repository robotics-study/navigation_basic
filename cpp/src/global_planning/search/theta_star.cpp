#include "navigation/global_planning/search/theta_star.hpp"

#include <chrono>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>

#include "navigation/global_planning/search/discrete_search.hpp"

namespace navigation::global_planning {

using core::LineOfSightSpace;

namespace {

// Euclidean distance on integer cell-index deltas (row, col). sqrt — NOT hypot:
// hypot is relaxed-accuracy and can differ 1 ULP between C++/Python, which would
// diverge f-values and the emitted trace stream the bench compares across
// languages. sqrt is correctly-rounded, and sqrt(2.0) exactly equals the
// diagonal edge cost from neighbors(), so a diagonal shortcut (path 2) and a
// diagonal grid step (path 1) are bit-equal.
double euclid(const Cell& a, const Cell& b) {
  double dr = static_cast<double>(a.row - b.row);
  double dc = static_cast<double>(a.col - b.col);
  return std::sqrt(dr * dr + dc * dc);
}

}  // namespace

core::PlanResult<Cell> ThetaStarPlanner::plan(LineOfSightSpace<Cell>& space, const Cell& start,
                                              const Cell& goal, TraceRecorder* recorder) {
  // Weighted Theta*: f = g + w*h, h Euclidean (straight-line). w == 1 is standard
  // Theta*; the octile heuristic (space.heuristic) would be inadmissible for
  // any-angle costs since octile >= Euclidean (Nash, Daniel, Koenig & Felner 2007;
  // Pohl 1970).
  const double w = params_.get_float("heuristic_weight");
  auto h = [&](const Cell& c) { return w * euclid(c, goal); };

  struct QItem {
    double f;
    unsigned long long seq;  // insertion order; FIFO tie-break keeps ties stable
    Cell cell;
  };
  struct Greater {
    bool operator()(const QItem& a, const QItem& b) const {
      // Same (f, seq) tie-break as A* so both languages settle the same path.
      return a.f > b.f || (a.f == b.f && a.seq > b.seq);
    }
  };

  auto t0 = std::chrono::steady_clock::now();
  std::priority_queue<QItem, std::vector<QItem>, Greater> open;
  std::unordered_map<Cell, double> g;
  std::unordered_map<Cell, Cell> parent;
  std::unordered_map<Cell, bool> settled;
  unsigned long long seq = 0;

  g[start] = 0.0;
  parent[start] = start;  // self-parent: grandparent lookup + reconstruct terminate
  open.push({h(start), seq++, start});

  PlanResult<Cell> result;
  PlanStats stats;
  bool found = false;

  while (!open.empty()) {
    Cell s = open.top().cell;
    open.pop();
    if (settled[s]) continue;  // stale duplicate with a worse key
    settled[s] = true;
    if (recorder) recorder->node_expanded(s, g[s]);
    ++stats.expanded_nodes;
    if (s == goal) {
      found = true;
      break;
    }
    const Cell p = parent[s];
    for (const auto& [s2, edge_cost] : space.neighbors(s)) {
      if (settled[s2]) continue;
      double cand;
      Cell par;
      double ecost;
      if (space.line_of_sight(p, s2)) {
        // Path 2 — any-angle shortcut straight from the grandparent.
        ecost = euclid(p, s2);
        cand = g[p] + ecost;
        par = p;
      } else {
        // Path 1 — standard grid step through s.
        ecost = edge_cost;
        cand = g[s] + ecost;
        par = s;
      }
      auto it = g.find(s2);
      if (it == g.end() || cand < it->second) {
        g[s2] = cand;
        parent[s2] = par;
        if (recorder) recorder->candidate_evaluated(s2, cand);
        if (recorder) recorder->edge_added(s2, par, ecost);
        open.push({cand + h(s2), seq++, s2});
      }
    }
  }

  if (found) {
    result.success = true;
    result.path = reconstruct_path(parent, start, goal);
    result.cost = g[goal];  // NOT path_cost(): it sums adjacent edges only and is ~0 for jumps.
    if (recorder) recorder->path_found(result.path);  // emit_finished_discrete does not.
  }
  result.stats = stats;
  double rt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  emit_finished_discrete(recorder, result.success, result.cost, stats, rt);
  return result;
}

}  // namespace navigation::global_planning
