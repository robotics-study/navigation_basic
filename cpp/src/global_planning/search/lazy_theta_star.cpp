#include "navigation/global_planning/search/lazy_theta_star.hpp"

#include <chrono>
#include <cmath>
#include <limits>
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

core::PlanResult<Cell> LazyThetaStarPlanner::plan(LineOfSightSpace<Cell>& space, const Cell& start,
                                                  const Cell& goal, TraceRecorder* recorder) {
  // Weighted Lazy Theta*: f = g + w*h, h Euclidean (straight-line). w == 1 is
  // standard Lazy Theta*; the octile heuristic (space.heuristic) would be
  // inadmissible for any-angle costs since octile >= Euclidean (Nash & Koenig 2010;
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
      // Same (f, seq) tie-break as A*/Theta* so both languages settle the same path.
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
    // set_vertex — the deferred line-of-sight check (Nash & Koenig 2010). The parent
    // was assumed visible when s was generated; verify it only now, once per expanded
    // vertex.
    Cell p = parent[s];
    if (!(p == s) && !space.line_of_sight(p, s)) {
      // Repair: the optimistic grandparent is not actually visible, so adopt the
      // cheapest already-settled grid neighbour as parent. Path 1 between adjacent
      // cells is always a valid move, so the generator (settled) is a guaranteed
      // visible fallback.
      double best_g = std::numeric_limits<double>::infinity();
      Cell best_par = p;
      double best_cost = 0.0;
      for (const auto& [s3, edge_cost] : space.neighbors(s)) {
        if (settled[s3]) {
          double cand = g[s3] + edge_cost;
          if (cand < best_g) {
            best_g = cand;
            best_par = s3;
            best_cost = edge_cost;
          }
        }
      }
      g[s] = best_g;
      parent[s] = best_par;
      p = best_par;
      if (recorder) {
        // Surface the lazy repair to the visualizer: the deferred check rejected the
        // optimistic parent, so re-emit s with its real (grid-neighbour) parent via
        // the existing relaxation events.
        recorder->candidate_evaluated(s, best_g);
        recorder->edge_added(s, best_par, best_cost);
      }
    }
    settled[s] = true;
    if (recorder) recorder->node_expanded(s, g[s]);
    ++stats.expanded_nodes;
    if (s == goal) {
      found = true;
      break;
    }
    for (const auto& [s2, edge_cost] : space.neighbors(s)) {
      (void)edge_cost;  // lazy Path 2 uses the straight-line cost, not the grid edge
      if (settled[s2]) continue;
      // Lazy Path 2: OPTIMISTICALLY assume line_of_sight(p, s2) instead of checking
      // it here — the check is deferred to set_vertex when s2 pops (Nash & Koenig 2010).
      double ecost = euclid(p, s2);
      double cand = g[p] + ecost;
      auto it = g.find(s2);
      if (it == g.end() || cand < it->second) {
        g[s2] = cand;
        parent[s2] = p;
        if (recorder) recorder->candidate_evaluated(s2, cand);
        if (recorder) recorder->edge_added(s2, p, ecost);
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
