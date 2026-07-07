#include "navigation/global_planning/search/jps.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "navigation/global_planning/search/discrete_search.hpp"  // emit_finished_discrete

namespace navigation::global_planning {

using core::DynamicGridSpace;

namespace {

const double kSqrt2 = std::sqrt(2.0);

// Octile distance on integer cell deltas, in the same operation order —
// (hi - lo) + sqrt(2)*lo — as OccupancyGrid2D::heuristic. A pure straight/diagonal
// jump line has this as its true traversed cost, so JPS f-values and the returned
// cost match 8-connected A* exactly. Admissible.
double octile(const Cell& a, const Cell& b) {
  int dr = std::abs(a.row - b.row);
  int dc = std::abs(a.col - b.col);
  int lo = std::min(dr, dc);
  int hi = std::max(dr, dc);
  return static_cast<double>(hi - lo) + kSqrt2 * static_cast<double>(lo);
}

int sgn(int x) { return (x > 0) - (x < 0); }

// Single-cell occupancy oracle over DynamicGridSpace::is_blocked (occupied OR out of
// bounds) — the only grid primitive JPS reads.
struct Oracle {
  DynamicGridSpace<Cell>& space;
  bool blocked(int r, int c) const { return space.is_blocked(Cell{r, c}); }
  bool freec(int r, int c) const { return !space.is_blocked(Cell{r, c}); }
};

// First jump point along (dr,dc) from (r,c), or nullopt if the run dead-ends at an
// obstacle / boundary. Recursion depth is at most 1: a diagonal run's orthogonal
// probes are straight scans that never recurse (Harabor & Grastien 2011).
std::optional<Cell> scan(const Oracle& o, int r, int c, int dr, int dc, const Cell& goal) {
  const bool diagonal = dr != 0 && dc != 0;
  while (true) {
    // Step legality mirrors the map's corner rule: a diagonal needs the target and
    // both shared orthogonal cells free (no corner cutting).
    if (o.blocked(r + dr, c + dc)) return std::nullopt;
    if (diagonal && (o.blocked(r + dr, c) || o.blocked(r, c + dc))) return std::nullopt;
    r += dr;
    c += dc;
    if (r == goal.row && c == goal.col) return Cell{r, c};
    if (!diagonal) {
      if (dc != 0) {  // horizontal: obstacle diagonally behind opens a side cell
        if ((o.freec(r - 1, c) && o.blocked(r - 1, c - dc)) ||
            (o.freec(r + 1, c) && o.blocked(r + 1, c - dc)))
          return Cell{r, c};
      } else {  // vertical
        if ((o.freec(r, c - 1) && o.blocked(r - dr, c - 1)) ||
            (o.freec(r, c + 1) && o.blocked(r - dr, c + 1)))
          return Cell{r, c};
      }
    } else if (scan(o, r, c, dr, 0, goal).has_value() ||
               scan(o, r, c, 0, dc, goal).has_value()) {
      // A diagonal cell whose orthogonal scan finds a jump point is itself one.
      return Cell{r, c};
    }
  }
}

// Directions to jump from u given how it was reached. Start (no parent) explores all
// eight; otherwise natural continuation + forced-neighbour branches. Illegal branches
// simply yield no jump point, so a slightly liberal set never costs correctness.
std::vector<std::pair<int, int>> successor_dirs(const Oracle& o, const Cell& u,
                                                const std::optional<Cell>& parent) {
  static const std::pair<int, int> kAll[8] = {{-1, 0}, {1, 0},  {0, -1}, {0, 1},
                                              {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
  // Parenthesised (iterator-range) ctor — a braced list here would try to build an
  // initializer_list<pair> from two pointers.
  if (!parent.has_value()) return std::vector<std::pair<int, int>>(kAll, kAll + 8);
  const int pdr = sgn(u.row - parent->row);
  const int pdc = sgn(u.col - parent->col);
  if (pdr != 0 && pdc != 0) {  // diagonal: continuation + both orthogonal legs
    return {{pdr, 0}, {0, pdc}, {pdr, pdc}};
  }
  std::vector<std::pair<int, int>> dirs{{pdr, pdc}};  // natural continuation
  if (pdc != 0) {                                     // horizontal
    if (o.freec(u.row - 1, u.col) && o.blocked(u.row - 1, u.col - pdc)) {
      dirs.push_back({-1, pdc});
      dirs.push_back({-1, 0});
    }
    if (o.freec(u.row + 1, u.col) && o.blocked(u.row + 1, u.col - pdc)) {
      dirs.push_back({1, pdc});
      dirs.push_back({1, 0});
    }
  } else {  // vertical
    if (o.freec(u.row, u.col - 1) && o.blocked(u.row - pdr, u.col - 1)) {
      dirs.push_back({pdr, -1});
      dirs.push_back({0, -1});
    }
    if (o.freec(u.row, u.col + 1) && o.blocked(u.row - pdr, u.col + 1)) {
      dirs.push_back({pdr, 1});
      dirs.push_back({0, 1});
    }
  }
  return dirs;
}

// JPS's parent chain is sparse (jump points only). Fill the straight/diagonal cells
// between consecutive jump points so the reported path is the full staircase, matching
// how A* reports every cell.
std::vector<Cell> reconstruct_full(const std::unordered_map<Cell, Cell>& parent,
                                   const Cell& start, const Cell& goal) {
  std::vector<Cell> jumps;
  Cell cur = goal;
  jumps.push_back(cur);
  while (!(cur == start)) {
    cur = parent.at(cur);
    jumps.push_back(cur);
  }
  std::reverse(jumps.begin(), jumps.end());
  std::vector<Cell> path{start};
  for (size_t i = 1; i < jumps.size(); ++i) {
    const Cell a = jumps[i - 1];
    const Cell b = jumps[i];
    const int dr = sgn(b.row - a.row);
    const int dc = sgn(b.col - a.col);
    Cell c = a;
    while (!(c == b)) {
      c = Cell{c.row + dr, c.col + dc};
      path.push_back(c);
    }
  }
  return path;
}

}  // namespace

core::PlanResult<Cell> JpsPlanner::plan(DynamicGridSpace<Cell>& space, const Cell& start,
                                        const Cell& goal, TraceRecorder* recorder) {
  Oracle oracle{space};
  struct QItem {
    double f;
    unsigned long long seq;  // insertion order; FIFO tie-break keeps ties stable
    Cell cell;
  };
  struct Greater {
    bool operator()(const QItem& a, const QItem& b) const {
      // Same (f, seq) tie-break as A* so the deterministic search is stable.
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
  open.push({octile(start, goal), seq++, start});

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
    std::optional<Cell> par;
    if (auto it = parent.find(u); it != parent.end()) par = it->second;
    for (const auto& [dr, dc] : successor_dirs(oracle, u, par)) {
      std::optional<Cell> jp = scan(oracle, u.row, u.col, dr, dc, goal);
      if (!jp.has_value() || settled[*jp]) continue;
      double ng = g[u] + octile(u, *jp);
      auto it = g.find(*jp);
      if (it == g.end() || ng < it->second) {
        g[*jp] = ng;
        parent[*jp] = u;
        if (recorder) recorder->candidate_evaluated(*jp, ng);
        if (recorder) recorder->edge_added(*jp, u, octile(u, *jp));
        open.push({ng + octile(*jp, goal), seq++, *jp});
      }
    }
  }

  if (found) {
    result.success = true;
    result.path = reconstruct_full(parent, start, goal);
    result.cost = g[goal];  // jump-point path cost; NOT path_cost() (sums per-edge only).
    if (recorder) recorder->path_found(result.path);
  }
  result.stats = stats;
  double rt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  emit_finished_discrete(recorder, result.success, result.cost, stats, rt);
  return result;
}

}  // namespace navigation::global_planning
