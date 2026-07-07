#include "navigation/global_planning/search/anya.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "navigation/global_planning/search/discrete_search.hpp"

namespace navigation::global_planning {

using core::Cell;
using core::LineOfSightSpace;
using core::PlanResult;
using core::PlanStats;
using core::TraceRecorder;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// A continuous point in the geometry frame (x = col + 0.5, y = row + 0.5).
struct Pt {
  double x = 0.0;
  double y = 0.0;
  bool operator==(const Pt& o) const { return x == o.x && y == o.y; }
};
struct PtHash {
  std::size_t operator()(const Pt& p) const noexcept {
    std::uint64_t hx, hy;
    static_assert(sizeof(double) == sizeof(std::uint64_t));
    std::memcpy(&hx, &p.x, sizeof(double));
    std::memcpy(&hy, &p.y, sizeof(double));
    std::size_t h = std::hash<std::uint64_t>()(hx);
    return h ^ (std::hash<std::uint64_t>()(hy) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2));
  }
};
// Row-major (y then x) order so both languages expand successors identically.
struct PtYX {
  bool operator()(const Pt& a, const Pt& b) const {
    return a.y < b.y || (a.y == b.y && a.x < b.x);
  }
};

using Interval = std::array<double, 3>;  // {yrow, lo, hi} in the geometry frame

// sqrt(dx^2 + dy^2), NOT std::hypot: hypot is relaxed-accuracy and can differ 1 ULP
// from the Python mirror, diverging the cost the bench compares across languages.
double euclid(double ax, double ay, double bx, double by) {
  double dx = ax - bx, dy = ay - by;
  return std::sqrt(dx * dx + dy * dy);
}

bool cell_free(const std::unordered_set<Cell>& free, int cx, int cy) {
  // Square with integer min-corner (x=cx, y=cy) == cell (row=cy, col=cx).
  return free.count(Cell{cy, cx}) > 0;
}

bool is_corner(const std::unordered_set<Cell>& free, int x, int y) {
  // Convex/reflex obstacle corner iff its four incident cells mix free and blocked.
  const std::array<std::pair<int, int>, 4> cells{{{x - 1, y - 1}, {x, y - 1},
                                                  {x - 1, y}, {x, y}}};
  int blocked = 0;
  for (auto [cx, cy] : cells) blocked += cell_free(free, cx, cy) ? 0 : 1;
  return blocked > 0 && blocked < 4;
}

double proj_x(double rx, double ry, double x0, double y0, double yn) {
  return rx + (x0 - rx) * (yn - ry) / (y0 - ry);
}

bool seg_clear(const std::unordered_set<Cell>& free, const Pt& p, const Pt& q, double eps) {
  // Valid any-angle move: enters no blocked cell interior, does not travel along a
  // grid edge whose two sides are both blocked, and squeezes through no blocked pinch
  // corner (corner-cutting forbidden). A segment lying exactly on an integer grid line
  // only grazes cell boundaries, so it stays valid while ONE adjacent side is free —
  // obstacle corners may be hugged (Harabor et al. 2016). Snapping the on-line sample
  // to one side with floor would wrongly forbid such edge-grazing legs and break
  // Euclidean optimality; the two straddling cells are tested instead.
  if (p.x == q.x && p.y == q.y) return true;
  double px = p.x, py = p.y, dx = q.x - px, dy = q.y - py;
  std::set<double> ts{0.0, 1.0};
  if (dx != 0.0) {
    double lo = std::min(px, q.x), hi = std::max(px, q.x);
    for (int xi = static_cast<int>(std::ceil(lo)); xi <= static_cast<int>(std::floor(hi)); ++xi)
      ts.insert((xi - px) / dx);
  }
  if (dy != 0.0) {
    double lo = std::min(py, q.y), hi = std::max(py, q.y);
    for (int yi = static_cast<int>(std::ceil(lo)); yi <= static_cast<int>(std::floor(hi)); ++yi)
      ts.insert((yi - py) / dy);
  }
  std::vector<double> ordered;
  for (double t : ts)
    if (t >= 0.0 && t <= 1.0) ordered.push_back(t);
  for (std::size_t i = 0; i + 1 < ordered.size(); ++i) {
    double a = ordered[i], b = ordered[i + 1];
    if (b - a < 1e-12) continue;
    double tm = 0.5 * (a + b);
    double mx = px + tm * dx, my = py + tm * dy;
    if (dx == 0.0 && std::abs(mx - std::round(mx)) < eps) {
      // Vertical segment on grid column x=round(mx): edge-graze the two cells it
      // straddles; blocked only if BOTH are solid.
      int xi = static_cast<int>(std::round(mx)), row = static_cast<int>(std::floor(my));
      if (!cell_free(free, xi - 1, row) && !cell_free(free, xi, row)) return false;
    } else if (dy == 0.0 && std::abs(my - std::round(my)) < eps) {
      // Horizontal segment on grid row y=round(my): same edge-graze rule.
      int yi = static_cast<int>(std::round(my)), col = static_cast<int>(std::floor(mx));
      if (!cell_free(free, col, yi - 1) && !cell_free(free, col, yi)) return false;
    } else if (!cell_free(free, static_cast<int>(std::floor(mx)),
                           static_cast<int>(std::floor(my)))) {
      return false;
    }
  }
  if (dx != 0.0 && dy != 0.0) {
    for (double t : ordered) {
      if (t <= eps || t >= 1.0 - eps) continue;
      double x = px + t * dx, y = py + t * dy;
      if (std::abs(x - std::round(x)) < eps && std::abs(y - std::round(y)) < eps) {
        int ix = static_cast<int>(std::round(x)), iy = static_cast<int>(std::round(y));
        if ((dx > 0.0) == (dy > 0.0)) {
          if (!cell_free(free, ix - 1, iy) && !cell_free(free, ix, iy - 1)) return false;
        } else if (!cell_free(free, ix - 1, iy - 1) && !cell_free(free, ix, iy)) {
          return false;
        }
      }
    }
  }
  return true;
}

std::vector<std::pair<double, double>> merge(std::vector<std::pair<double, double>> pieces,
                                             double eps) {
  if (pieces.empty()) return {};
  std::sort(pieces.begin(), pieces.end());
  std::vector<std::pair<double, double>> out{pieces.front()};
  for (std::size_t i = 1; i < pieces.size(); ++i) {
    if (pieces[i].first <= out.back().second + eps)
      out.back().second = std::max(out.back().second, pieces[i].second);
    else
      out.push_back(pieces[i]);
  }
  return out;
}

std::vector<std::pair<double, double>> clear_pieces(const std::unordered_set<Cell>& free,
                                                    const Pt& root, double yn, double lo,
                                                    double hi, const std::vector<double>& splits,
                                                    double eps) {
  // Maximal x-subintervals of [lo,hi] on row yn fully visible from root; split at
  // candidate visibility transitions, each homogeneous piece verified exactly.
  std::set<double> pts{lo, hi};
  for (double s : splits)
    if (lo - eps <= s && s <= hi + eps) pts.insert(std::min(std::max(s, lo), hi));
  std::vector<double> ordered(pts.begin(), pts.end());
  std::vector<std::pair<double, double>> pieces;
  for (std::size_t i = 0; i + 1 < ordered.size(); ++i) {
    double a = ordered[i], b = ordered[i + 1];
    if (b - a < 1e-7) continue;
    double mid = 0.5 * (a + b);
    if (seg_clear(free, root, Pt{mid, yn}, eps)) pieces.push_back({a, b});
  }
  return merge(std::move(pieces), eps);
}

void emit(const std::unordered_set<Cell>& free, const Pt& root, double y,
          const std::vector<std::pair<double, double>>& beams,
          std::map<Pt, Interval, PtYX>& found, double eps) {
  int iy = static_cast<int>(std::round(y));
  for (auto [a, b] : beams) {
    int x0 = static_cast<int>(std::ceil(a - 1e-6));
    int x1 = static_cast<int>(std::floor(b + 1e-6));
    for (int x = x0; x <= x1; ++x) {
      if (is_corner(free, x, iy) && seg_clear(free, root, Pt{static_cast<double>(x), y}, eps))
        found.emplace(Pt{static_cast<double>(x), y}, Interval{y, a, b});
    }
  }
}

// Cone (row-by-row projection) + flat (along the root's row) successor corners,
// each with the interval it was first reached through, in deterministic (y,x) order.
std::vector<std::pair<Pt, Interval>> successors(const std::unordered_set<Cell>& free,
                                                const Pt& root, int r0, int r1, int c0, int c1,
                                                double eps) {
  double rx = root.x, ry = root.y;
  std::map<Pt, Interval, PtYX> found;
  double span_lo = c0 - 2.0, span_hi = c1 + 3.0;

  for (int direction : {1, -1}) {
    double y = direction > 0 ? std::floor(ry) + 1 : std::ceil(ry) - 1;
    std::vector<double> splits;
    for (int e = static_cast<int>(span_lo) - 1; e <= static_cast<int>(span_hi) + 1; ++e)
      splits.push_back(static_cast<double>(e));
    auto beams = clear_pieces(free, root, y, span_lo, span_hi, splits, eps);
    emit(free, root, y, beams, found, eps);
    int steps = 0;
    while (!beams.empty() && r0 - 2 <= y && y <= r1 + 2 && steps < 400) {
      double yn = y + direction;
      std::vector<std::pair<double, double>> child;
      for (auto [a, b] : beams) {
        double an = proj_x(rx, ry, a, y, yn), bn = proj_x(rx, ry, b, y, yn);
        double lo = std::min(an, bn), hi = std::max(an, bn);
        std::vector<double> cand;
        for (int e = static_cast<int>(std::floor(lo)) - 1; e <= static_cast<int>(std::floor(hi)) + 1;
             ++e) {
          cand.push_back(static_cast<double>(e));
          cand.push_back(proj_x(rx, ry, static_cast<double>(e), y, yn));
        }
        auto pieces = clear_pieces(free, root, yn, lo, hi, cand, eps);
        child.insert(child.end(), pieces.begin(), pieces.end());
      }
      y = yn;
      beams = merge(std::move(child), eps);
      emit(free, root, y, beams, found, eps);
      ++steps;
    }
  }

  if (std::abs(ry - std::round(ry)) < eps) {
    int yr = static_cast<int>(std::round(ry));
    for (int direction : {1, -1}) {
      int x = static_cast<int>(std::round(rx));
      int steps = 0;
      while (c0 - 2 <= x && x <= c1 + 2 && steps < 400) {
        int col = direction > 0 ? x : x - 1;
        if (!cell_free(free, col, yr - 1) && !cell_free(free, col, yr)) break;
        x += direction;
        ++steps;
        Pt cp{static_cast<double>(x), static_cast<double>(yr)};
        if (is_corner(free, x, yr) && seg_clear(free, root, cp, eps))
          found.emplace(cp, Interval{static_cast<double>(yr), static_cast<double>(x),
                                     static_cast<double>(x)});
      }
    }
  }

  found.erase(root);
  std::vector<std::pair<Pt, Interval>> out(found.begin(), found.end());
  return out;
}

// Corner (integer x,y) -> a representative Cell for the shared list<Cell> / viz
// contract: start/goal render at their own cell; a corner has none of its own, so
// it renders at the first incident FREE cell (fixed order). cost stays the exact
// corner geometry, not this snap.
Cell cell_of(const std::unordered_set<Cell>& free, const Pt& p) {
  double row = p.y - 0.5, col = p.x - 0.5;
  if (std::abs(row - std::round(row)) < 1e-6 && std::abs(col - std::round(col)) < 1e-6)
    return Cell{static_cast<int>(std::round(row)), static_cast<int>(std::round(col))};
  int x = static_cast<int>(std::round(p.x)), y = static_cast<int>(std::round(p.y));
  const std::array<std::pair<int, int>, 4> around{{{x - 1, y - 1}, {x - 1, y},
                                                   {x, y - 1}, {x, y}}};
  for (auto [cx, cy] : around)
    if (cell_free(free, cx, cy)) return Cell{cy, cx};
  return Cell{y, x};
}

}  // namespace

core::PlanResult<Cell> AnyaPlanner::plan(LineOfSightSpace<Cell>& space, const Cell& start,
                                         const Cell& goal, TraceRecorder* recorder) {
  auto t0 = std::chrono::steady_clock::now();
  const double eps = params_.get_float("vertex_epsilon");

  // Occupancy observed via the capability: the start's reachable free component.
  std::unordered_set<Cell> free{start};
  std::vector<Cell> stack{start};
  while (!stack.empty()) {
    Cell cur = stack.back();
    stack.pop_back();
    for (const auto& [nb, cost] : space.neighbors(cur)) {
      (void)cost;
      if (free.insert(nb).second) stack.push_back(nb);
    }
  }
  int r0 = start.row, r1 = start.row, c0 = start.col, c1 = start.col;
  for (const Cell& c : free) {
    r0 = std::min(r0, c.row);
    r1 = std::max(r1, c.row);
    c0 = std::min(c0, c.col);
    c1 = std::max(c1, c.col);
  }

  const Pt s_pt{start.col + 0.5, start.row + 0.5};
  const Pt g_pt{goal.col + 0.5, goal.row + 0.5};

  struct QItem {
    double f;
    unsigned long long seq;
    Pt p;
  };
  struct Greater {
    bool operator()(const QItem& a, const QItem& b) const {
      return a.f > b.f || (a.f == b.f && a.seq > b.seq);
    }
  };

  std::unordered_map<Pt, double, PtHash> g;
  std::unordered_map<Pt, Pt, PtHash> parent;
  std::unordered_map<Pt, Interval, PtHash> via_interval;
  std::unordered_set<Pt, PtHash> settled;
  std::priority_queue<QItem, std::vector<QItem>, Greater> open;
  unsigned long long seq = 0;

  g[s_pt] = 0.0;
  parent[s_pt] = s_pt;
  open.push({euclid(s_pt.x, s_pt.y, g_pt.x, g_pt.y), seq++, s_pt});

  double goal_cost = kInf;
  bool have_goal = false;
  Pt goal_root{};
  PlanStats stats;

  while (!open.empty()) {
    QItem top = open.top();
    open.pop();
    Pt root = top.p;
    if (settled.count(root)) continue;
    if (top.f >= goal_cost - eps) break;
    settled.insert(root);
    ++stats.expanded_nodes;
    if (recorder) recorder->node_expanded(cell_of(free, root), g[root]);
    if (seg_clear(free, root, g_pt, eps)) {
      double cand = g[root] + euclid(root.x, root.y, g_pt.x, g_pt.y);
      if (cand < goal_cost) {
        goal_cost = cand;
        goal_root = root;
        have_goal = true;
      }
    }
    for (const auto& [corner, iv] : successors(free, root, r0, r1, c0, c1, eps)) {
      double nd = g[root] + euclid(root.x, root.y, corner.x, corner.y);
      auto it = g.find(corner);
      if (it == g.end() || nd < it->second - eps) {
        g[corner] = nd;
        parent[corner] = root;
        Interval rec{iv[0] - 0.5, iv[1] - 0.5, iv[2] - 0.5};
        via_interval[corner] = rec;
        if (recorder) {
          Cell c_cell = cell_of(free, corner);
          recorder->candidate_evaluated(c_cell, nd);
          TraceRecorder::EventData data{{"row", rec[0]}, {"col_lo", rec[1]}, {"col_hi", rec[2]}};
          recorder->edge_added(c_cell, cell_of(free, root),
                               euclid(root.x, root.y, corner.x, corner.y), data);
        }
        open.push({nd + euclid(corner.x, corner.y, g_pt.x, g_pt.y), seq++, corner});
      }
    }
  }

  PlanResult<Cell> result;
  if (have_goal) {
    // Reconstruct start -> corners -> goal, snapping corners to cells for list<Cell>.
    std::vector<Pt> chain{goal_root};
    Pt node = goal_root;
    while (!(node == s_pt)) {
      node = parent[node];
      chain.push_back(node);
    }
    std::reverse(chain.begin(), chain.end());
    result.path.push_back(start);
    for (const Pt& pt : chain) {
      Cell cell = cell_of(free, pt);
      if (!(cell == result.path.back())) result.path.push_back(cell);
    }
    if (!(result.path.back() == goal)) result.path.push_back(goal);
    result.success = true;
    result.cost = goal_cost;
    if (recorder) recorder->path_found(result.path);
  }
  result.stats = stats;
  double rt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  emit_finished_discrete(recorder, result.success, result.cost, stats, rt);
  return result;
}

}  // namespace navigation::global_planning
