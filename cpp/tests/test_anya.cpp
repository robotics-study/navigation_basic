#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/global_planning/search/anya.hpp"
#include "navigation/global_planning/search/theta_star.hpp"
#include "navigation/global_planning/search/visibility_astar.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::Cell;

namespace {

core::ParamSet cfg(const std::string& algo) {
  return core::ParamSet::from_yaml(test::repo_path("configs/global_planning/" + algo + ".yaml"));
}

using Pt = std::pair<double, double>;

// Independent ground truth: turning points of a shortest Euclidean path in a
// blocked-cell domain lie only at convex obstacle corners (grid vertices), so the
// optimum is an all-pairs corner-visibility Dijkstra with a pinch-aware corner LOS
// over the same reachable component the planner observes. A different computation
// than Anya's interval sweep, so agreement is a real correctness check.

bool cfree(const std::unordered_set<Cell>& free, int cx, int cy) {
  return free.count(Cell{cy, cx}) > 0;
}

// t-interval where p + t*d lies strictly inside the open interval (lo, hi); ok=false
// if empty. A component staying exactly on a boundary (d==0, p==lo/hi) is empty, so
// grazing a cell edge is not counted as crossing that cell's interior.
struct Slab {
  bool ok;
  double t0, t1;
};
Slab open_slab(double p, double d, double lo, double hi) {
  if (d == 0.0) return (lo < p && p < hi) ? Slab{true, -1e300, 1e300} : Slab{false, 0.0, 0.0};
  double a = (lo - p) / d, b = (hi - p) / d;
  return a <= b ? Slab{true, a, b} : Slab{true, b, a};
}

// Exact grazing-aware any-angle line of sight, computed independently of the planner:
// (1) analytic Liang-Barsky clip against each blocked cell's OPEN square (positive
// overlap = real interior crossing; touching an edge is not), (2) no travel along a
// grid edge with both sides blocked, (3) no corner-cut through a diagonal blocked
// pinch. Edge-grazing an obstacle corner is allowed (Harabor et al. 2016).
bool seg_clear(const std::unordered_set<Cell>& free, const Pt& p, const Pt& q, double eps = 1e-9) {
  if (p == q) return true;
  double px = p.first, py = p.second, qx = q.first, qy = q.second;
  double dx = qx - px, dy = qy - py;
  double xlo = std::min(px, qx), xhi = std::max(px, qx);
  double ylo = std::min(py, qy), yhi = std::max(py, qy);
  for (int cy = static_cast<int>(std::floor(ylo)); cy < static_cast<int>(std::ceil(yhi)); ++cy) {
    for (int cx = static_cast<int>(std::floor(xlo)); cx < static_cast<int>(std::ceil(xhi)); ++cx) {
      if (cfree(free, cx, cy)) continue;
      Slab sx = open_slab(px, dx, cx, cx + 1), sy = open_slab(py, dy, cy, cy + 1);
      if (!sx.ok || !sy.ok) continue;
      double t0 = std::max({sx.t0, sy.t0, 0.0}), t1 = std::min({sx.t1, sy.t1, 1.0});
      if (t1 - t0 > 1e-9) return false;
    }
  }
  if (dx == 0.0 && std::abs(px - std::round(px)) < eps) {
    int xi = static_cast<int>(std::round(px));
    for (int row = static_cast<int>(std::floor(ylo)); row < static_cast<int>(std::ceil(yhi)); ++row)
      if (ylo < row + 1 && yhi > row && !cfree(free, xi - 1, row) && !cfree(free, xi, row))
        return false;
  }
  if (dy == 0.0 && std::abs(py - std::round(py)) < eps) {
    int yi = static_cast<int>(std::round(py));
    for (int col = static_cast<int>(std::floor(xlo)); col < static_cast<int>(std::ceil(xhi)); ++col)
      if (xlo < col + 1 && xhi > col && !cfree(free, col, yi - 1) && !cfree(free, col, yi))
        return false;
  }
  if (dx != 0.0 && dy != 0.0) {
    for (int ix = static_cast<int>(std::ceil(xlo)); ix <= static_cast<int>(std::floor(xhi)); ++ix) {
      double t = (ix - px) / dx;
      if (t <= eps || t >= 1.0 - eps) continue;
      double y = py + t * dy;
      if (std::abs(y - std::round(y)) < eps) {
        int iy = static_cast<int>(std::round(y));
        if ((dx > 0.0) == (dy > 0.0)) {
          if (!cfree(free, ix - 1, iy) && !cfree(free, ix, iy - 1)) return false;
        } else if (!cfree(free, ix - 1, iy - 1) && !cfree(free, ix, iy)) {
          return false;
        }
      }
    }
  }
  return true;
}

double corner_optimum(maps::OccupancyGrid2D& grid, const Cell& start, const Cell& goal) {
  std::unordered_set<Cell> free{start};
  std::vector<Cell> st{start};
  while (!st.empty()) {
    Cell c = st.back();
    st.pop_back();
    for (const auto& [nb, w] : grid.neighbors(c)) {
      (void)w;
      if (free.insert(nb).second) st.push_back(nb);
    }
  }
  if (!free.count(goal)) return std::numeric_limits<double>::infinity();
  int r0 = start.row, r1 = start.row, c0 = start.col, c1 = start.col;
  for (const Cell& c : free) {
    r0 = std::min(r0, c.row);
    r1 = std::max(r1, c.row);
    c0 = std::min(c0, c.col);
    c1 = std::max(c1, c.col);
  }
  std::vector<Pt> nodes{{start.col + 0.5, start.row + 0.5}, {goal.col + 0.5, goal.row + 0.5}};
  for (int y = r0; y <= r1 + 1; ++y) {
    for (int x = c0; x <= c1 + 1; ++x) {
      int blocked = 0;
      for (auto [cx, cy] : {std::pair{x - 1, y - 1}, {x, y - 1}, {x - 1, y}, {x, y}})
        blocked += cfree(free, cx, cy) ? 0 : 1;
      if (blocked > 0 && blocked < 4) nodes.push_back({static_cast<double>(x), static_cast<double>(y)});
    }
  }
  int n = static_cast<int>(nodes.size());
  std::vector<double> dist(n, std::numeric_limits<double>::infinity());
  std::vector<bool> settled(n, false);
  dist[0] = 0.0;
  using QI = std::pair<double, int>;
  std::priority_queue<QI, std::vector<QI>, std::greater<QI>> pq;
  pq.push({0.0, 0});
  while (!pq.empty()) {
    auto [d, u] = pq.top();
    pq.pop();
    if (settled[u]) continue;
    settled[u] = true;
    if (u == 1) return d;
    for (int v = 0; v < n; ++v) {
      if (settled[v] || v == u) continue;
      if (seg_clear(free, nodes[u], nodes[v])) {
        double nd = d + std::hypot(nodes[u].first - nodes[v].first,
                                   nodes[u].second - nodes[v].second);
        if (nd < dist[v]) {
          dist[v] = nd;
          pq.push({nd, v});
        }
      }
    }
  }
  return std::numeric_limits<double>::infinity();
}

}  // namespace

// (a) OPTIMALITY: turning at a grid corner beats every cell-centre planner --------

// A vertical bar hides the goal; the shortest route grazes the bar's top-left grid
// corner (2,2). Turning THERE (not at any cell centre) yields the true Euclidean
// optimum 2*sqrt(8.5); Visibility A* / Theta*, pinned to cell centres, are strictly
// longer. This is Anya's defining property.
TEST(Anya, TurnsAtCornerAndBeatsCellCentre) {
  auto g = test::make_grid({".....", ".....", "..#..", "..#..", "....."});
  Cell start{4, 0}, goal{0, 4};
  auto ra = global_planning::AnyaPlanner(cfg("anya")).plan(g, start, goal, nullptr);
  ASSERT_TRUE(ra.success);
  EXPECT_EQ(ra.path.front(), start);
  EXPECT_EQ(ra.path.back(), goal);
  EXPECT_NEAR(ra.cost, 2.0 * std::sqrt(8.5), 1e-9);
  EXPECT_NEAR(ra.cost, corner_optimum(g, start, goal), 1e-9);
  global_planning::VisibilityAStarPlanner vis(cfg("visibility_astar"));
  global_planning::ThetaStarPlanner theta(cfg("theta_star"));
  EXPECT_LT(ra.cost, vis.plan(g, start, goal, nullptr).cost - 1e-9);
  EXPECT_LT(ra.cost, theta.plan(g, start, goal, nullptr).cost - 1e-9);
}

// A second, asymmetric instance guards against a projection correct only on the
// symmetric bend above.
TEST(Anya, MatchesCornerOptimumOnAsymmetricMap) {
  auto g = test::make_grid({
      "........", "...##...", "...##...", ".....#..", "..#.....", "........",
  });
  Cell start{5, 0}, goal{0, 7};
  auto ra = global_planning::AnyaPlanner(cfg("anya")).plan(g, start, goal, nullptr);
  ASSERT_TRUE(ra.success);
  EXPECT_NEAR(ra.cost, corner_optimum(g, start, goal), 1e-9);
  global_planning::VisibilityAStarPlanner vis(cfg("visibility_astar"));
  EXPECT_LE(ra.cost, vis.plan(g, start, goal, nullptr).cost + 1e-9);
}

// (a') EDGE-GRAZING optimum: a taut leg hugs an obstacle edge without entering its
// interior. Snapping the on-grid-line line-of-sight sample to one side (floor) would
// forbid such a leg and inflate the cost. Optimum 1 + sqrt(2) here.
TEST(Anya, OptimumHugsBlockedCellEdge) {
  auto g = test::make_grid({".##.", "..#.", "....", "..#."});
  Cell start{3, 1}, goal{3, 3};
  auto ra = global_planning::AnyaPlanner(cfg("anya")).plan(g, start, goal, nullptr);
  ASSERT_TRUE(ra.success);
  EXPECT_NEAR(ra.cost, 1.0 + std::sqrt(2.0), 1e-9);
  EXPECT_NEAR(ra.cost, corner_optimum(g, start, goal), 1e-9);
}

// A corner-turning instance where floor-snapped LOS made Anya STRICTLY worse than
// Visibility A*; the true any-angle optimum must never exceed it.
TEST(Anya, NeverExceedsVisibilityAStarOnCornerTurn) {
  auto g = test::make_grid({".......", "#.##...", "##...#.", ".....#.", "....#.."});
  Cell start{2, 6}, goal{2, 2};
  auto ra = global_planning::AnyaPlanner(cfg("anya")).plan(g, start, goal, nullptr);
  ASSERT_TRUE(ra.success);
  EXPECT_NEAR(ra.cost, corner_optimum(g, start, goal), 1e-9);
  global_planning::VisibilityAStarPlanner vis(cfg("visibility_astar"));
  EXPECT_LE(ra.cost, vis.plan(g, start, goal, nullptr).cost + 1e-9);
}

// (a'') PROPERTY: Anya == exact grazing-aware corner optimum on random maps. The
// oracle uses analytic Liang-Barsky clips (a different computation than the planner's
// sampled sweep), so agreement pins true continuous Euclidean optimality.
TEST(Anya, MatchesExactOracleOnRandomGrids) {
  int checked = 0;
  for (unsigned seed = 0; seed < 600; ++seed) {
    std::mt19937 rng(seed);
    int rows = std::uniform_int_distribution<int>(3, 6)(rng);
    int cols = std::uniform_int_distribution<int>(3, 6)(rng);
    double density = std::uniform_real_distribution<double>(0.12, 0.32)(rng);
    std::vector<std::string> ascii(rows, std::string(cols, '.'));
    std::vector<Cell> frees;
    for (int r = 0; r < rows; ++r)
      for (int c = 0; c < cols; ++c) {
        if (std::uniform_real_distribution<double>(0.0, 1.0)(rng) < density)
          ascii[r][c] = '#';
        else
          frees.push_back(Cell{r, c});
      }
    if (frees.size() < 2) continue;
    Cell start = frees[std::uniform_int_distribution<size_t>(0, frees.size() - 1)(rng)];
    Cell goal = frees[std::uniform_int_distribution<size_t>(0, frees.size() - 1)(rng)];
    if (start == goal) continue;
    auto g = test::make_grid(ascii);
    double opt = corner_optimum(g, start, goal);
    if (std::isinf(opt)) continue;  // goal unreachable — no path to compare
    auto ra = global_planning::AnyaPlanner(cfg("anya")).plan(g, start, goal, nullptr);
    ASSERT_TRUE(ra.success) << "seed " << seed;
    EXPECT_NEAR(ra.cost, opt, 1e-9) << "seed " << seed;
    ++checked;
  }
  EXPECT_GT(checked, 200);  // sanity: a meaningful number of solvable instances ran
}

// (b) open space: a single straight segment ---------------------------------------

TEST(Anya, ReturnsStraightLineWhenVisible) {
  auto g = test::make_grid({"....", "....", "....", "...."});
  Cell start{3, 0}, goal{0, 3};
  auto ra = global_planning::AnyaPlanner(cfg("anya")).plan(g, start, goal, nullptr);
  ASSERT_TRUE(ra.success);
  EXPECT_EQ(ra.path.front(), start);
  EXPECT_EQ(ra.path.back(), goal);
  EXPECT_NEAR(ra.cost, std::sqrt(18.0), 1e-9);
}

// (c) no-path case ----------------------------------------------------------------

TEST(Anya, NoPathWhenWalledOff) {
  auto g = test::make_grid({"..#..", "..#..", "..#..", "..#..", "..#.."});
  auto ra = global_planning::AnyaPlanner(cfg("anya")).plan(g, Cell{0, 0}, Cell{0, 4}, nullptr);
  EXPECT_FALSE(ra.success);
  EXPECT_TRUE(ra.path.empty());
  EXPECT_EQ(ra.cost, 0.0);
}

// (d) param validation failure ----------------------------------------------------

TEST(Anya, BadParamThrows) {
  std::string bad = test::write_temp(
      "anya.yaml",
      "algorithm: anya\ncategory: global_planning\nparams:\n"
      "  - name: vertex_epsilon\n    type: float\n    default: 1.0\n    min: 1.0e-12\n"
      "    max: 1.0e-3\n    description: above max\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}
