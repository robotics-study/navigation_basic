#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/global_planning/search/anya.hpp"
#include "navigation/global_planning/search/astar.hpp"
#include "navigation/global_planning/search/theta_star.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::Cell;

namespace {

core::ParamSet cfg(const std::string& algo) {
  return core::ParamSet::from_yaml(test::repo_path("configs/global_planning/" + algo + ".yaml"));
}

double euclid(const Cell& a, const Cell& b) {
  double dr = a.row - b.row, dc = a.col - b.col;
  return std::sqrt(dr * dr + dc * dc);
}

// Every reconstructed any-angle leg must be a legal straight move.
bool path_los_clear(maps::OccupancyGrid2D& g, const std::vector<Cell>& path) {
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    if (!g.line_of_sight(path[i], path[i + 1])) return false;
  }
  return true;
}

// Independent ground truth: the Euclidean-shortest any-angle cost over cell-centre
// vertices = Dijkstra on the full visibility graph (every pair of mutually
// LOS-visible reachable free cells, weighted by straight-line length). A different,
// obviously-correct computation than Anya's interval search, so agreement is a real
// optimality check.
double visibility_optimum(maps::OccupancyGrid2D& grid, const Cell& start, const Cell& goal) {
  std::unordered_set<Cell> seen{start};
  std::vector<Cell> st{start};
  while (!st.empty()) {
    Cell c = st.back();
    st.pop_back();
    for (const auto& [nb, w] : grid.neighbors(c)) {
      (void)w;
      if (seen.insert(nb).second) st.push_back(nb);
    }
  }
  std::vector<Cell> cells(seen.begin(), seen.end());
  std::unordered_map<Cell, double> dist;
  dist[start] = 0.0;
  using QI = std::pair<double, Cell>;
  std::priority_queue<QI, std::vector<QI>, std::greater<QI>> pq;
  pq.push({0.0, start});
  std::unordered_set<Cell> settled;
  while (!pq.empty()) {
    auto [d, u] = pq.top();
    pq.pop();
    if (settled.count(u)) continue;
    settled.insert(u);
    if (u == goal) return d;
    for (const Cell& v : cells) {
      if (settled.count(v) || v == u) continue;
      if (grid.line_of_sight(u, v)) {
        double nd = d + euclid(u, v);
        auto it = dist.find(v);
        if (it == dist.end() || nd < it->second) {
          dist[v] = nd;
          pq.push({nd, v});
        }
      }
    }
  }
  return std::numeric_limits<double>::infinity();
}

}  // namespace

// (a) optimal any-angle path -------------------------------------------------

// Goal directly visible from start on an open grid: the global any-angle optimum
// is the single straight segment (cost = Euclidean). Anya returns exactly it, no
// longer than Theta* and strictly shorter than grid A*.
TEST(Anya, ReturnsOptimalStraightLine) {
  auto g = test::make_grid({"...", "...", "..."});
  Cell start{2, 0}, goal{0, 1};
  global_planning::AnyaPlanner anya(cfg("anya"));
  auto ra = anya.plan(g, start, goal, nullptr);
  ASSERT_TRUE(ra.success);
  EXPECT_EQ(ra.path.front(), start);
  EXPECT_EQ(ra.path.back(), goal);
  EXPECT_NEAR(ra.cost, std::sqrt(5.0), 1e-9);
  EXPECT_TRUE(path_los_clear(g, ra.path));
  global_planning::ThetaStarPlanner theta(cfg("theta_star"));
  EXPECT_LE(ra.cost, theta.plan(g, start, goal, nullptr).cost + 1e-9);
  global_planning::AstarPlanner astar(cfg("astar"));
  EXPECT_LT(ra.cost, astar.plan(g, start, goal, nullptr).cost);
}

// A blocker hides the goal, forcing a bend. Anya's cost must equal the
// visibility-graph optimum (true shortest any-angle cost) and never exceed Theta*.
TEST(Anya, MatchesTrueOptimumAroundObstacle) {
  auto g = test::make_grid({".....", ".....", "..#..", "..#..", "....."});
  Cell start{4, 0}, goal{0, 4};
  global_planning::AnyaPlanner anya(cfg("anya"));
  auto ra = anya.plan(g, start, goal, nullptr);
  ASSERT_TRUE(ra.success);
  EXPECT_EQ(ra.path.front(), start);
  EXPECT_EQ(ra.path.back(), goal);
  EXPECT_TRUE(path_los_clear(g, ra.path));
  EXPECT_FALSE(g.line_of_sight(start, goal));  // goal genuinely hidden
  EXPECT_NEAR(ra.cost, visibility_optimum(g, start, goal), 1e-9);
  global_planning::ThetaStarPlanner theta(cfg("theta_star"));
  EXPECT_LE(ra.cost, theta.plan(g, start, goal, nullptr).cost + 1e-9);
}

// Second, differently-shaped instance: guards against a projection optimal only
// on the symmetric bend above.
TEST(Anya, MatchesTrueOptimumOnAsymmetricMap) {
  auto g = test::make_grid({
      "........", "...##...", "...##...", ".....#..", "..#.....", "........",
  });
  Cell start{5, 0}, goal{0, 7};
  global_planning::AnyaPlanner anya(cfg("anya"));
  auto ra = anya.plan(g, start, goal, nullptr);
  ASSERT_TRUE(ra.success);
  EXPECT_TRUE(path_los_clear(g, ra.path));
  EXPECT_NEAR(ra.cost, visibility_optimum(g, start, goal), 1e-9);
  global_planning::ThetaStarPlanner theta(cfg("theta_star"));
  EXPECT_LE(ra.cost, theta.plan(g, start, goal, nullptr).cost + 1e-9);
}

// (b) no-path case -----------------------------------------------------------

TEST(Anya, NoPathWhenWalledOff) {
  auto g = test::make_grid({"..#..", "..#..", "..#..", "..#..", "..#.."});
  global_planning::AnyaPlanner anya(cfg("anya"));
  auto ra = anya.plan(g, Cell{0, 0}, Cell{0, 4}, nullptr);
  EXPECT_FALSE(ra.success);
  EXPECT_TRUE(ra.path.empty());
  EXPECT_EQ(ra.cost, 0.0);
}

// (c) param validation failure -----------------------------------------------

TEST(Anya, BadParamThrows) {
  std::string bad = test::write_temp(
      "anya.yaml",
      "algorithm: anya\ncategory: global_planning\nparams:\n"
      "  - name: heuristic_weight\n    type: float\n    default: 0.5\n    min: 1.0\n    max: 5.0\n"
      "    description: below min\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}
