#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/global_planning/search/astar.hpp"
#include "navigation/global_planning/search/bfs.hpp"
#include "navigation/global_planning/search/dijkstra.hpp"
#include "navigation/global_planning/search/theta_star.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::Cell;

namespace {

core::ParamSet cfg(const std::string& algo) {
  return core::ParamSet::from_yaml(test::repo_path("configs/global_planning/" + algo + ".yaml"));
}

bool path_is_connected(maps::OccupancyGrid2D& g, const std::vector<Cell>& path) {
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    bool ok = false;
    for (const auto& [v, w] : g.neighbors(path[i])) {
      (void)w;
      if (v == path[i + 1]) ok = true;
    }
    if (!ok) return false;
  }
  return true;
}

// Every reconstructed Theta* edge must be a legal straight move — validated via
// line_of_sight (paths are sparse, not neighbor-adjacent).
bool path_los_clear(maps::OccupancyGrid2D& g, const std::vector<Cell>& path) {
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    if (!g.line_of_sight(path[i], path[i + 1])) return false;
  }
  return true;
}

}  // namespace

// (a) valid/optimal path on a known map --------------------------------------

TEST(Discrete, DijkstraAndAstarAreOptimal) {
  auto g = test::make_grid({"...", "...", "..."});  // open 3x3, 8-connected
  Cell start{2, 0}, goal{0, 2};
  double optimal = 2.0 * std::sqrt(2.0);

  global_planning::DijkstraPlanner dij(cfg("dijkstra"));
  auto rd = dij.plan(g, start, goal, nullptr);
  ASSERT_TRUE(rd.success);
  EXPECT_NEAR(rd.cost, optimal, 1e-9);
  EXPECT_EQ(rd.path.front(), start);
  EXPECT_EQ(rd.path.back(), goal);

  global_planning::AstarPlanner astar(cfg("astar"));
  auto ra = astar.plan(g, start, goal, nullptr);
  ASSERT_TRUE(ra.success);
  EXPECT_NEAR(ra.cost, optimal, 1e-9);
}

TEST(Discrete, BfsMinimisesMoveCount) {
  auto g = test::make_grid({"...", "...", "..."});
  global_planning::BfsPlanner bfs(cfg("bfs"));
  auto r = bfs.plan(g, Cell{2, 0}, Cell{0, 2}, nullptr);
  ASSERT_TRUE(r.success);
  EXPECT_EQ(r.path.size(), 3u);  // 2 diagonal moves = 3 cells
  EXPECT_TRUE(path_is_connected(g, r.path));
}

// Theta* any-angle path (path 2): open grid, goal directly visible from start, so
// the straight segment beats A*'s grid-locked octile path on a non-diagonal offset.
TEST(Discrete, ThetaStarTakesAnyAngleShortcut) {
  auto g = test::make_grid({"...", "...", "..."});  // open 3x3, 8-connected
  Cell start{2, 0}, goal{0, 1};
  global_planning::ThetaStarPlanner theta(cfg("theta_star"));
  auto rt = theta.plan(g, start, goal, nullptr);
  ASSERT_TRUE(rt.success);
  EXPECT_EQ(rt.path.front(), start);
  EXPECT_EQ(rt.path.back(), goal);
  EXPECT_NEAR(rt.cost, std::sqrt(5.0), 1e-9);
  EXPECT_TRUE(path_los_clear(g, rt.path));
  global_planning::AstarPlanner astar(cfg("astar"));
  EXPECT_LT(rt.cost, astar.plan(g, start, goal, nullptr).cost);
}

// Theta* path 1: a blocker hides the goal (no direct LOS), forcing a turn. The
// any-angle path keeps an interior waypoint, every leg is LOS-clear, and it still
// beats grid-locked A*.
TEST(Discrete, ThetaStarBendsAroundObstacle) {
  auto g = test::make_grid({".....", ".....", "..#..", "..#..", "....."});
  Cell start{4, 0}, goal{0, 4};
  global_planning::ThetaStarPlanner theta(cfg("theta_star"));
  auto rt = theta.plan(g, start, goal, nullptr);
  ASSERT_TRUE(rt.success);
  EXPECT_EQ(rt.path.front(), start);
  EXPECT_EQ(rt.path.back(), goal);
  EXPECT_GE(rt.path.size(), 3u);  // bends -> at least one interior waypoint
  EXPECT_FALSE(g.line_of_sight(start, goal));  // goal genuinely hidden
  EXPECT_TRUE(path_los_clear(g, rt.path));
  global_planning::AstarPlanner astar(cfg("astar"));
  EXPECT_LT(rt.cost, astar.plan(g, start, goal, nullptr).cost);
}

// (b) no-path case ------------------------------------------------------------

TEST(Discrete, NoPathWhenWalledOff) {
  auto g = test::make_grid({".#.", ".#.", ".#."});  // middle column blocks all crossing
  Cell start{2, 0}, goal{0, 2};
  global_planning::AstarPlanner astar(cfg("astar"));
  auto r = astar.plan(g, start, goal, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.path.empty());
  global_planning::DijkstraPlanner dij(cfg("dijkstra"));
  EXPECT_FALSE(dij.plan(g, start, goal, nullptr).success);
  global_planning::BfsPlanner bfs(cfg("bfs"));
  auto rb = bfs.plan(g, start, goal, nullptr);
  EXPECT_FALSE(rb.success);
  EXPECT_TRUE(rb.path.empty());
  EXPECT_EQ(rb.cost, 0.0);
  global_planning::ThetaStarPlanner theta(cfg("theta_star"));
  auto rth = theta.plan(g, start, goal, nullptr);
  EXPECT_FALSE(rth.success);
  EXPECT_TRUE(rth.path.empty());
  EXPECT_EQ(rth.cost, 0.0);
}

// (c) param validation failure -----------------------------------------------

TEST(Discrete, BadAstarParamThrows) {
  std::string bad = test::write_temp(
      "astar.yaml",
      "algorithm: astar\ncategory: global_planning\nparams:\n"
      "  - name: heuristic_weight\n    type: float\n    default: 0.5\n    min: 1.0\n    max: 5.0\n"
      "    description: below min\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}

TEST(Discrete, BadThetaStarParamThrows) {
  std::string bad = test::write_temp(
      "theta_star.yaml",
      "algorithm: theta_star\ncategory: global_planning\nparams:\n"
      "  - name: heuristic_weight\n    type: float\n    default: 0.5\n    min: 1.0\n    max: 5.0\n"
      "    description: below min\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}
