#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/global_planning/search/astar.hpp"
#include "navigation/global_planning/search/bfs.hpp"
#include "navigation/global_planning/search/dijkstra.hpp"
#include "navigation/global_planning/search/dstar_lite.hpp"
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

// D* Lite: reaches the goal and — on an all-free grid it never has to replan (nothing
// is ever sensed as blocked), so the executed trajectory is the freespace optimum.
TEST(Discrete, DStarLiteReachesGoalWithoutReplanOnOpenGrid) {
  auto g = test::make_grid(std::vector<std::string>(11, std::string(11, '.')));
  Cell start{7, 3}, goal{3, 7};  // interior: the sensor disk never leaves the grid
  global_planning::DStarLitePlanner dstar(cfg("dstar_lite"));
  auto rd = dstar.plan(g, start, goal, nullptr);
  ASSERT_TRUE(rd.success);
  EXPECT_EQ(rd.path.front(), start);
  EXPECT_EQ(rd.path.back(), goal);
  EXPECT_EQ(rd.stats.iterations, 0);  // no obstacle ever revealed -> no replan
  global_planning::AstarPlanner astar(cfg("astar"));
  EXPECT_NEAR(rd.cost, astar.plan(g, start, goal, nullptr).cost, 1e-9);
}

// D* Lite's defining behaviour: a wall the sensor cannot see from the start forces the
// robot to commit to a step, discover the obstacle, and incrementally repair. With a
// 1-cell sensor the vertical wall (only a bottom gap) is revealed leg by leg.
TEST(Discrete, DStarLiteReplansAroundHiddenWall) {
  std::string bad = test::write_temp(
      "dstar_lite.yaml",
      "algorithm: dstar_lite\ncategory: global_planning\nparams:\n"
      "  - name: sensor_radius\n    type: int\n    default: 1\n    min: 1\n    max: 50\n"
      "    description: one-cell sensor\n");
  auto params = core::ParamSet::from_yaml(bad);
  auto g = test::make_grid({"..#..", "..#..", "..#..", "..#..", "..#..", "..#..", "....."});
  Cell start{3, 0}, goal{3, 4};  // straight line crosses the wall at (3,2)
  global_planning::DStarLitePlanner dstar(params);
  auto rd = dstar.plan(g, start, goal, nullptr);
  ASSERT_TRUE(rd.success);
  EXPECT_EQ(rd.path.front(), start);
  EXPECT_EQ(rd.path.back(), goal);
  EXPECT_GE(rd.stats.iterations, 1);              // the hidden wall triggered a replan
  EXPECT_TRUE(path_is_connected(g, rd.path));     // every step is a legal true-grid move
  double dr = start.row - goal.row, dc = start.col - goal.col;
  EXPECT_GT(rd.cost, std::sqrt(dr * dr + dc * dc));  // genuine detour, not the straight line
  global_planning::AstarPlanner astar(cfg("astar"));
  EXPECT_GE(rd.cost, astar.plan(g, start, goal, nullptr).cost);  // A* has full knowledge
}

TEST(Discrete, DStarLiteDoesNotCutIsolatedDiagonalObstacle) {
  // Regression: the 1-cell sensor's Euclidean disk (dr^2+dc^2<=1) omits the diagonals,
  // yet the robot may step diagonally next. The immediate 8-neighbourhood must always be
  // sensed, or the robot walks onto an isolated diagonal obstacle it never detected.
  std::string yaml = test::write_temp(
      "dstar_lite.yaml",
      "algorithm: dstar_lite\ncategory: global_planning\nparams:\n"
      "  - name: sensor_radius\n    type: int\n    default: 1\n    min: 1\n    max: 50\n"
      "    description: one-cell sensor\n");
  auto g = test::make_grid({"...", ".#.", "..."});  // lone obstacle at (1,1)
  global_planning::DStarLitePlanner dstar(core::ParamSet::from_yaml(yaml));
  auto rd = dstar.plan(g, Cell{2, 0}, Cell{0, 2}, nullptr);
  ASSERT_TRUE(rd.success);
  for (const auto& c : rd.path) EXPECT_FALSE(c.row == 1 && c.col == 1);  // never through (1,1)
  EXPECT_TRUE(path_is_connected(g, rd.path));  // every step is a legal true-grid move
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
  global_planning::DStarLitePlanner dstar(cfg("dstar_lite"));
  auto rds = dstar.plan(g, start, goal, nullptr);
  EXPECT_FALSE(rds.success);
  EXPECT_TRUE(rds.path.empty());
  EXPECT_EQ(rds.cost, 0.0);
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

TEST(Discrete, BadDStarLiteParamThrows) {
  std::string bad = test::write_temp(
      "dstar_lite.yaml",
      "algorithm: dstar_lite\ncategory: global_planning\nparams:\n"
      "  - name: sensor_radius\n    type: int\n    default: 0\n    min: 1\n    max: 50\n"
      "    description: below min\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}
