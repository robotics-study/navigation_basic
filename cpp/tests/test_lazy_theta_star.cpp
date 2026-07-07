#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/global_planning/search/astar.hpp"
#include "navigation/global_planning/search/lazy_theta_star.hpp"
#include "navigation/global_planning/search/theta_star.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::Cell;

namespace {

core::ParamSet cfg(const std::string& algo) {
  return core::ParamSet::from_yaml(test::repo_path("configs/global_planning/" + algo + ".yaml"));
}

// Every reconstructed edge must be a legal straight move — validated via
// line_of_sight (any-angle paths are sparse, not neighbor-adjacent).
bool path_los_clear(maps::OccupancyGrid2D& g, const std::vector<Cell>& path) {
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    if (!g.line_of_sight(path[i], path[i + 1])) return false;
  }
  return true;
}

}  // namespace

// (a) any-angle path on a known map: goal directly visible from start, so the
// deferred check confirms the optimistic straight segment. Cost is the Euclidean
// distance and matches eager Theta* on this instance, and beats grid-locked A*.
TEST(LazyThetaStar, TakesAnyAngleShortcut) {
  auto g = test::make_grid({"...", "...", "..."});  // open 3x3, 8-connected
  Cell start{2, 0}, goal{0, 1};
  global_planning::LazyThetaStarPlanner lazy(cfg("lazy_theta_star"));
  auto rl = lazy.plan(g, start, goal, nullptr);
  ASSERT_TRUE(rl.success);
  EXPECT_EQ(rl.path.front(), start);
  EXPECT_EQ(rl.path.back(), goal);
  EXPECT_NEAR(rl.cost, std::sqrt(5.0), 1e-9);
  EXPECT_TRUE(path_los_clear(g, rl.path));
  global_planning::ThetaStarPlanner theta(cfg("theta_star"));
  EXPECT_NEAR(rl.cost, theta.plan(g, start, goal, nullptr).cost, 1e-9);
  global_planning::AstarPlanner astar(cfg("astar"));
  EXPECT_LT(rl.cost, astar.plan(g, start, goal, nullptr).cost);
}

// (a) the lazy-repair path: a blocker hides the goal, so some vertex's optimistic
// parent fails its deferred line-of-sight check and is repaired to a grid neighbour.
// Every leg stays LOS-clear and the cost matches eager Theta*.
TEST(LazyThetaStar, BendsAroundObstacleMatchingTheta) {
  auto g = test::make_grid({".....", ".....", "..#..", "..#..", "....."});
  Cell start{4, 0}, goal{0, 4};
  global_planning::LazyThetaStarPlanner lazy(cfg("lazy_theta_star"));
  auto rl = lazy.plan(g, start, goal, nullptr);
  ASSERT_TRUE(rl.success);
  EXPECT_EQ(rl.path.front(), start);
  EXPECT_EQ(rl.path.back(), goal);
  EXPECT_GE(rl.path.size(), 3u);  // bends -> at least one interior waypoint
  EXPECT_FALSE(g.line_of_sight(start, goal));  // goal genuinely hidden
  EXPECT_TRUE(path_los_clear(g, rl.path));
  global_planning::ThetaStarPlanner theta(cfg("theta_star"));
  EXPECT_NEAR(rl.cost, theta.plan(g, start, goal, nullptr).cost, 1e-9);
}

// (b) no-path case: a fully occupied middle column separates start from goal.
TEST(LazyThetaStar, NoPathWhenWalledOff) {
  auto g = test::make_grid({".#.", ".#.", ".#."});  // middle column blocks all crossing
  Cell start{2, 0}, goal{0, 2};
  global_planning::LazyThetaStarPlanner lazy(cfg("lazy_theta_star"));
  auto rl = lazy.plan(g, start, goal, nullptr);
  EXPECT_FALSE(rl.success);
  EXPECT_TRUE(rl.path.empty());
  EXPECT_EQ(rl.cost, 0.0);
}

// (c) param validation failure: heuristic_weight >= 1.0 is declared, so a below-min
// default must fail at load time before it can ever reach plan().
TEST(LazyThetaStar, BadParamThrows) {
  std::string bad = test::write_temp(
      "lazy_theta_star.yaml",
      "algorithm: lazy_theta_star\ncategory: global_planning\nparams:\n"
      "  - name: heuristic_weight\n    type: float\n    default: 0.5\n    min: 1.0\n    max: 5.0\n"
      "    description: below min\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}
