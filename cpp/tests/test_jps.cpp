#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/global_planning/search/astar.hpp"
#include "navigation/global_planning/search/jps.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::Cell;

namespace {

core::ParamSet cfg(const std::string& algo) {
  return core::ParamSet::from_yaml(test::repo_path("configs/global_planning/" + algo + ".yaml"));
}

bool path_is_connected(maps::OccupancyGrid2D& g, const std::vector<Cell>& path) {
  // Every reported cell must be a legal 8-connected move from the previous — proves
  // JPS's interpolated staircase never cuts a corner the grid forbids.
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

}  // namespace

// (a) optimal path on a known map, with fewer expansions than A* -----------------

TEST(Jps, MatchesAstarOptimumWithFewerExpansions) {
  auto g = test::make_grid(std::vector<std::string>(9, std::string(9, '.')));  // open 9x9
  Cell start{8, 0}, goal{0, 8};

  global_planning::AstarPlanner astar(cfg("astar"));
  auto ra = astar.plan(g, start, goal, nullptr);
  global_planning::JpsPlanner jps(cfg("jps"));
  auto rj = jps.plan(g, start, goal, nullptr);

  ASSERT_TRUE(ra.success);
  ASSERT_TRUE(rj.success);
  EXPECT_NEAR(rj.cost, ra.cost, 1e-9);        // JPS returns the 8-connected optimum
  EXPECT_NEAR(rj.cost, 8.0 * std::sqrt(2.0), 1e-9);
  EXPECT_EQ(rj.path.front(), start);
  EXPECT_EQ(rj.path.back(), goal);
  EXPECT_TRUE(path_is_connected(g, rj.path));  // interpolated staircase is corner-cut free
  EXPECT_LT(rj.stats.expanded_nodes, ra.stats.expanded_nodes);  // symmetry pruning pays off
}

TEST(Jps, BendsOptimallyAroundObstacle) {
  // A wall with a single bottom gap: JPS must match A*'s grid optimum, not the
  // straight-line lower bound.
  auto g = test::make_grid({"..#..", "..#..", "..#..", "..#..", "....."});
  Cell start{0, 0}, goal{0, 4};
  global_planning::AstarPlanner astar(cfg("astar"));
  global_planning::JpsPlanner jps(cfg("jps"));
  auto ra = astar.plan(g, start, goal, nullptr);
  auto rj = jps.plan(g, start, goal, nullptr);
  ASSERT_TRUE(rj.success);
  EXPECT_NEAR(rj.cost, ra.cost, 1e-9);
  EXPECT_EQ(rj.path.front(), start);
  EXPECT_EQ(rj.path.back(), goal);
  EXPECT_TRUE(path_is_connected(g, rj.path));
}

// (b) no-path case ---------------------------------------------------------------

TEST(Jps, NoPathWhenWalledOff) {
  auto g = test::make_grid({".#.", ".#.", ".#."});  // middle column blocks all crossing
  Cell start{2, 0}, goal{0, 2};
  global_planning::JpsPlanner jps(cfg("jps"));
  auto r = jps.plan(g, start, goal, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.path.empty());
  EXPECT_EQ(r.cost, 0.0);
}

// (c) param validation failure ---------------------------------------------------

TEST(Jps, BadParamThrows) {
  // A jps config carrying an out-of-range default must fail at load time (the param
  // validation contract), so a bad value can never reach plan().
  std::string bad = test::write_temp(
      "jps.yaml",
      "algorithm: jps\ncategory: global_planning\nparams:\n"
      "  - name: heuristic_weight\n    type: float\n    default: 0.5\n    min: 1.0\n    max: 5.0\n"
      "    description: below min\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}
