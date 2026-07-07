#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/global_planning/search/ara_star.hpp"
#include "navigation/global_planning/search/astar.hpp"
#include "navigation/maps/loader.hpp"
#include "navigation/maps/occupancy_grid.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::Cell;

namespace {

core::ParamSet cfg(const std::string& algo) {
  return core::ParamSet::from_yaml(test::repo_path("configs/global_planning/" + algo + ".yaml"));
}

// Writes an ARA* config with the given schedule so tests can force a specific
// eps_start -> eps_final sweep without depending on the shipped defaults.
core::ParamSet ara_cfg(double eps_start, double eps_final, double eps_step) {
  std::ostringstream y;
  y << "algorithm: ara_star\ncategory: global_planning\nparams:\n"
    << "  - name: eps_start\n    type: float\n    default: " << eps_start
    << "\n    min: 1.0\n    max: 10.0\n    description: start\n"
    << "  - name: eps_final\n    type: float\n    default: " << eps_final
    << "\n    min: 1.0\n    max: 10.0\n    description: final\n"
    << "  - name: eps_step\n    type: float\n    default: " << eps_step
    << "\n    min: 0.01\n    max: 10.0\n    description: step\n"
    << "  - name: max_expansions\n    type: int\n    default: 100000\n    min: 1\n"
    << "    max: 100000000\n    description: cap\n";
  return core::ParamSet::from_yaml(test::write_temp("ara_star.yaml", y.str()));
}

maps::OccupancyGrid2D& as_grid(core::MapBase& m) {
  return *dynamic_cast<maps::OccupancyGrid2D*>(&m);
}

}  // namespace

// (a) known map: ARA*'s final (eps -> 1.0) solution equals the A* optimum ---------
TEST(AraStar, ConvergesToAstarOptimumOnMaze01) {
  auto map = maps::load_map(test::repo_path("maps/grid/maze01.yaml"), 0, 8);
  auto& grid = as_grid(*map);
  auto sc = maps::load_scenario(test::repo_path("maps/scenarios/maze01_s1.yaml"));
  Cell start = grid.world_to_cell(sc.start.x, sc.start.y);
  Cell goal = grid.world_to_cell(sc.goal.x, sc.goal.y);

  global_planning::AraStar ara(cfg("ara_star"));
  auto ra = ara.plan(grid, start, goal, nullptr);
  global_planning::AstarPlanner astar(cfg("astar"));
  auto ro = astar.plan(grid, start, goal, nullptr);

  ASSERT_TRUE(ra.success);
  EXPECT_EQ(ra.path.front(), start);
  EXPECT_EQ(ra.path.back(), goal);
  EXPECT_NEAR(ra.cost, ro.cost, 1e-9);  // final ARA* == A* optimum
}

// (b) no-path case ----------------------------------------------------------------
TEST(AraStar, NoPathWhenWalledOff) {
  auto g = test::make_grid({"..#..", "..#..", "..#..", "..#..", "..#.."});
  global_planning::AraStar ara(cfg("ara_star"));
  auto r = ara.plan(g, Cell{0, 0}, Cell{0, 4}, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.path.empty());
  EXPECT_EQ(r.cost, 0.0);
}

// (c) param validation failure ----------------------------------------------------
TEST(AraStar, BadEpsStartThrows) {
  std::string bad = test::write_temp(
      "ara_star.yaml",
      "algorithm: ara_star\ncategory: global_planning\nparams:\n"
      "  - name: eps_start\n    type: float\n    default: 0.5\n    min: 1.0\n    max: 10.0\n"
      "    description: below min\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}

// (d) anytime property: an inflated first solution, then repair to the optimum.
// wastar_greedy01 has two obstacles an inflated heuristic rounds on the costly side
// first, so the eps=3 solution is strictly worse than the eps->1 optimum.
TEST(AraStar, AnytimeSolutionsImproveToOptimum) {
  auto map = maps::load_map(test::repo_path("maps/grid/wastar_greedy01.yaml"), 0, 8);
  auto& grid = as_grid(*map);
  auto sc = maps::load_scenario(test::repo_path("maps/scenarios/wastar_greedy01_s1.yaml"));
  Cell start = grid.world_to_cell(sc.start.x, sc.start.y);
  Cell goal = grid.world_to_cell(sc.goal.x, sc.goal.y);

  // First solution only: a single weighted-A* pass at eps=3 (eps_final=3).
  global_planning::AraStar first(ara_cfg(3.0, 3.0, 1.0));
  auto rf = first.plan(grid, start, goal, nullptr);
  ASSERT_TRUE(rf.success);

  // Full sweep eps 3 -> 1, capturing the emitted anytime solutions.
  std::ostringstream os;
  core::TraceRecorder rec(os);
  global_planning::AraStar ara(ara_cfg(3.0, 1.0, 1.0));
  auto ra = ara.plan(grid, start, goal, &rec);
  ASSERT_TRUE(ra.success);

  int path_found = 0;
  std::istringstream in(os.str());
  for (std::string line; std::getline(in, line);) {
    if (line.find("\"event\":\"path_found\"") != std::string::npos) ++path_found;
  }
  EXPECT_GE(path_found, 2);  // genuinely anytime: at least one repair after the first

  global_planning::AstarPlanner astar(cfg("astar"));
  auto ro = astar.plan(grid, start, goal, nullptr);
  EXPECT_NEAR(ra.cost, ro.cost, 1e-9);  // converged to the optimum
  EXPECT_GT(rf.cost, ra.cost);          // first eps-inflated path is strictly suboptimal
}
