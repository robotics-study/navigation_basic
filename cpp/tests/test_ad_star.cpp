#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/global_planning/search/ad_star.hpp"
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

// Writes an AD* config with an explicit schedule + sensor radius so tests can force a
// specific ε sweep / sensing horizon without depending on the shipped defaults.
core::ParamSet ad_cfg(double eps_start, double eps_final, double eps_step, int sensor_radius) {
  std::ostringstream y;
  y << "algorithm: ad_star\ncategory: global_planning\nparams:\n"
    << "  - name: eps_start\n    type: float\n    default: " << eps_start
    << "\n    min: 1.0\n    max: 10.0\n    description: start\n"
    << "  - name: eps_final\n    type: float\n    default: " << eps_final
    << "\n    min: 1.0\n    max: 10.0\n    description: final\n"
    << "  - name: eps_step\n    type: float\n    default: " << eps_step
    << "\n    min: 0.01\n    max: 10.0\n    description: step\n"
    << "  - name: sensor_radius\n    type: int\n    default: " << sensor_radius
    << "\n    min: 1\n    max: 50\n    description: radius\n"
    << "  - name: max_expansions\n    type: int\n    default: 100000\n    min: 1\n"
    << "    max: 100000000\n    description: cap\n";
  return core::ParamSet::from_yaml(test::write_temp("ad_star.yaml", y.str()));
}

maps::OccupancyGrid2D& as_grid(core::MapBase& m) {
  return *dynamic_cast<maps::OccupancyGrid2D*>(&m);
}

// Every consecutive pair is a legal move under GROUND TRUTH (grid.neighbors), proving
// the executed trajectory never crosses a real obstacle.
bool valid_trajectory(const maps::OccupancyGrid2D& grid, const std::vector<Cell>& path) {
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    bool ok = false;
    for (const auto& [c, w] : grid.neighbors(path[i])) {
      (void)w;
      if (c == path[i + 1]) {
        ok = true;
        break;
      }
    }
    if (!ok) return false;
  }
  return true;
}

// Same field as the ARA* anytime fixture but start/goal swapped so the two obstacles
// sit near the robot: AD*'s backward search rushes toward the robot under the inflated
// key and rounds them the costly way first, then repairs to the optimum.
const std::vector<std::string> kField = {
    ".............", "..........#..", "...........#.", ".............", ".............",
    ".............", ".............", ".............", ".............", ".............",
    ".............", ".............", ".............", "............."};
const Cell kFieldStart{0, 11};
const Cell kFieldGoal{11, 12};

}  // namespace

// (a) static map: the executed trajectory equals the omniscient A* optimum -----------
TEST(AdStar, StaticTrajectoryMatchesAstarOptimumOnMaze01) {
  auto map = maps::load_map(test::repo_path("maps/grid/maze01.yaml"), 0, 8);
  auto& grid = as_grid(*map);
  auto sc = maps::load_scenario(test::repo_path("maps/scenarios/maze01_s1.yaml"));
  Cell start = grid.world_to_cell(sc.start.x, sc.start.y);
  Cell goal = grid.world_to_cell(sc.goal.x, sc.goal.y);

  global_planning::AdStarPlanner ad(cfg("ad_star"));
  auto ra = ad.plan(grid, start, goal, nullptr);
  global_planning::AstarPlanner astar(cfg("astar"));
  auto ro = astar.plan(grid, start, goal, nullptr);

  ASSERT_TRUE(ra.success);
  EXPECT_EQ(ra.path.front(), start);
  EXPECT_EQ(ra.path.back(), goal);
  EXPECT_TRUE(valid_trajectory(grid, ra.path));
  EXPECT_NEAR(ra.cost, ro.cost, 1e-9);
}

// (b) replan: a valid, strictly longer detour once the C-trap's back wall is sensed ---
TEST(AdStar, ReplansValidDetourWhenTrapRevealed) {
  auto map = maps::load_map(test::repo_path("maps/grid/dstar_trap01.yaml"), 0, 8);
  auto& grid = as_grid(*map);
  auto sc = maps::load_scenario(test::repo_path("maps/scenarios/dstar_trap01_s1.yaml"));
  Cell start = grid.world_to_cell(sc.start.x, sc.start.y);
  Cell goal = grid.world_to_cell(sc.goal.x, sc.goal.y);

  global_planning::AdStarPlanner ad(cfg("ad_star"));
  auto ra = ad.plan(grid, start, goal, nullptr);
  global_planning::AstarPlanner astar(cfg("astar"));
  auto ro = astar.plan(grid, start, goal, nullptr);

  ASSERT_TRUE(ra.success);
  EXPECT_EQ(ra.path.front(), start);
  EXPECT_EQ(ra.path.back(), goal);
  EXPECT_TRUE(valid_trajectory(grid, ra.path));
  EXPECT_GE(ra.stats.iterations, 1);   // replanned after sensing the trap
  EXPECT_GT(ra.cost, ro.cost + 1e-9);  // the detour is genuinely longer than omniscient A*
}

// (c) no-path case -------------------------------------------------------------------
TEST(AdStar, NoPathWhenWalledOff) {
  auto g = test::make_grid({"..#..", "..#..", "..#..", "..#..", "..#.."});
  global_planning::AdStarPlanner ad(cfg("ad_star"));
  auto r = ad.plan(g, Cell{0, 0}, Cell{0, 4}, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.path.empty());
  EXPECT_EQ(r.cost, 0.0);
}

// (d) param validation failure -------------------------------------------------------
TEST(AdStar, BadEpsStartThrows) {
  std::string bad = test::write_temp(
      "ad_star.yaml",
      "algorithm: ad_star\ncategory: global_planning\nparams:\n"
      "  - name: eps_start\n    type: float\n    default: 0.5\n    min: 1.0\n    max: 10.0\n"
      "    description: below min\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}

// (e) anytime: an inflated first trajectory, then repair to the optimum. Large sensor
// radius makes belief == truth from the start, isolating the ε sweep to a static
// improvement so the first (ε fixed high) executed trajectory is strictly suboptimal.
TEST(AdStar, AnytimeSolutionsImproveToOptimum) {
  auto grid = test::make_grid(kField);

  // First solution only: ε pinned at 4 (eps_final = 4), so the robot moves on a
  // suboptimal frozen g.
  global_planning::AdStarPlanner first(ad_cfg(4.0, 4.0, 1.0, 50));
  auto rf = first.plan(grid, kFieldStart, kFieldGoal, nullptr);
  ASSERT_TRUE(rf.success);

  // Full sweep ε 4 -> 1, capturing the emitted anytime solutions.
  std::ostringstream os;
  core::TraceRecorder rec(os);
  global_planning::AdStarPlanner ad(ad_cfg(4.0, 1.0, 0.5, 50));
  auto ra = ad.plan(grid, kFieldStart, kFieldGoal, &rec);
  ASSERT_TRUE(ra.success);

  int path_found = 0;
  std::istringstream in(os.str());
  for (std::string line; std::getline(in, line);) {
    if (line.find("\"event\":\"path_found\"") != std::string::npos) ++path_found;
  }
  EXPECT_GE(path_found, 2);  // genuinely anytime: at least one repair after the first

  global_planning::AstarPlanner astar(cfg("astar"));
  auto ro = astar.plan(grid, kFieldStart, kFieldGoal, nullptr);
  EXPECT_NEAR(ra.cost, ro.cost, 1e-9);  // converged to the optimum
  EXPECT_GT(rf.cost, ra.cost);          // first ε-inflated trajectory is strictly suboptimal
}
