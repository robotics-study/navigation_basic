#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/core/params.hpp"
#include "navigation/global_planning/sampling/sst.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::Point;

namespace {

// Open square grid (resolution 0.5): '.' everywhere.
std::vector<std::string> open_rows(int n) {
  return std::vector<std::string>(static_cast<size_t>(n), std::string(static_cast<size_t>(n), '.'));
}

// Full SST config; smaller iteration budgets keep the tests fast.
std::string sst_cfg(int max_iterations, const std::string& sst_star = "false") {
  return test::write_temp(
      "sst.yaml",
      "algorithm: sst\ncategory: global_planning\nparams:\n"
      "  - {name: max_iterations, type: int, default: " + std::to_string(max_iterations) +
          ", min: 1, max: 2000000, description: n}\n"
      "  - {name: goal_bias, type: float, default: 0.1, min: 0.0, max: 1.0, description: b}\n"
      "  - {name: goal_tolerance, type: float, default: 0.6, min: 0.0, max: 100, description: t}\n"
      "  - {name: delta_bn, type: float, default: 1.2, min: 0.01, max: 100, description: bn}\n"
      "  - {name: delta_s, type: float, default: 0.5, min: 0.01, max: 100, description: ds}\n"
      "  - {name: max_velocity, type: float, default: 1.5, min: 0.01, max: 100, description: v}\n"
      "  - {name: max_omega, type: float, default: 1.5, min: 0.0, max: 100, description: w}\n"
      "  - {name: prop_duration_min, type: float, default: 0.2, min: 0.001, max: 100, description: pmn}\n"
      "  - {name: prop_duration_max, type: float, default: 0.8, min: 0.001, max: 100, description: pmx}\n"
      "  - {name: sst_star, type: bool, default: " + sst_star + ", description: star}\n"
      "  - {name: seed, type: int, default: 1, min: 0, max: 2147483647, description: s}\n");
}

}  // namespace

// (a) valid, collision-free, dynamically-feasible path on an open grid -------------

TEST(Sst, FindsDynamicallyFeasiblePathOnOpenGrid) {
  auto g = test::make_grid(open_rows(20));  // 10m x 10m world
  Point start{0.5, 0.5}, goal{9.0, 9.0};
  global_planning::SstPlanner p(core::ParamSet::from_yaml(sst_cfg(30000)));
  auto r = p.plan(g, start, goal, nullptr);
  ASSERT_TRUE(r.success);
  // Goal reached within tolerance (position-only; the unicycle has no goal heading).
  EXPECT_LE(g.distance(r.path.back(), goal), 0.6);
  double max_step = 0.0;
  for (size_t i = 0; i + 1 < r.path.size(); ++i) {
    EXPECT_TRUE(g.is_state_valid(r.path[i]));
    EXPECT_TRUE(g.is_motion_valid(r.path[i], r.path[i + 1])) << "arc chord must be collision-free";
    max_step = std::max(max_step, g.distance(r.path[i], r.path[i + 1]));
  }
  // Dynamic feasibility: a densely-sampled propagated trajectory, not steer jumps.
  EXPECT_LE(max_step, 0.25);
}

// (b) no-path case: a full wall separates start from goal --------------------------

TEST(Sst, NoPathAcrossFullWall) {
  auto rows = open_rows(10);
  rows[5] = "##########";  // impassable horizontal wall
  auto g = test::make_grid(rows);
  Point start{0.25, 0.25}, goal{4.75, 4.75};
  global_planning::SstPlanner p(core::ParamSet::from_yaml(sst_cfg(500)));
  auto r = p.plan(g, start, goal, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.path.empty());
}

// (c) param validation failure: out-of-range default ------------------------------

TEST(Sst, BadGoalBiasThrows) {
  std::string bad = test::write_temp(
      "sst.yaml",
      "algorithm: sst\ncategory: global_planning\nparams:\n"
      "  - {name: goal_bias, type: float, default: 2.0, min: 0.0, max: 1.0, description: over max}\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}

// (d) witness set bounds active-node growth; SST* schedule runs --------------------

TEST(Sst, WitnessSetBoundsActiveGrowth) {
  auto g = test::make_grid(open_rows(24));  // 12m x 12m world
  Point start{0.5, 0.5}, goal{11.0, 11.0};
  global_planning::SstPlanner p(core::ParamSet::from_yaml(sst_cfg(20000)));
  auto r = p.plan(g, start, goal, nullptr);
  ASSERT_TRUE(r.success);
  EXPECT_LT(r.stats.tree_size, r.stats.iterations / 10);   // sparse active set
  EXPECT_GT(r.stats.expanded_nodes, r.stats.tree_size);    // pruning happened
  EXPECT_LE(r.stats.tree_size, 800);
}

TEST(Sst, SstStarShrinkingScheduleRuns) {
  auto g = test::make_grid(open_rows(20));
  Point start{0.5, 0.5}, goal{9.0, 9.0};
  global_planning::SstPlanner p(core::ParamSet::from_yaml(sst_cfg(4000, "true")));
  auto r = p.plan(g, start, goal, nullptr);  // must not crash
  EXPECT_EQ(r.stats.iterations, 4000);
  EXPECT_GT(r.stats.tree_size, 1);
}
