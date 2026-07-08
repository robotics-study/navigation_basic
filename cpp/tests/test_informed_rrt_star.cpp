#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/core/params.hpp"
#include "navigation/global_planning/sampling/informed_rrt_star.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::Point;

namespace {

// Open square grid (resolution 0.5): '.' everywhere.
std::vector<std::string> open_rows(int n) {
  return std::vector<std::string>(static_cast<size_t>(n), std::string(static_cast<size_t>(n), '.'));
}

// Generous anytime budget so informed sampling can tighten toward the optimum
// while the test stays fast.
std::string informed_cfg(int max_iterations) {
  return test::write_temp(
      "informed_rrt_star.yaml",
      "algorithm: informed_rrt_star\ncategory: global_planning\nparams:\n"
      "  - {name: max_iterations, type: int, default: " + std::to_string(max_iterations) +
          ", min: 1, max: 200000, description: n}\n"
      "  - {name: step_size, type: float, default: 0.5, min: 0.01, max: 100, description: eta}\n"
      "  - {name: goal_bias, type: float, default: 0.2, min: 0.0, max: 1.0, description: b}\n"
      "  - {name: goal_tolerance, type: float, default: 0.3, min: 0.0, max: 100, description: t}\n"
      "  - {name: neighbor_radius, type: float, default: 1.5, min: 0.01, max: 100, description: r}\n"
      "  - {name: radius_mode, type: enum, default: fixed, choices: [fixed, shrinking], description: m}\n"
      "  - {name: rgg_gamma, type: float, default: 2.0, min: 0.01, max: 100, description: g}\n"
      "  - {name: seed, type: int, default: 7, min: 0, max: 2147483647, description: s}\n");
}

}  // namespace

// (a) valid path, near the straight-line optimum on an open grid ---------------

TEST(InformedRrtStar, FindsNearOptimalPathOnOpenGrid) {
  auto g = test::make_grid(open_rows(12));  // 6m x 6m world
  Point start{0.25, 0.25}, goal{5.75, 5.75};
  global_planning::InformedRrtStarPlanner p(core::ParamSet::from_yaml(informed_cfg(8000)));
  auto r = p.plan(g, start, goal, nullptr);
  ASSERT_TRUE(r.success);
  // The last segment connects the tree to the exact goal.
  EXPECT_NEAR(r.path.back().x, goal.x, 1e-9);
  EXPECT_NEAR(r.path.back().y, goal.y, 1e-9);
  for (size_t i = 0; i + 1 < r.path.size(); ++i) {
    EXPECT_TRUE(g.is_motion_valid(r.path[i], r.path[i + 1])) << "path segment must be collision-free";
  }
  // Informed sampling should drive the cost close to the straight-line lower bound.
  double lower_bound = g.distance(start, goal);
  EXPECT_LE(r.cost, lower_bound * 1.15);
}

// (b) no-path case: a full wall separates start from goal ---------------------

TEST(InformedRrtStar, NoPathAcrossFullWall) {
  auto rows = open_rows(10);
  rows[5] = "##########";  // impassable horizontal wall
  auto g = test::make_grid(rows);
  Point start{0.25, 0.25}, goal{4.75, 4.75};
  global_planning::InformedRrtStarPlanner p(core::ParamSet::from_yaml(informed_cfg(400)));
  auto r = p.plan(g, start, goal, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.path.empty());
}

// (c) param validation failure: out-of-range default -------------------------

TEST(InformedRrtStar, BadGoalBiasThrows) {
  std::string bad = test::write_temp(
      "informed_rrt_star.yaml",
      "algorithm: informed_rrt_star\ncategory: global_planning\nparams:\n"
      "  - {name: goal_bias, type: float, default: 2.0, min: 0.0, max: 1.0, description: over max}\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}
