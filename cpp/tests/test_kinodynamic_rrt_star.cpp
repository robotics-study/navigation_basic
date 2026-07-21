#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/core/params.hpp"
#include "navigation/global_planning/sampling/kinodynamic_rrt_star.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::Point;

namespace {

// Open square grid (resolution 0.5): '.' everywhere.
std::vector<std::string> open_rows(int n) {
  return std::vector<std::string>(static_cast<size_t>(n), std::string(static_cast<size_t>(n), '.'));
}

std::string kino_cfg(int max_iterations, double goal_tolerance) {
  return test::write_temp(
      "kinodynamic_rrt_star.yaml",
      "algorithm: kinodynamic_rrt_star\ncategory: global_planning\nparams:\n"
      "  - {name: max_iterations, type: int, default: " + std::to_string(max_iterations) +
          ", min: 1, max: 200000, description: n}\n"
      "  - {name: goal_bias, type: float, default: 0.2, min: 0.0, max: 1.0, description: b}\n"
      "  - {name: goal_tolerance, type: float, default: " + std::to_string(goal_tolerance) +
          ", min: 0.0, max: 100, description: t}\n"
      "  - {name: neighbor_radius, type: float, default: 2.0, min: 0.01, max: 100, description: r}\n"
      "  - {name: control_weight, type: float, default: 1.0, min: 0.001, max: 1000, description: w}\n"
      "  - {name: max_velocity, type: float, default: 1.5, min: 0.01, max: 100, description: v}\n"
      "  - {name: footprint_radius, type: float, default: 0.15, min: 0.01, max: 20, description: fp}\n"
      "  - {name: seed, type: int, default: 3, min: 0, max: 2147483647, description: s}\n");
}

}  // namespace

// (a) valid, dynamically-feasible, collision-free path on an open grid --------

TEST(KinodynamicRrtStar, FindsFeasiblePathOnOpenGrid) {
  auto g = test::make_grid(open_rows(12));  // 6m x 6m world
  Point start{0.75, 0.75}, goal{5.25, 5.25};
  global_planning::KinodynamicRrtStarPlanner p(core::ParamSet::from_yaml(kino_cfg(4000, 1.5)));
  auto r = p.plan(g, start, goal, nullptr);
  ASSERT_TRUE(r.success);
  // Optimal steering arrives at the goal rest-state; the projected endpoint is within
  // tolerance of the goal position.
  EXPECT_LE(std::hypot(r.path.back().x - goal.x, r.path.back().y - goal.y), 1.5);
  // The propagated double-integrator trajectory must be collision-free throughout.
  for (size_t i = 0; i + 1 < r.path.size(); ++i) {
    EXPECT_TRUE(g.is_motion_valid(r.path[i], r.path[i + 1])) << "path segment must be collision-free";
  }
  EXPECT_GT(r.cost, 0.0);
}

// (b) no-path case: a full wall separates start from goal --------------------

TEST(KinodynamicRrtStar, NoPathAcrossFullWall) {
  auto rows = open_rows(10);
  rows[5] = "##########";  // impassable horizontal wall
  auto g = test::make_grid(rows);
  Point start{0.25, 0.25}, goal{4.75, 4.75};
  global_planning::KinodynamicRrtStarPlanner p(core::ParamSet::from_yaml(kino_cfg(300, 1.0)));
  auto r = p.plan(g, start, goal, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.path.empty());
}

// (c) param validation failure: out-of-range default -------------------------

TEST(KinodynamicRrtStar, BadControlWeightThrows) {
  std::string bad = test::write_temp(
      "kinodynamic_rrt_star.yaml",
      "algorithm: kinodynamic_rrt_star\ncategory: global_planning\nparams:\n"
      "  - {name: control_weight, type: float, default: 0.0, min: 0.001, max: 1000, description: under min}\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}
