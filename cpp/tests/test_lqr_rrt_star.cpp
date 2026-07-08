#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/core/params.hpp"
#include "navigation/global_planning/sampling/lqr_rrt_star.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::Point;

namespace {

// Open square grid (resolution 0.5): '.' everywhere.
std::vector<std::string> open_rows(int n) {
  return std::vector<std::string>(static_cast<size_t>(n), std::string(static_cast<size_t>(n), '.'));
}

std::string lqr_cfg(int max_iterations, double goal_tolerance, double r_ctrl = 1.0) {
  return test::write_temp(
      "lqr_rrt_star.yaml",
      "algorithm: lqr_rrt_star\ncategory: global_planning\nparams:\n"
      "  - {name: max_iterations, type: int, default: " + std::to_string(max_iterations) +
          ", min: 1, max: 200000, description: n}\n"
      "  - {name: step_size, type: float, default: 1.5, min: 0.01, max: 100, description: e}\n"
      "  - {name: goal_bias, type: float, default: 0.2, min: 0.0, max: 1.0, description: b}\n"
      "  - {name: goal_tolerance, type: float, default: " + std::to_string(goal_tolerance) +
          ", min: 0.0, max: 100, description: t}\n"
      "  - {name: neighbor_radius, type: float, default: 2.0, min: 0.01, max: 100, description: r}\n"
      "  - {name: q_pos, type: float, default: 1.0, min: 0.001, max: 1000, description: qp}\n"
      "  - {name: q_vel, type: float, default: 1.0, min: 0.001, max: 1000, description: qv}\n"
      "  - {name: r_ctrl, type: float, default: " + std::to_string(r_ctrl) +
          ", min: 0.001, max: 1000, description: rc}\n"
      "  - {name: lqr_dt, type: float, default: 0.2, min: 0.01, max: 10, description: dt}\n"
      "  - {name: control_limit, type: float, default: 10.0, min: 0.01, max: 1000, description: u}\n"
      "  - {name: max_velocity, type: float, default: 1.5, min: 0.01, max: 100, description: v}\n"
      "  - {name: seed, type: int, default: 3, min: 0, max: 2147483647, description: s}\n");
}

}  // namespace

// (a) valid, dynamically-feasible, collision-free path on an open grid --------

TEST(LqrRrtStar, FindsFeasiblePathOnOpenGrid) {
  auto g = test::make_grid(open_rows(20));  // 10m x 10m world
  Point start{0.75, 0.75}, goal{9.0, 9.0};
  global_planning::LqrRrtStarPlanner p(core::ParamSet::from_yaml(lqr_cfg(3000, 1.5)));
  auto r = p.plan(g, start, goal, nullptr);
  ASSERT_TRUE(r.success);
  // The LQR feedback regulates onto the goal rest-state; the projected endpoint is
  // within tolerance of the goal position.
  EXPECT_LE(std::hypot(r.path.back().x - goal.x, r.path.back().y - goal.y), 1.5);
  // The propagated double-integrator trajectory must be collision-free throughout.
  for (size_t i = 0; i + 1 < r.path.size(); ++i) {
    EXPECT_TRUE(g.is_motion_valid(r.path[i], r.path[i + 1])) << "path segment must be collision-free";
  }
  EXPECT_GT(r.cost, 0.0);
}

// (b) no-path case: a full wall separates start from goal --------------------

TEST(LqrRrtStar, NoPathAcrossFullWall) {
  auto rows = open_rows(10);
  rows[5] = "##########";  // impassable horizontal wall
  auto g = test::make_grid(rows);
  Point start{0.25, 0.25}, goal{4.75, 4.75};
  global_planning::LqrRrtStarPlanner p(core::ParamSet::from_yaml(lqr_cfg(400, 1.0)));
  auto r = p.plan(g, start, goal, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.path.empty());
}

// (c) param validation failure: out-of-range default -------------------------

TEST(LqrRrtStar, BadControlWeightThrows) {
  std::string bad = test::write_temp(
      "lqr_rrt_star.yaml",
      "algorithm: lqr_rrt_star\ncategory: global_planning\nparams:\n"
      "  - {name: r_ctrl, type: float, default: 0.0, min: 0.001, max: 1000, description: under min}\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}

// (d) the extension heuristics are genuinely LQR-derived ---------------------

// A heavier control weight R yields a different Riccati solution, hence a different
// steering — so the planned trajectory changes. Same map/seed/iterations, only r_ctrl
// differs: proves the metric/steer are derived from the LQR, not fixed (Perez 2012).
TEST(LqrRrtStar, ControlWeightChangesThePath) {
  auto g = test::make_grid(open_rows(20));
  Point start{0.75, 0.75}, goal{9.0, 9.0};
  global_planning::LqrRrtStarPlanner cheap(core::ParamSet::from_yaml(lqr_cfg(2000, 1.5, 0.2)));
  global_planning::LqrRrtStarPlanner dear(core::ParamSet::from_yaml(lqr_cfg(2000, 1.5, 50.0)));
  auto rc = cheap.plan(g, start, goal, nullptr);
  auto rd = dear.plan(g, start, goal, nullptr);
  ASSERT_TRUE(rc.success);
  ASSERT_TRUE(rd.success);
  // Different LQR gains => different realised trajectories. Compare path content
  // (not just length, which can coincide) so the assertion is not fragile.
  bool differ = rc.path.size() != rd.path.size();
  for (size_t i = 0; !differ && i < rc.path.size(); ++i) {
    differ = std::abs(rc.path[i].x - rd.path[i].x) > 1e-9 ||
             std::abs(rc.path[i].y - rd.path[i].y) > 1e-9;
  }
  EXPECT_TRUE(differ);
}
