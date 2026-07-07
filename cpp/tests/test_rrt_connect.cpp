#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/global_planning/sampling/rrt_connect.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::Point;

namespace {

// Open 10x10 grid (5m x 5m world).
std::vector<std::string> open_rows(int n = 10) {
  return std::vector<std::string>(static_cast<size_t>(n), std::string(static_cast<size_t>(n), '.'));
}

std::string rrt_connect_cfg() {
  return test::write_temp(
      "rrt_connect.yaml",
      "algorithm: rrt_connect\ncategory: global_planning\nparams:\n"
      "  - {name: max_iterations, type: int, default: 4000, min: 1, max: 200000, description: n}\n"
      "  - {name: step_size, type: float, default: 0.5, min: 0.01, max: 100, description: eta}\n"
      "  - {name: goal_tolerance, type: float, default: 0.3, min: 0.0, max: 100, description: t}\n"
      "  - {name: seed, type: int, default: 7, min: 0, max: 2147483647, description: s}\n");
}

}  // namespace

// (a) valid path reaching the goal --------------------------------------------

TEST(RrtConnect, FindsValidPath) {
  auto g = test::make_grid(open_rows());
  Point start{0.25, 0.25}, goal{4.75, 4.75};
  global_planning::RrtConnectPlanner p(core::ParamSet::from_yaml(rrt_connect_cfg()));
  auto r = p.plan(g, start, goal, nullptr);
  ASSERT_TRUE(r.success);
  ASSERT_GE(r.path.size(), 2u);
  // Both trees are rooted at exact states, so the spliced path pins to start/goal.
  EXPECT_NEAR(r.path.front().x, start.x, 1e-9);
  EXPECT_NEAR(r.path.front().y, start.y, 1e-9);
  EXPECT_NEAR(r.path.back().x, goal.x, 1e-9);
  EXPECT_NEAR(r.path.back().y, goal.y, 1e-9);
  for (size_t i = 0; i + 1 < r.path.size(); ++i) {
    EXPECT_TRUE(g.is_motion_valid(r.path[i], r.path[i + 1]))
        << "path segment must be collision-free";
  }
  EXPECT_GT(r.cost, 0.0);
}

// (b) no-path case: a full wall separates start from goal ---------------------

TEST(RrtConnect, NoPathAcrossFullWall) {
  auto rows = open_rows();
  rows[5] = "##########";  // impassable horizontal wall
  auto g = test::make_grid(rows);
  Point start{0.25, 0.25}, goal{4.75, 4.75};
  global_planning::RrtConnectPlanner p(core::ParamSet::from_yaml(rrt_connect_cfg()));
  auto r = p.plan(g, start, goal, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.path.empty());
  EXPECT_EQ(r.cost, 0.0);
}

// (c) param validation failure ------------------------------------------------

TEST(RrtConnect, BadStepSizeThrows) {
  std::string bad = test::write_temp(
      "rrt_connect.yaml",
      "algorithm: rrt_connect\ncategory: global_planning\nparams:\n"
      "  - {name: step_size, type: float, default: 200.0, min: 0.01, max: 100.0, description: over max}\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}
