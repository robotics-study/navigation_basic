#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/global_planning/sampling/fcit_star.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::Point;

namespace {

std::vector<std::string> open_rows(int n = 10) {
  return std::vector<std::string>(static_cast<size_t>(n), std::string(static_cast<size_t>(n), '.'));
}

// Modest budget: FCIT*'s candidate graph is fully connected (O(n^2) edges), so
// keep the accumulated sample count small enough to run fast.
std::string fcit_star_cfg() {
  return test::write_temp("fcit_star.yaml",
                          "algorithm: fcit_star\ncategory: global_planning\nparams:\n"
                          "  - {name: batch_size, type: int, default: 60, min: 1, max: 100000, description: b}\n"
                          "  - {name: max_batches, type: int, default: 3, min: 1, max: 10000, description: mb}\n"
                          "  - {name: seed, type: int, default: 7, min: 0, max: 2147483647, description: s}\n");
}

}  // namespace

// (a) valid, near-optimal path reaching the goal ------------------------------

TEST(FcitStar, FindsNearOptimalPath) {
  auto g = test::make_grid(open_rows());
  Point start{0.25, 0.25}, goal{4.75, 4.75};
  global_planning::FcitStarPlanner p(core::ParamSet::from_yaml(fcit_star_cfg()));
  auto r = p.plan(g, start, goal, nullptr);
  ASSERT_TRUE(r.success);
  ASSERT_GE(r.path.size(), 2u);
  // start/goal are permanent samples, so the incumbent pins to them exactly.
  EXPECT_NEAR(r.path.front().x, start.x, 1e-9);
  EXPECT_NEAR(r.path.front().y, start.y, 1e-9);
  EXPECT_NEAR(r.path.back().x, goal.x, 1e-9);
  EXPECT_NEAR(r.path.back().y, goal.y, 1e-9);
  for (size_t i = 0; i + 1 < r.path.size(); ++i) {
    EXPECT_TRUE(g.is_motion_valid(r.path[i], r.path[i + 1])) << "segment must be collision-free";
  }
  // On the obstacle-free grid the fully connected graph holds the direct
  // start-goal edge, so an asymptotically optimal search returns essentially the
  // straight-line lower bound. 1.3x leaves ample slack for the grid metric.
  double lower_bound = g.distance(start, goal);
  EXPECT_LE(r.cost, lower_bound * 1.3);
}

// (b) no-path case: a full wall separates start from goal ---------------------

TEST(FcitStar, NoPathAcrossFullWall) {
  auto rows = open_rows();
  rows[5] = "##########";  // impassable horizontal wall
  auto g = test::make_grid(rows);
  Point start{0.25, 0.25}, goal{4.75, 4.75};
  global_planning::FcitStarPlanner p(core::ParamSet::from_yaml(fcit_star_cfg()));
  auto r = p.plan(g, start, goal, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.path.empty());
  EXPECT_EQ(r.cost, 0.0);
}

// (c) param validation failure ------------------------------------------------

TEST(FcitStar, OutOfRangeParamThrows) {
  std::string bad = test::write_temp(
      "fcit_star.yaml",
      "algorithm: fcit_star\ncategory: global_planning\nparams:\n"
      "  - {name: batch_size, type: int, default: 0, min: 1, max: 100000, description: below min}\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}
