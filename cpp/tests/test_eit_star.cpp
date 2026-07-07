#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/global_planning/sampling/eit_star.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::Point;

namespace {

// Open n x n grid (resolution 0.5 -> n/2 m square); '.' everywhere.
std::vector<std::string> open_rows(int n = 10) {
  return std::vector<std::string>(static_cast<size_t>(n), std::string(static_cast<size_t>(n), '.'));
}

// Reduced batch budget keeps the anytime planner fast while still exercising real
// behavior (informed batches, dual reverse Dijkstra, lexicographic forward search).
std::string eit_cfg() {
  return test::write_temp(
      "eit_star.yaml",
      "algorithm: eit_star\ncategory: global_planning\nparams:\n"
      "  - {name: batch_size, type: int, default: 120, min: 1, max: 100000, description: b}\n"
      "  - {name: max_batches, type: int, default: 5, min: 1, max: 10000, description: mb}\n"
      "  - {name: gamma, type: float, default: 30.0, min: 0.01, max: 1000, description: g}\n"
      "  - {name: step_size, type: float, default: 0.5, min: 0.01, max: 100, description: ss}\n"
      "  - {name: seed, type: int, default: 7, min: 0, max: 2147483647, description: s}\n");
}

}  // namespace

// (a) valid, near-optimal path on open space ----------------------------------

TEST(EitStar, FindsValidNearOptimalPath) {
  auto g = test::make_grid(open_rows());
  Point start{0.25, 0.25}, goal{4.75, 4.75};
  global_planning::EitStarPlanner p(core::ParamSet::from_yaml(eit_cfg()));
  auto r = p.plan(g, start, goal, nullptr);
  ASSERT_TRUE(r.success);
  // start/goal are permanent sample nodes, so the path pins to them exactly.
  ASSERT_GE(r.path.size(), 2u);
  EXPECT_NEAR(r.path.front().x, start.x, 1e-9);
  EXPECT_NEAR(r.path.front().y, start.y, 1e-9);
  EXPECT_NEAR(r.path.back().x, goal.x, 1e-9);
  EXPECT_NEAR(r.path.back().y, goal.y, 1e-9);
  for (size_t i = 0; i + 1 < r.path.size(); ++i) {
    EXPECT_TRUE(g.is_motion_valid(r.path[i], r.path[i + 1])) << "path segment must be collision-free";
  }
  // On an obstacle-free grid an asymptotically optimal batch planner returns close
  // to the straight-line lower bound. 1.3x leaves margin for RGG discretization
  // (samples are not aligned to the straight line) at this modest sample budget.
  double lower_bound = g.distance(start, goal);
  EXPECT_LE(r.cost, lower_bound * 1.3);
}

// (b) no-path case: a full wall separates start from goal ---------------------

TEST(EitStar, NoPathAcrossFullWall) {
  auto rows = open_rows();
  rows[5] = "##########";  // impassable horizontal wall
  auto g = test::make_grid(rows);
  Point start{0.25, 0.25}, goal{4.75, 4.75};
  global_planning::EitStarPlanner p(core::ParamSet::from_yaml(eit_cfg()));
  auto r = p.plan(g, start, goal, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.path.empty());
  EXPECT_EQ(r.cost, 0.0);
}

// (c) param validation failure: out-of-range default throws at load ----------

TEST(EitStar, BadStepSizeThrows) {
  std::string bad = test::write_temp(
      "eit_star.yaml",
      "algorithm: eit_star\ncategory: global_planning\nparams:\n"
      "  - {name: step_size, type: float, default: 0.0, min: 0.01, max: 100, description: below min}\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}
