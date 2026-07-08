#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/global_planning/sampling/abit_star.hpp"
#include "navigation/global_planning/sampling/rrt.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::Point;

namespace {

std::vector<std::string> open_rows(int n = 10) {
  return std::vector<std::string>(static_cast<size_t>(n), std::string(static_cast<size_t>(n), '.'));
}

// Small batch budget over the open 5m x 5m grid: reliably connects start/goal
// while staying fast. `inflation` lets a test hold the schedule at a chosen value.
std::string abit_star_cfg(double inflation = 10.0, int max_batches = 5) {
  return test::write_temp(
      "abit_star.yaml",
      "algorithm: abit_star\ncategory: global_planning\nparams:\n"
      "  - {name: batch_size, type: int, default: 120, min: 1, max: 100000, description: b}\n"
      "  - {name: max_batches, type: int, default: " +
          std::to_string(max_batches) +
          ", min: 1, max: 10000, description: mb}\n"
          "  - {name: gamma, type: float, default: 30.0, min: 0.01, max: 1000, description: g}\n"
          "  - {name: inflation_factor, type: float, default: " +
          std::to_string(inflation) +
          ", min: 1.0, max: 1000000, description: ei}\n"
          "  - {name: inflation_final, type: float, default: 1.0, min: 1.0, max: 1000000, description: eif}\n"
          "  - {name: truncation_factor, type: float, default: 2.0, min: 1.0, max: 1000000, description: et}\n"
          "  - {name: seed, type: int, default: 7, min: 0, max: 2147483647, description: s}\n");
}

std::string rrt_cfg() {
  return test::write_temp("rrt.yaml",
                          "algorithm: rrt\ncategory: global_planning\nparams:\n"
                          "  - {name: max_iterations, type: int, default: 4000, min: 1, max: 200000, description: n}\n"
                          "  - {name: step_size, type: float, default: 0.5, min: 0.01, max: 100, description: eta}\n"
                          "  - {name: goal_bias, type: float, default: 0.2, min: 0.0, max: 1.0, description: b}\n"
                          "  - {name: goal_tolerance, type: float, default: 0.3, min: 0.0, max: 100, description: t}\n"
                          "  - {name: seed, type: int, default: 7, min: 0, max: 2147483647, description: s}\n");
}

double straight_line(const Point& a, const Point& b) {
  double dx = a.x - b.x, dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

// (a) valid, near-optimal path on an open grid --------------------------------

TEST(AbitStar, FindsNearOptimalPath) {
  auto g = test::make_grid(open_rows());
  Point start{0.25, 0.25}, goal{4.75, 4.75};
  global_planning::AbitStarPlanner p(core::ParamSet::from_yaml(abit_star_cfg()));
  auto r = p.plan(g, start, goal, nullptr);
  ASSERT_TRUE(r.success);
  ASSERT_GE(r.path.size(), 2u);
  EXPECT_NEAR(r.path.front().x, start.x, 1e-9);
  EXPECT_NEAR(r.path.front().y, start.y, 1e-9);
  // start/goal are permanent samples, so the path pins to the exact goal.
  EXPECT_NEAR(r.path.back().x, goal.x, 1e-9);
  EXPECT_NEAR(r.path.back().y, goal.y, 1e-9);
  for (size_t i = 0; i + 1 < r.path.size(); ++i) {
    EXPECT_TRUE(g.is_motion_valid(r.path[i], r.path[i + 1])) << "segment must be collision-free";
  }
  // The ε schedule relaxes to 1 on the last batch, so ABIT* recovers BIT*'s
  // admissible search and stays close to the straight-line lower bound.
  EXPECT_LE(r.cost, straight_line(start, goal) * 1.15);
}

// ABIT* is asymptotically optimal; a plain RRT returns the first feasible path.
// On the same scenario+budget ABIT*'s path must be no longer than RRT's.
TEST(AbitStar, NoWorseThanPlainRrt) {
  auto g = test::make_grid(open_rows());
  Point start{0.25, 0.25}, goal{4.75, 4.75};
  global_planning::AbitStarPlanner abit(core::ParamSet::from_yaml(abit_star_cfg()));
  auto ra = abit.plan(g, start, goal, nullptr);
  global_planning::RrtPlanner rrt(core::ParamSet::from_yaml(rrt_cfg()));
  auto rr = rrt.plan(g, start, goal, nullptr);
  ASSERT_TRUE(ra.success);
  ASSERT_TRUE(rr.success);
  EXPECT_LE(ra.cost, rr.cost + 1e-6);
}

// inflation > 1 anytime property: a large ε_infl held across a single batch (no
// de-inflation) still returns a feasible, collision-free path — inflation only
// reorders work, never blocks a solution.
TEST(AbitStar, AnytimeUnderHeavyInflation) {
  auto g = test::make_grid(open_rows());
  Point start{0.25, 0.25}, goal{4.75, 4.75};
  global_planning::AbitStarPlanner p(core::ParamSet::from_yaml(abit_star_cfg(1000.0, 1)));
  auto r = p.plan(g, start, goal, nullptr);
  ASSERT_TRUE(r.success);
  EXPECT_NEAR(r.path.back().x, goal.x, 1e-9);
  EXPECT_NEAR(r.path.back().y, goal.y, 1e-9);
  for (size_t i = 0; i + 1 < r.path.size(); ++i) {
    EXPECT_TRUE(g.is_motion_valid(r.path[i], r.path[i + 1])) << "segment must be collision-free";
  }
}

// (b) no-path case: a full wall separates start from goal ---------------------

TEST(AbitStar, NoPathAcrossFullWall) {
  auto rows = open_rows();
  rows[5] = "##########";  // impassable horizontal wall
  auto g = test::make_grid(rows);
  Point start{0.25, 0.25}, goal{4.75, 4.75};
  global_planning::AbitStarPlanner p(core::ParamSet::from_yaml(abit_star_cfg()));
  auto r = p.plan(g, start, goal, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.path.empty());
  EXPECT_EQ(r.cost, 0.0);
}

// (c) param validation failure ------------------------------------------------

TEST(AbitStar, BadInflationThrows) {
  std::string bad = test::write_temp(
      "abit_star.yaml",
      "algorithm: abit_star\ncategory: global_planning\nparams:\n"
      "  - {name: inflation_factor, type: float, default: 0.5, min: 1.0, max: 1000000, description: under min}\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}
