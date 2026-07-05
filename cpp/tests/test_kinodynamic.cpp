#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/global_planning/search/hybrid_astar.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::Footprint;
using core::Pose;

namespace {

// Hybrid A* is the first Pose-state / SE2CollisionSpace planner — a third test family
// paralleling test_discrete (Cell) / test_sampling (Point).

core::ParamSet cfg() {
  return core::ParamSet::from_yaml(test::repo_path("configs/global_planning/hybrid_astar.yaml"));
}

std::vector<std::string> open_rows(int n) {
  return std::vector<std::string>(static_cast<size_t>(n), std::string(static_cast<size_t>(n), '.'));
}

double wrap_pi(double a) {
  const double two_pi = 2.0 * M_PI;
  double d = std::fmod(a + M_PI, two_pi);
  if (d < 0.0) d += two_pi;
  return d - M_PI;
}

// A full 11-param config with overridable reverse + turn radius, so a written yaml
// satisfies every params.get_* the planner reads.
std::string reverse_cfg() {
  return test::write_temp(
      "hybrid_astar.yaml",
      "algorithm: hybrid_astar\ncategory: global_planning\nparams:\n"
      "  - {name: min_turn_radius, type: float, default: 3.0, min: 0.1, max: 50.0, description: r}\n"
      "  - {name: arc_step, type: float, default: 0.5, min: 0.05, max: 20.0, description: s}\n"
      "  - {name: num_steering, type: int, default: 5, min: 2, max: 51, description: n}\n"
      "  - {name: theta_bins, type: int, default: 72, min: 4, max: 360, description: t}\n"
      "  - {name: xy_resolution, type: float, default: 0.5, min: 0.01, max: 10.0, description: x}\n"
      "  - {name: footprint_radius, type: float, default: 0.3, min: 0.01, max: 20.0, description: f}\n"
      "  - {name: allow_reverse, type: bool, default: true, description: rev}\n"
      "  - {name: reverse_penalty, type: float, default: 2.0, min: 1.0, max: 100.0, description: rp}\n"
      "  - {name: steer_penalty, type: float, default: 0.1, min: 0.0, max: 100.0, description: sp}\n"
      "  - {name: goal_pos_tolerance, type: float, default: 0.5, min: 0.01, max: 20.0, "
      "description: gp}\n"
      "  - {name: goal_heading_tolerance, type: float, default: 0.26, min: 0.01, max: 3.1416, "
      "description: gh}\n");
}

// Number of comma-separated elements in the JSON array that follows "key":[ ... ].
int array_len(const std::string& line, const std::string& key) {
  size_t at = line.find("\"" + key + "\":[");
  if (at == std::string::npos) return -1;
  size_t open = line.find('[', at);
  size_t close = line.find(']', open);
  std::string inner = line.substr(open + 1, close - open - 1);
  if (inner.empty()) return 0;
  return 1 + static_cast<int>(std::count(inner.begin(), inner.end(), ','));
}

}  // namespace

// (1) feasible kinodynamic path -------------------------------------------------
TEST(Kinodynamic, HybridAStarFindsFeasiblePath) {
  auto g = test::make_grid(open_rows(48));  // resolution 0.5 -> 24m x 24m open world
  auto params = cfg();
  Pose start{3.0, 3.0, 0.0}, goal{14.0, 14.0, 0.0};
  global_planning::HybridAStarPlanner planner(params);
  auto r = planner.plan(g, start, goal, nullptr);
  ASSERT_TRUE(r.success);

  const Footprint fp{params.get_float("footprint_radius")};
  const double arc = params.get_float("arc_step");
  const double turn_r = params.get_float("min_turn_radius");
  EXPECT_NEAR(r.path.front().x, start.x, 1e-12);
  EXPECT_NEAR(r.path.front().y, start.y, 1e-12);
  EXPECT_NEAR(r.path.front().theta, start.theta, 1e-12);
  const Pose& end = r.path.back();
  EXPECT_LE(std::hypot(end.x - goal.x, end.y - goal.y),
            params.get_float("goal_pos_tolerance") + 1e-9);
  EXPECT_LE(std::abs(wrap_pi(end.theta - goal.theta)),
            params.get_float("goal_heading_tolerance") + 1e-9);
  for (const Pose& p : r.path) EXPECT_FALSE(g.is_collision(fp, p));
  for (size_t i = 0; i + 1 < r.path.size(); ++i) {
    const Pose& a = r.path[i];
    const Pose& b = r.path[i + 1];
    EXPECT_LE(std::hypot(b.x - a.x, b.y - a.y), arc + 1e-9);          // within one primitive
    EXPECT_LE(std::abs(wrap_pi(b.theta - a.theta)), arc / turn_r + 1e-9);  // max-curvature bound
  }
}

// (2) no-path -------------------------------------------------------------------
TEST(Kinodynamic, HybridAStarNoPathAcrossFullWall) {
  auto g = test::make_grid({"...#...", "...#...", "...#...", "...#...", "...#...", "...#...",
                            "...#..."});  // full middle column (resolution 0.5 -> world 3.5)
  global_planning::HybridAStarPlanner planner(cfg());
  auto r = planner.plan(g, Pose{0.75, 1.75, 0.0}, Pose{2.75, 1.75, 0.0}, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.path.empty());
  EXPECT_EQ(r.cost, 0.0);
}

// (3) reverse branch ------------------------------------------------------------
TEST(Kinodynamic, HybridAStarUsesReverseWhenEnabled) {
  auto g = test::make_grid(open_rows(40));  // world 20m
  global_planning::HybridAStarPlanner planner(core::ParamSet::from_yaml(reverse_cfg()));
  auto r = planner.plan(g, Pose{10.0, 10.0, 0.0}, Pose{6.0, 10.0, 0.0}, nullptr);
  ASSERT_TRUE(r.success);
  bool has_reverse = false;
  for (size_t i = 0; i + 1 < r.path.size(); ++i) {
    const Pose& a = r.path[i];
    const Pose& b = r.path[i + 1];
    // Reverse: displacement opposite the heading (dot < 0).
    if ((b.x - a.x) * std::cos(a.theta) + (b.y - a.y) * std::sin(a.theta) < 0.0) has_reverse = true;
  }
  EXPECT_TRUE(has_reverse);
}

// (4) param validation failure --------------------------------------------------
TEST(Kinodynamic, BadHybridAStarParamThrows) {
  std::string bad = test::write_temp(
      "hybrid_astar.yaml",
      "algorithm: hybrid_astar\ncategory: global_planning\nparams:\n"
      "  - name: min_turn_radius\n    type: float\n    default: 0.0\n    min: 0.1\n    max: 50.0\n"
      "    description: below min\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}

// (5) trace-emission contract ---------------------------------------------------
TEST(Kinodynamic, HybridAStarEmitsArcAndPathTrace) {
  auto g = test::make_grid(open_rows(20));  // world 10m
  global_planning::HybridAStarPlanner planner(cfg());
  std::ostringstream os;
  core::TraceRecorder rec(os);
  auto r = planner.plan(g, Pose{2.0, 2.0, 0.0}, Pose{5.0, 5.0, 0.0}, &rec);
  ASSERT_TRUE(r.success);

  std::istringstream in(os.str());
  std::string line, last;
  int node_expanded = 0, edges = 0, path_found = 0;
  while (std::getline(in, line)) {
    if (line.find("\"event\":\"node_expanded\"") != std::string::npos) ++node_expanded;
    if (line.find("\"event\":\"edge_added\"") != std::string::npos) {
      ++edges;
      EXPECT_EQ(array_len(line, "state"), 3) << line;   // [x, y, theta]
      EXPECT_EQ(array_len(line, "parent"), 3) << line;
    }
    if (line.find("\"event\":\"path_found\"") != std::string::npos) ++path_found;
    if (!line.empty()) last = line;
  }
  EXPECT_GE(node_expanded, 1);
  EXPECT_GE(edges, 1);
  EXPECT_EQ(path_found, 1);
  EXPECT_NE(last.find("\"event\":\"planning_finished\""), std::string::npos);
  EXPECT_NE(last.find("\"runtime_sec\""), std::string::npos);
  EXPECT_NE(last.find("\"path_cost\""), std::string::npos);
  EXPECT_NE(last.find("\"expanded_nodes\""), std::string::npos);
}
