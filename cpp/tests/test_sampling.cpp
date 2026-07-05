#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/core/trace.hpp"
#include "navigation/global_planning/fast_rrt.hpp"
#include "navigation/global_planning/rrt.hpp"
#include "navigation/global_planning/rrt_star.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::Point;

namespace {

// Open 10x10 grid (5m x 5m world); '.' everywhere unless overridden.
std::vector<std::string> open_rows(int n = 10) {
  return std::vector<std::string>(static_cast<size_t>(n), std::string(static_cast<size_t>(n), '.'));
}

// Modest iteration budgets keep the anytime planners fast in tests while still
// exercising real behavior (choose-parent, rewire, shortcut pruning).
std::string rrt_cfg() {
  return test::write_temp("rrt.yaml",
                          "algorithm: rrt\ncategory: global_planning\nparams:\n"
                          "  - {name: max_iterations, type: int, default: 4000, min: 1, max: 200000, description: n}\n"
                          "  - {name: step_size, type: float, default: 0.5, min: 0.01, max: 100, description: eta}\n"
                          "  - {name: goal_bias, type: float, default: 0.2, min: 0.0, max: 1.0, description: b}\n"
                          "  - {name: goal_tolerance, type: float, default: 0.3, min: 0.0, max: 100, description: t}\n"
                          "  - {name: seed, type: int, default: 7, min: 0, max: 2147483647, description: s}\n");
}
std::string rrt_star_cfg() {
  return test::write_temp("rrt_star.yaml",
                          "algorithm: rrt_star\ncategory: global_planning\nparams:\n"
                          "  - {name: max_iterations, type: int, default: 3000, min: 1, max: 200000, description: n}\n"
                          "  - {name: step_size, type: float, default: 0.5, min: 0.01, max: 100, description: eta}\n"
                          "  - {name: goal_bias, type: float, default: 0.2, min: 0.0, max: 1.0, description: b}\n"
                          "  - {name: goal_tolerance, type: float, default: 0.3, min: 0.0, max: 100, description: t}\n"
                          "  - {name: neighbor_radius, type: float, default: 1.5, min: 0.01, max: 100, description: r}\n"
                          "  - {name: seed, type: int, default: 7, min: 0, max: 2147483647, description: s}\n");
}
std::string fast_rrt_cfg() {
  return test::write_temp("fast_rrt.yaml",
                          "algorithm: fast_rrt\ncategory: global_planning\nparams:\n"
                          "  - {name: max_iterations, type: int, default: 3000, min: 1, max: 200000, description: n}\n"
                          "  - {name: step_size, type: float, default: 0.5, min: 0.01, max: 100, description: eta}\n"
                          "  - {name: goal_bias, type: float, default: 0.2, min: 0.0, max: 1.0, description: b}\n"
                          "  - {name: goal_tolerance, type: float, default: 0.3, min: 0.0, max: 100, description: t}\n"
                          "  - {name: neighbor_radius, type: float, default: 1.5, min: 0.01, max: 100, description: r}\n"
                          "  - {name: reached_radius, type: float, default: 0.4, min: 0.0, max: 100, description: rr}\n"
                          "  - {name: steering_attempts, type: int, default: 10, min: 1, max: 100, description: sa}\n"
                          "  - {name: seed, type: int, default: 7, min: 0, max: 2147483647, description: s}\n");
}

void expect_path_valid(maps::OccupancyGrid2D& g, const std::vector<Point>& path, const Point& start,
                       const Point& goal, double tol) {
  ASSERT_GE(path.size(), 2u);
  EXPECT_NEAR(path.front().x, start.x, 1e-9);
  EXPECT_NEAR(path.front().y, start.y, 1e-9);
  EXPECT_LE(g.distance(path.back(), goal), tol + 1e-9);
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    EXPECT_TRUE(g.is_motion_valid(path[i], path[i + 1])) << "path segment must be collision-free";
  }
}

}  // namespace

// (a) valid path reaching the goal -------------------------------------------

TEST(Sampling, RrtFindsValidPath) {
  auto g = test::make_grid(open_rows());
  Point start{0.25, 0.25}, goal{4.75, 4.75};
  global_planning::RrtPlanner p(core::ParamSet::from_yaml(rrt_cfg()));
  auto r = p.plan(g, start, goal, nullptr);
  ASSERT_TRUE(r.success);
  expect_path_valid(g, r.path, start, goal, 0.3);
}

// edge_added 의 parent 는 반드시 기존 트리 노드여야 한다. tree.add() 이전에 잡아둔
// 노드 참조가 재할당으로 dangling 되면 쓰레기 좌표가 방출되는 회귀를 잡는다.
TEST(Sampling, RrtTraceEdgeParentsAreTreeNodes) {
  auto g = test::make_grid(open_rows());
  Point start{0.25, 0.25}, goal{4.75, 4.75};
  global_planning::RrtPlanner p(core::ParamSet::from_yaml(rrt_cfg()));
  std::ostringstream os;
  core::TraceRecorder rec(os);
  auto r = p.plan(g, start, goal, &rec);
  ASSERT_TRUE(r.success);

  auto parse_point = [](const std::string& line, const std::string& key) {
    size_t at = line.find("\"" + key + "\":[");
    EXPECT_NE(at, std::string::npos) << line;
    size_t open = line.find('[', at);
    size_t comma = line.find(',', open);
    size_t close = line.find(']', comma);
    return std::pair<double, double>{std::stod(line.substr(open + 1, comma - open - 1)),
                                     std::stod(line.substr(comma + 1, close - comma - 1))};
  };

  std::set<std::pair<double, double>> nodes{{start.x, start.y}};
  std::istringstream in(os.str());
  std::string line;
  int edges = 0;
  while (std::getline(in, line)) {
    if (line.find("\"event\":\"edge_added\"") == std::string::npos) continue;
    ++edges;
    EXPECT_TRUE(nodes.count(parse_point(line, "parent"))) << "parent is not a tree node: " << line;
    nodes.insert(parse_point(line, "state"));
  }
  EXPECT_GT(edges, 2);
}

TEST(Sampling, RrtStarFindsValidPath) {
  auto g = test::make_grid(open_rows());
  Point start{0.25, 0.25}, goal{4.75, 4.75};
  global_planning::RrtStarPlanner p(core::ParamSet::from_yaml(rrt_star_cfg()));
  auto r = p.plan(g, start, goal, nullptr);
  ASSERT_TRUE(r.success);
  // The last segment connects the tree to the exact goal.
  EXPECT_NEAR(r.path.back().x, goal.x, 1e-9);
  EXPECT_NEAR(r.path.back().y, goal.y, 1e-9);
  for (size_t i = 0; i + 1 < r.path.size(); ++i) {
    EXPECT_TRUE(g.is_motion_valid(r.path[i], r.path[i + 1]));
  }
}

TEST(Sampling, FastRrtFindsValidPath) {
  auto g = test::make_grid(open_rows());
  Point start{0.25, 0.25}, goal{4.75, 4.75};
  global_planning::FastRrtPlanner p(core::ParamSet::from_yaml(fast_rrt_cfg()));
  auto r = p.plan(g, start, goal, nullptr);
  ASSERT_TRUE(r.success);
  for (size_t i = 0; i + 1 < r.path.size(); ++i) {
    EXPECT_TRUE(g.is_motion_valid(r.path[i], r.path[i + 1]));
  }
}

// (b) no-path case: a full wall separates start from goal ---------------------

TEST(Sampling, NoPathAcrossFullWall) {
  auto rows = open_rows();
  rows[5] = "##########";  // impassable horizontal wall
  auto g = test::make_grid(rows);
  Point start{0.25, 0.25}, goal{4.75, 4.75};
  global_planning::RrtPlanner rrt(core::ParamSet::from_yaml(rrt_cfg()));
  EXPECT_FALSE(rrt.plan(g, start, goal, nullptr).success);
  global_planning::FastRrtPlanner fr(core::ParamSet::from_yaml(fast_rrt_cfg()));
  EXPECT_FALSE(fr.plan(g, start, goal, nullptr).success);
  global_planning::RrtStarPlanner rs(core::ParamSet::from_yaml(rrt_star_cfg()));
  auto r = rs.plan(g, start, goal, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.path.empty());
}

// (c) param validation failure -----------------------------------------------

TEST(Sampling, BadGoalBiasThrows) {
  std::string bad = test::write_temp(
      "rrt.yaml",
      "algorithm: rrt\ncategory: global_planning\nparams:\n"
      "  - {name: goal_bias, type: float, default: 2.0, min: 0.0, max: 1.0, description: over max}\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}
