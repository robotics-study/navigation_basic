#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/global_planning/astar.hpp"
#include "navigation/global_planning/bfs.hpp"
#include "navigation/global_planning/dijkstra.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::Cell;

namespace {

core::ParamSet cfg(const std::string& algo) {
  return core::ParamSet::from_yaml(test::repo_path("configs/global_planning/" + algo + ".yaml"));
}

bool path_is_connected(maps::OccupancyGrid2D& g, const std::vector<Cell>& path) {
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    bool ok = false;
    for (const auto& [v, w] : g.neighbors(path[i])) {
      (void)w;
      if (v == path[i + 1]) ok = true;
    }
    if (!ok) return false;
  }
  return true;
}

}  // namespace

// (a) valid/optimal path on a known map --------------------------------------

TEST(Discrete, DijkstraAndAstarAreOptimal) {
  auto g = test::make_grid({"...", "...", "..."});  // open 3x3, 8-connected
  Cell start{2, 0}, goal{0, 2};
  double optimal = 2.0 * std::sqrt(2.0);

  global_planning::DijkstraPlanner dij(cfg("dijkstra"));
  auto rd = dij.plan(g, start, goal, nullptr);
  ASSERT_TRUE(rd.success);
  EXPECT_NEAR(rd.cost, optimal, 1e-9);
  EXPECT_EQ(rd.path.front(), start);
  EXPECT_EQ(rd.path.back(), goal);

  global_planning::AstarPlanner astar(cfg("astar"));
  auto ra = astar.plan(g, start, goal, nullptr);
  ASSERT_TRUE(ra.success);
  EXPECT_NEAR(ra.cost, optimal, 1e-9);
}

TEST(Discrete, BfsMinimisesMoveCount) {
  auto g = test::make_grid({"...", "...", "..."});
  global_planning::BfsPlanner bfs(cfg("bfs"));
  auto r = bfs.plan(g, Cell{2, 0}, Cell{0, 2}, nullptr);
  ASSERT_TRUE(r.success);
  EXPECT_EQ(r.path.size(), 3u);  // 2 diagonal moves = 3 cells
  EXPECT_TRUE(path_is_connected(g, r.path));
}

// (b) no-path case ------------------------------------------------------------

TEST(Discrete, NoPathWhenWalledOff) {
  auto g = test::make_grid({".#.", ".#.", ".#."});  // middle column blocks all crossing
  Cell start{2, 0}, goal{0, 2};
  global_planning::AstarPlanner astar(cfg("astar"));
  auto r = astar.plan(g, start, goal, nullptr);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.path.empty());
  global_planning::DijkstraPlanner dij(cfg("dijkstra"));
  EXPECT_FALSE(dij.plan(g, start, goal, nullptr).success);
  global_planning::BfsPlanner bfs(cfg("bfs"));
  auto rb = bfs.plan(g, start, goal, nullptr);
  EXPECT_FALSE(rb.success);
  EXPECT_TRUE(rb.path.empty());
  EXPECT_EQ(rb.cost, 0.0);
}

// (c) param validation failure -----------------------------------------------

TEST(Discrete, BadAstarParamThrows) {
  std::string bad = test::write_temp(
      "astar.yaml",
      "algorithm: astar\ncategory: global_planning\nparams:\n"
      "  - name: heuristic_weight\n    type: float\n    default: 0.5\n    min: 1.0\n    max: 5.0\n"
      "    description: below min\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}
