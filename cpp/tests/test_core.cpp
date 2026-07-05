#include <cmath>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/core/params.hpp"
#include "navigation/core/trace.hpp"
#include "navigation/core/yaml.hpp"
#include "navigation/maps/loader.hpp"
#include "navigation/maps/occupancy_grid.hpp"
#include "navigation/maps/pgm.hpp"
#include "test_util.hpp"

using namespace navigation;

// --- YAML parser on the real repo files -------------------------------------

TEST(Yaml, ParsesRealConfigWithBlockSequenceAndFoldedScalar) {
  core::YamlNode root = core::parse_yaml_file(test::repo_path("configs/global_planning/rrt.yaml"));
  EXPECT_EQ(root.at("algorithm").as_string(), "rrt");
  EXPECT_EQ(root.at("category").as_string(), "global_planning");
  const core::YamlNode& params = root.at("params");
  ASSERT_TRUE(params.is_seq());
  EXPECT_EQ(params.seq.front().at("name").as_string(), "max_iterations");
  EXPECT_EQ(params.seq.front().at("default").as_int(), 5000);
}

TEST(Yaml, ParsesRealMapAndScenarioFlowSequences) {
  core::YamlNode m = core::parse_yaml_file(test::repo_path("maps/grid/maze01.yaml"));
  EXPECT_EQ(m.at("type").as_string(), "occupancy_grid");
  EXPECT_DOUBLE_EQ(m.at("resolution").as_double(), 0.5);
  EXPECT_DOUBLE_EQ(m.at("origin").seq.at(0).as_double(), 0.0);

  core::YamlNode s = core::parse_yaml_file(test::repo_path("maps/scenarios/maze01_s1.yaml"));
  EXPECT_DOUBLE_EQ(s.at("start").seq.at(0).as_double(), 0.75);
  EXPECT_DOUBLE_EQ(s.at("goal").seq.at(1).as_double(), 9.25);
}

TEST(Yaml, EmptyFlowSequenceIsEmpty) {
  core::YamlNode root = core::parse_yaml_file(test::repo_path("configs/global_planning/bfs.yaml"));
  EXPECT_TRUE(root.at("params").is_seq());
  EXPECT_TRUE(root.at("params").seq.empty());
}

// --- ParamSet validation -----------------------------------------------------

TEST(Params, LoadsRealAstarConfig) {
  auto p = core::ParamSet::from_yaml(test::repo_path("configs/global_planning/astar.yaml"));
  EXPECT_EQ(p.algorithm(), "astar");
  EXPECT_DOUBLE_EQ(p.get_float("heuristic_weight"), 1.0);
}

TEST(Params, OutOfRangeDefaultThrows) {
  std::string bad = test::write_temp("astar.yaml",
                                     "algorithm: astar\ncategory: global_planning\nparams:\n"
                                     "  - name: heuristic_weight\n    type: float\n"
                                     "    default: 9.0\n    min: 1.0\n    max: 5.0\n"
                                     "    description: weight\n");
  EXPECT_THROW(core::ParamSet::from_yaml(bad), std::runtime_error);
}

TEST(Params, WrongTypeAccessThrows) {
  auto p = core::ParamSet::from_yaml(test::repo_path("configs/global_planning/rrt.yaml"));
  EXPECT_EQ(p.get_int("max_iterations"), 5000);
  EXPECT_THROW(p.get_float("max_iterations"), std::runtime_error);  // declared int
}

// --- Capabilities ------------------------------------------------------------

TEST(Capabilities, GridSupportsDiscreteSamplingLineOfSightAndDynamicGrid) {
  auto g = test::make_grid({"..", ".."});
  EXPECT_TRUE(g.supports(core::Capability::DISCRETE_SPACE));
  EXPECT_TRUE(g.supports(core::Capability::SAMPLING_SPACE));
  EXPECT_TRUE(g.supports(core::Capability::LINE_OF_SIGHT_SPACE));
  EXPECT_TRUE(g.supports(core::Capability::DYNAMIC_GRID_SPACE));
  EXPECT_FALSE(g.supports(core::Capability::OBSTACLE_QUERY));
}

// --- Grid geometry + thresholding -------------------------------------------

TEST(Grid, WorldCellRoundTrip) {
  auto g = test::make_grid({"....", "....", "....", "...."});
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      core::Cell cell{r, c};
      core::Point w = g.cell_to_world(cell);
      core::Cell back = g.world_to_cell(w.x, w.y);
      EXPECT_EQ(back.row, r);
      EXPECT_EQ(back.col, c);
    }
  }
}

TEST(Grid, OccupancyThresholding) {
  // occ = 1 - pixel/255: black(0)->occupied, white(255)->free, mid(128)->unknown->blocked.
  maps::PgmImage img;
  img.width = 3;
  img.height = 1;
  img.maxval = 255;
  img.pixels = {0, 128, 255};
  auto g = maps::OccupancyGrid2D::from_image(img, 0.5, 0.0, 0.0, 0.65, 0.196, 8, 0);
  EXPECT_FALSE(g.is_free(0, 0));  // occupied
  EXPECT_FALSE(g.is_free(0, 1));  // unknown treated as blocked
  EXPECT_TRUE(g.is_free(0, 2));   // free
}

TEST(Grid, EightConnPreventsCornerCutting) {
  // Diagonal from (1,0) to (0,1) is blocked because both shared orthogonals are occupied.
  auto g = test::make_grid({".#", "#."});
  auto nbrs = g.neighbors(core::Cell{1, 0});
  for (const auto& [c, w] : nbrs) {
    (void)w;
    EXPECT_FALSE(c.row == 0 && c.col == 1) << "corner-cut diagonal must be excluded";
  }
}

TEST(Grid, OctileHeuristicIsAdmissible) {
  auto g = test::make_grid({"...", "...", "..."});
  // 8-conn optimal from (2,0) to (0,2) is two diagonals; heuristic must not exceed it.
  double h = g.heuristic(core::Cell{2, 0}, core::Cell{0, 2});
  EXPECT_LE(h, 2.0 * std::sqrt(2.0) + 1e-9);
}

TEST(Grid, MotionRejectsCornerClip) {
  // resolution 0.5: the center cell spans [1.0,1.5)x[1.0,1.5). The segment
  // y = x + 0.48 grazes its top-left corner with an in-cell chord (~0.028) far
  // shorter than any fixed sample spacing — must be rejected, both directions.
  auto g = test::make_grid({".....", ".....", "..#..", ".....", "....."});
  EXPECT_FALSE(g.is_motion_valid({0.55, 1.03}, {1.55, 2.03}));
  EXPECT_FALSE(g.is_motion_valid({1.55, 2.03}, {0.55, 1.03}));
  // y = x + 0.52 passes just outside the same corner and must stay valid.
  EXPECT_TRUE(g.is_motion_valid({0.55, 1.07}, {1.55, 2.07}));
}

TEST(Grid, MotionExactCornerCrossingFollowsCornerCutRule) {
  // Passing exactly through a corner point obeys the neighbors() rule: both
  // shared orthogonal cells must be free.
  auto blocked = test::make_grid({".#", "#."});
  EXPECT_FALSE(blocked.is_motion_valid({0.25, 0.75}, {0.75, 0.25}));
  auto open = test::make_grid({"..", ".."});
  EXPECT_TRUE(open.is_motion_valid({0.25, 0.25}, {0.75, 0.75}));
}

TEST(Grid, MotionDegenerateAndGridlineSegments) {
  auto g = test::make_grid({"..", ".#"});
  EXPECT_TRUE(g.is_motion_valid({0.25, 0.75}, {0.25, 0.75}));    // zero-length, free cell
  EXPECT_FALSE(g.is_motion_valid({0.75, 0.25}, {0.75, 0.25}));   // zero-length, occupied cell
  // Vertical run lying exactly on a grid line (zero delta on one axis).
  auto open = test::make_grid({"..", ".."});
  EXPECT_TRUE(open.is_motion_valid({0.5, 0.1}, {0.5, 0.9}));
}

// --- Trace -------------------------------------------------------------------

TEST(Trace, SeqMonotonicAndFieldsPresent) {
  std::ostringstream os;
  core::TraceRecorder rec(os);
  rec.planning_started("astar", "maps/grid/maze01.yaml", {{"heuristic_weight", core::ParamValue(1.0)}});
  rec.node_expanded(core::Cell{2, 0}, 0.0);
  rec.edge_added(core::Cell{1, 0}, core::Cell{2, 0}, 1.0);
  rec.path_found(std::vector<core::Cell>{{2, 0}, {1, 0}});
  rec.planning_finished(true, {{"path_cost", 1.0}});

  std::istringstream in(os.str());
  std::string line;
  long long expected = 0;
  std::string first, last;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::string tag = "\"seq\":" + std::to_string(expected);
    EXPECT_NE(line.find(tag), std::string::npos) << "seq must increment by 1: " << line;
    if (expected == 0) first = line;
    last = line;
    ++expected;
  }
  EXPECT_EQ(expected, 5);
  EXPECT_NE(first.find("\"event\":\"planning_started\""), std::string::npos);
  EXPECT_NE(first.find("\"map\":\"maps/grid/maze01.yaml\""), std::string::npos);
  EXPECT_NE(last.find("\"event\":\"planning_finished\""), std::string::npos);
}

TEST(Trace, CellStateSerializesAsIntegerArray) {
  std::ostringstream os;
  core::TraceRecorder rec(os);
  rec.node_expanded(core::Cell{3, 7});
  EXPECT_NE(os.str().find("\"state\":[3,7]"), std::string::npos);
}

// --- PGM reader --------------------------------------------------------------

TEST(Pgm, P2AsciiWithCommentAndBlockLayout) {
  std::string p = test::write_temp("a.pgm", "P2\n# a comment\n3 2\n255\n0 255 0\n255 0 255\n");
  maps::PgmImage img = maps::load_pgm(p);
  EXPECT_EQ(img.width, 3);
  EXPECT_EQ(img.height, 2);
  EXPECT_EQ(img.pixels, (std::vector<int>{0, 255, 0, 255, 0, 255}));
}

TEST(Pgm, P5BinaryMatchesAsciiValues) {
  std::string content = "P5\n3 2\n255\n";
  for (int v : {0, 255, 128, 64, 200, 10}) content += static_cast<char>(v);
  std::string p = test::write_temp("b.pgm", content);
  maps::PgmImage img = maps::load_pgm(p);
  EXPECT_EQ(img.pixels, (std::vector<int>{0, 255, 128, 64, 200, 10}));
}

TEST(Pgm, P5SixteenBitBigEndian) {
  // Values above 255 need two big-endian bytes per sample (Netpbm spec).
  std::vector<int> values{0, 65535, 300, 1000};
  std::string content = "P5\n2 2\n65535\n";
  for (int v : values) {
    content += static_cast<char>((v >> 8) & 0xFF);
    content += static_cast<char>(v & 0xFF);
  }
  std::string p = test::write_temp("c.pgm", content);
  maps::PgmImage img = maps::load_pgm(p);
  EXPECT_EQ(img.pixels, values);
}

TEST(Pgm, TruncatedAsciiThrows) {
  std::string p = test::write_temp("d.pgm", "P2\n3 3\n255\n0 255 0\n");  // 3 of 9 values
  EXPECT_THROW(maps::load_pgm(p), std::runtime_error);
}

TEST(Pgm, UnsupportedMagicThrows) {
  std::string p = test::write_temp("e.pgm", "P3\n1 1\n255\n0 0 0\n");  // P3 = color PPM
  EXPECT_THROW(maps::load_pgm(p), std::runtime_error);
}

// --- Map / scenario loaders --------------------------------------------------

TEST(Loader, LoadsRealOccupancyGrid) {
  auto map = maps::load_map(test::repo_path("maps/grid/maze01.yaml"), 0, 8);
  ASSERT_NE(map, nullptr);
  EXPECT_TRUE(map->supports(core::Capability::DISCRETE_SPACE));
  EXPECT_TRUE(map->supports(core::Capability::SAMPLING_SPACE));
}

TEST(Loader, UnsupportedMapTypeThrows) {
  std::string p = test::write_temp("g.yaml", "type: graph\nnodes: []\nedges: []\n");
  EXPECT_THROW(maps::load_map(p), std::runtime_error);
}

TEST(Loader, ScenarioResolvesMapPathToAbsolute) {
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/maze01_s1.yaml"));
  std::filesystem::path expected =
      std::filesystem::weakly_canonical(test::repo_path("maps/grid/maze01.yaml"));
  EXPECT_EQ(std::filesystem::path(sc.map_path), expected);
  EXPECT_DOUBLE_EQ(sc.start.x, 0.75);
  EXPECT_DOUBLE_EQ(sc.goal.y, 9.25);
}

TEST(Loader, MultiAgentScenarioRejected) {
  std::string p = test::write_temp("s.yaml", "map: ../grid/maze01.yaml\nagents: []\n");
  EXPECT_THROW(maps::load_scenario(p), std::runtime_error);
}
