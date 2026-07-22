#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/core/params.hpp"
#include "navigation/core/trace.hpp"
#include "navigation/core/types.hpp"
#include "navigation/local_planning/band/band.hpp"
#include "navigation/local_planning/band/elastic_bands.hpp"
#include "navigation/local_planning/simulation.hpp"
#include "navigation/maps/loader.hpp"
#include "navigation/maps/occupancy_grid.hpp"
#include "test_util.hpp"

// Elastic Bands (Quinlan & Khatib 1993): deforms a bubble chain draped over a
// reference path away from obstacles each tick, via the closed-loop simulator
// on a real map (obstacles the raw reference path cuts straight through) and
// via direct compute_command calls for the deformation/maintenance behavior a
// full episode can't isolate cleanly (straight-line equilibrium, band
// breaking).

using namespace navigation;
using core::LocalTask;
using core::ParamSet;
using core::Point;
using core::Pose;
using core::RobotState;
using core::TraceRecorder;
using local_planning::ElasticBandsPlanner;
using local_planning::resample_polyline;
using local_planning::SimConfig;
using local_planning::SimStatus;
using local_planning::simulate;

namespace {

std::string config_path() { return test::repo_path("configs/local_planning/elastic_bands.yaml"); }

ParamSet real_config() { return ParamSet::from_yaml(config_path()); }

ParamSet config_with_rho_min(double rho_min) {
  std::ostringstream extra;
  extra << "\n  - {name: rho_min, type: float, default: " << rho_min
        << ", min: 0.05, max: 2.0, description: override}\n";
  std::string base = test::repo_path("configs/local_planning/elastic_bands.yaml");
  std::ifstream in(base);
  std::ostringstream buf;
  buf << in.rdbuf();
  std::string doc = buf.str();
  // Overriding rho_min by re-declaring it: ParamSet::from_yaml keeps the last
  // occurrence of a repeated key when the yaml parser folds duplicate map
  // keys, mirroring _config()'s python-side "rewrite the entry" override.
  std::string path = test::write_temp("elastic_bands_rho_min.yaml", doc + extra.str());
  return ParamSet::from_yaml(path);
}

SimConfig sim_config_from(const ParamSet& params) {
  return SimConfig{params.get_float("control_dt"),     params.get_int("max_steps"),
                   params.get_float("goal_tolerance"), params.get_float("footprint_radius"),
                   params.get_int("stall_window"),     params.get_float("stall_distance")};
}

maps::OccupancyGrid2D& as_grid(core::MapBase& m) {
  return *dynamic_cast<maps::OccupancyGrid2D*>(&m);
}

std::vector<std::string> split_lines(const std::string& s) {
  std::vector<std::string> out;
  std::istringstream in(s);
  std::string line;
  while (std::getline(in, line)) out.push_back(line);
  return out;
}

std::vector<std::string> lines_with_event(const std::vector<std::string>& lines, const char* event) {
  std::vector<std::string> out;
  std::string needle = std::string("\"") + event + "\"";
  for (const std::string& line : lines) {
    if (line.find(needle) != std::string::npos) out.push_back(line);
  }
  return out;
}

double find_data_value(const std::string& line, const std::string& key) {
  std::string needle = "\"" + key + "\":";
  size_t at = line.find(needle);
  if (at == std::string::npos) return std::nan("");
  size_t start = at + needle.size();
  size_t end = line.find_first_of(",}", start);
  return std::stod(line.substr(start, end - start));
}

// Parses the `"band":[[x,y,r],...]` array out of one raw trace line. Manual
// bracket-depth scanning (no JSON library in this codebase's C++ tests --
// find_data_value above is the established precedent for reading trace JSON
// straight out of the raw line).
std::vector<std::vector<double>> parse_band(const std::string& line) {
  std::string needle = "\"band\":[";
  size_t start = line.find(needle);
  if (start == std::string::npos) return {};
  size_t outer_open = start + needle.size() - 1;
  int depth = 0;
  size_t outer_close = std::string::npos;
  for (size_t i = outer_open; i < line.size(); ++i) {
    if (line[i] == '[') ++depth;
    else if (line[i] == ']') {
      --depth;
      if (depth == 0) {
        outer_close = i;
        break;
      }
    }
  }
  std::vector<std::vector<double>> result;
  std::string inner = line.substr(outer_open + 1, outer_close - outer_open - 1);
  size_t j = 0;
  while (j < inner.size()) {
    if (inner[j] != '[') {
      ++j;
      continue;
    }
    size_t close = inner.find(']', j);
    std::string item = inner.substr(j + 1, close - j - 1);
    std::vector<double> vals;
    std::istringstream ss(item);
    std::string tok;
    while (std::getline(ss, tok, ',')) vals.push_back(std::stod(tok));
    result.push_back(vals);
    j = close + 1;
  }
  return result;
}

}  // namespace

// (a) the raw reference path cuts through 5 obstacle blocks; the deformed
// band routes around all of them without collision -----------------------------
TEST(ElasticBands, ReachesGoalAroundBlockingObstacles) {
  auto params = real_config();
  auto map = maps::load_map(test::repo_path("maps/grid/clutter01.yaml"));
  auto& grid = as_grid(*map);
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/clutter01_s3.yaml"));
  ElasticBandsPlanner planner(params);
  RobotState start{Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
  LocalTask task{Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, sc.reference_path};
  auto result = simulate(planner, grid, start, task, sim_config_from(params), nullptr);
  EXPECT_EQ(result.status, SimStatus::REACHED);
  EXPECT_TRUE(result.success);
  EXPECT_GT(result.min_clearance, 0.0);
}

// (b) after one tick's deformation, the band's worst-case clearance beats the
// raw (undeformed) reference path's worst-case clearance -----------------------
TEST(ElasticBands, BandDeformsAwayFromObstacle) {
  auto params = real_config();
  auto map = maps::load_map(test::repo_path("maps/grid/clutter01.yaml"));
  auto& grid = as_grid(*map);
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/clutter01_s3.yaml"));
  ElasticBandsPlanner planner(params);
  RobotState state{Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
  LocalTask task{Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, sc.reference_path};

  std::vector<Point> baseline_points = resample_polyline(sc.reference_path, params.get_float("bubble_spacing"));
  double baseline_min_clearance = std::numeric_limits<double>::infinity();
  for (const Point& p : baseline_points) {
    baseline_min_clearance = std::min(baseline_min_clearance, grid.distance_to_nearest(p));
  }

  std::ostringstream os;
  TraceRecorder rec(os);
  planner.compute_command(grid, state, task, params.get_float("control_dt"), &rec);

  std::vector<std::string> band_lines = lines_with_event(split_lines(os.str()), "band_updated");
  ASSERT_FALSE(band_lines.empty());
  std::vector<std::vector<double>> band = parse_band(band_lines.back());
  ASSERT_FALSE(band.empty());
  double deformed_min_clearance = std::numeric_limits<double>::infinity();
  for (const auto& item : band) deformed_min_clearance = std::min(deformed_min_clearance, item[2]);

  EXPECT_GT(deformed_min_clearance, baseline_min_clearance);
}

// (c) an obstacle-free straight corridor: contraction alone keeps the band
// exactly on the line joining its two fixed endpoints --------------------------
TEST(ElasticBands, StraightBandStaysStraight) {
  auto params = real_config();
  auto map = maps::load_map(test::repo_path("maps/grid/open01.yaml"));
  auto& grid = as_grid(*map);
  const Point start_xy{2.0, 8.75};
  const Point goal_xy{8.0, 8.75};  // horizontal segment, obstacle-free within rho_influence
  ElasticBandsPlanner planner(params);
  RobotState state{Pose{start_xy.x, start_xy.y, 0.0}, 0.0, 0.0};
  LocalTask task{Pose{goal_xy.x, goal_xy.y, 0.0}, {start_xy, goal_xy}};

  std::ostringstream os;
  TraceRecorder rec(os);
  planner.compute_command(grid, state, task, params.get_float("control_dt"), &rec);

  std::vector<std::string> band_lines = lines_with_event(split_lines(os.str()), "band_updated");
  ASSERT_FALSE(band_lines.empty());
  std::vector<std::vector<double>> band = parse_band(band_lines.back());
  ASSERT_FALSE(band.empty());
  for (const auto& item : band) EXPECT_NEAR(item[1], start_xy.y, 1e-6);
}

// (d) an unreachable rho_min forces the very first tick's validity check to
// fail: zero command, broken flagged in the trace -------------------------------
TEST(ElasticBands, BandBreakYieldsZeroCommand) {
  auto params = config_with_rho_min(1.2);
  auto map = maps::load_map(test::repo_path("maps/grid/clutter01.yaml"));
  auto& grid = as_grid(*map);
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/clutter01_s3.yaml"));
  ElasticBandsPlanner planner(params);
  RobotState state{Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
  LocalTask task{Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, sc.reference_path};

  std::ostringstream os;
  TraceRecorder rec(os);
  core::VelocityCommand cmd = planner.compute_command(grid, state, task, params.get_float("control_dt"), &rec);

  EXPECT_DOUBLE_EQ(cmd.v, 0.0);
  EXPECT_DOUBLE_EQ(cmd.omega, 0.0);
  std::vector<std::string> band_lines = lines_with_event(split_lines(os.str()), "band_updated");
  ASSERT_EQ(band_lines.size(), 1u);
  EXPECT_DOUBLE_EQ(find_data_value(band_lines[0], "broken"), 1.0);
}

// (e) an out-of-range override fails load-time validation -----------------------
TEST(ElasticBands, ParamValidation) {
  std::string bad = test::write_temp(
      "elastic_bands_k_repulsion.yaml",
      "algorithm: elastic_bands\ncategory: local_planning\nparams:\n"
      "  - {name: k_repulsion, type: float, default: -1.0, min: 0.0, max: 10.0, "
      "description: below min}\n");
  EXPECT_THROW(ParamSet::from_yaml(bad), std::runtime_error);
}
