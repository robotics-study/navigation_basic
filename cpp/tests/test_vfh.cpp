#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/core/params.hpp"
#include "navigation/core/trace.hpp"
#include "navigation/core/types.hpp"
#include "navigation/local_planning/reactive/vfh.hpp"
#include "navigation/local_planning/simulation.hpp"
#include "navigation/maps/loader.hpp"
#include "navigation/maps/occupancy_grid.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::LocalTask;
using core::ParamSet;
using core::Pose;
using core::RobotState;
using core::TraceRecorder;
using local_planning::SimConfig;
using local_planning::SimStatus;
using local_planning::simulate;
using local_planning::VfhPlanner;

namespace {

ParamSet real_config() { return ParamSet::from_yaml(test::repo_path("configs/local_planning/vfh.yaml")); }

SimConfig sim_config_from(const ParamSet& params) {
  return SimConfig{params.get_float("control_dt"),     params.get_int("max_steps"),
                   params.get_float("goal_tolerance"), params.get_float("footprint_radius"),
                   params.get_int("stall_window"),     params.get_float("stall_distance")};
}

maps::OccupancyGrid2D& as_grid(core::MapBase& m) {
  return *dynamic_cast<maps::OccupancyGrid2D*>(&m);
}

// Locates `"key":<number>` in a raw trace line and parses the number.
double find_data_value(const std::string& line, const std::string& key) {
  std::string needle = "\"" + key + "\":";
  size_t at = line.find(needle);
  if (at == std::string::npos) return std::nan("");
  size_t start = at + needle.size();
  size_t end = line.find_first_of(",}", start);
  return std::stod(line.substr(start, end - start));
}

std::vector<std::string> split_lines(const std::string& s) {
  std::vector<std::string> out;
  std::istringstream in(s);
  std::string line;
  while (std::getline(in, line)) out.push_back(line);
  return out;
}

}  // namespace

// (a) reaches goal on the cluttered map, weaving a gap, never colliding -------

TEST(Vfh, ReachesGoalOnClutterMapWeavingAGap) {
  auto params = real_config();
  auto map = maps::load_map(test::repo_path("maps/grid/clutter01.yaml"), 0, 8);
  auto& grid = as_grid(*map);
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/clutter01_s1.yaml"));
  VfhPlanner planner(params);
  RobotState start{Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
  LocalTask task{Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, {}};
  auto r = simulate(planner, grid, start, task, sim_config_from(params), nullptr);

  EXPECT_EQ(r.status, SimStatus::REACHED);
  EXPECT_TRUE(r.success);
  EXPECT_GT(r.min_clearance, 0.0);
}

// (b) fully enclosed: every sector blocked -> no forward command -> STALLED ----

TEST(Vfh, FullyEnclosedEmitsNoForwardCommandAndStalls) {
  // A single free cell boxed in on all 8 neighbors, resolution small enough
  // (0.2 m) that window_radius=1.32 (default) comfortably covers even the
  // diagonal neighbors (0.283 m away) well above threshold, so every sector
  // -- not just the 4 cardinal ones -- reads as blocked.
  std::vector<bool> free_cells(9, false);
  free_cells[1 * 3 + 1] = true;
  maps::OccupancyGrid2D grid(3, 3, 0.2, 0.0, 0.0, free_cells);
  core::Point center = grid.cell_to_world(core::Cell{1, 1});

  auto params = real_config();
  VfhPlanner planner(params);
  RobotState state{Pose{center.x, center.y, 0.0}, 0.0, 0.0};
  LocalTask task{Pose{100.0, 100.0, 0.0}, {}};  // far away in every direction

  core::VelocityCommand cmd =
      planner.compute_command(grid, state, task, params.get_float("control_dt"), nullptr);
  EXPECT_DOUBLE_EQ(cmd.v, 0.0);

  SimConfig config{params.get_float("control_dt"), 50, 0.05, 0.06, 5, 0.01};
  auto r = simulate(planner, grid, state, task, config, nullptr);
  EXPECT_EQ(r.status, SimStatus::STALLED);
  EXPECT_FALSE(r.success);
}

// (c) param validation failures at load / construction time -------------------

TEST(Vfh, NumSectorsBelowMinRejected) {
  std::string bad = test::write_temp(
      "vfh.yaml",
      "algorithm: vfh\ncategory: local_planning\nparams:\n"
      "  - name: num_sectors\n    type: int\n    default: 4\n    min: 8\n    max: 720\n"
      "    description: below min\n");
  EXPECT_THROW(ParamSet::from_yaml(bad), std::runtime_error);
}

TEST(Vfh, EvenSmoothingWindowRejected) {
  // param_schema.json cannot express "must be odd", so this is enforced by
  // the planner's own constructor (load time, not compute_command) -- an
  // even window has no unambiguous center sector for the moving average.
  auto doc_path = test::write_temp(
      "vfh_even.yaml",
      "algorithm: vfh\ncategory: local_planning\nparams:\n"
      "  - {name: num_sectors, type: int, default: 60, min: 8, max: 720, description: n}\n"
      "  - {name: window_radius, type: float, default: 1.32, min: 0.1, max: 20, description: w}\n"
      "  - {name: threshold, type: float, default: 0.041, min: 0, max: 1000, description: t}\n"
      "  - {name: smoothing_window, type: int, default: 4, min: 1, max: 31, description: s}\n"
      "  - {name: wide_valley_sectors, type: int, default: 20, min: 1, max: 720, description: v}\n"
      "  - {name: h_m, type: float, default: 0.09, min: 0.001, max: 1000, description: h}\n"
      "  - {name: k_omega, type: float, default: 1.54, min: 0.001, max: 100, description: k}\n"
      "  - {name: max_speed, type: float, default: 0.44, min: 0.01, max: 10, description: ms}\n"
      "  - {name: max_omega, type: float, default: 2.86, min: 0.01, max: 20, description: mo}\n"
      "  - {name: control_dt, type: float, default: 0.1, min: 0.001, max: 1, description: dt}\n"
      "  - {name: max_steps, type: int, default: 1000, min: 1, max: 100000, description: n2}\n"
      "  - {name: goal_tolerance, type: float, default: 0.3, min: 0.01, max: 5, description: g}\n"
      "  - {name: footprint_radius, type: float, default: 0.2, min: 0.01, max: 5, description: f}\n"
      "  - {name: stall_window, type: int, default: 20, min: 1, max: 100000, description: sw}\n"
      "  - {name: stall_distance, type: float, default: 0.05, min: 0.0, max: 5, description: sd}\n");
  auto params = ParamSet::from_yaml(doc_path);
  EXPECT_THROW((VfhPlanner(params)), std::runtime_error);
}

// (behavior) frontal obstacle + open side --------------------------------------

TEST(Vfh, FrontalObstacleWithOpenSideSteersTowardOpeningAndReportsBins) {
  // Block sits east + southeast of the robot's free cell (row3 col3, row4-5
  // col3-4); goal is due east (straight into the block), north (rows 0-2) is
  // the only open side -- selected_direction must swing toward +y (north),
  // not stay near the blocked eastward goal bearing.
  // resolution 1.0 (not test::make_grid's fixed 0.5) so world coordinates
  // match the Python mirror's `grid_from` (default resolution 1.0) exactly --
  // window_radius is an absolute-meters parameter, so a different resolution
  // would change how many rings of cells it reaches and could change which
  // side ends up "open" relative to this test's Python counterpart.
  std::vector<bool> free_cells = {
      true,  true,  true,  true,  true,  true,  true,   //
      true,  true,  true,  true,  true,  true,  true,   //
      true,  true,  true,  true,  true,  true,  true,   //
      true,  true,  true,  false, true,  true,  true,   //
      true,  true,  true,  false, false, true,  true,   //
      true,  true,  true,  false, false, true,  true,   //
      true,  true,  true,  true,  true,  true,  true,   //
  };
  maps::OccupancyGrid2D grid(7, 7, 1.0, 0.0, 0.0, free_cells);
  auto params = real_config();
  VfhPlanner planner(params);
  RobotState state{Pose{2.5, 3.5, 0.0}, 0.0, 0.0};
  LocalTask task{Pose{10.0, 3.5, 0.0}, {}};

  std::ostringstream os;
  TraceRecorder rec(os);
  planner.compute_command(grid, state, task, params.get_float("control_dt"), &rec);

  std::vector<std::string> lines = split_lines(os.str());
  std::vector<std::string> hist_lines;
  std::vector<std::string> candidate_lines;
  for (const std::string& line : lines) {
    if (line.find("\"histogram_updated\"") != std::string::npos) hist_lines.push_back(line);
    if (line.find("\"candidate_evaluated\"") != std::string::npos) candidate_lines.push_back(line);
  }
  ASSERT_EQ(hist_lines.size(), 1u);

  // bins array length == num_sectors: count comma-separated entries between
  // the "bins":[ ... ] brackets.
  const std::string& hl = hist_lines[0];
  size_t bins_at = hl.find("\"bins\":[");
  ASSERT_NE(bins_at, std::string::npos);
  size_t open = bins_at + std::string("\"bins\":[").size() - 1;
  size_t close = hl.find(']', open);
  std::string bins_body = hl.substr(open + 1, close - open - 1);
  int bins_count = bins_body.empty() ? 0 : 1;
  for (char c : bins_body) {
    if (c == ',') ++bins_count;
  }
  EXPECT_EQ(bins_count, params.get_int("num_sectors"));

  double selected_direction = find_data_value(hl, "selected_direction");
  EXPECT_GT(selected_direction, 20.0 * M_PI / 180.0);
  EXPECT_LT(selected_direction, 160.0 * M_PI / 180.0);

  ASSERT_GE(candidate_lines.size(), 1u);
  int selected_count = 0;
  for (const std::string& line : candidate_lines) {
    if (find_data_value(line, "selected") == 1.0) ++selected_count;
  }
  EXPECT_EQ(selected_count, 1);
}
