#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/core/params.hpp"
#include "navigation/core/trace.hpp"
#include "navigation/core/types.hpp"
#include "navigation/local_planning/simulation.hpp"
#include "navigation/local_planning/tracking/regulated_pure_pursuit.hpp"
#include "navigation/maps/loader.hpp"
#include "navigation/maps/occupancy_grid.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::LocalTask;
using core::ParamSet;
using core::Point;
using core::Pose;
using core::RobotState;
using core::TraceRecorder;
using local_planning::RegulatedPurePursuitPlanner;
using local_planning::SimConfig;
using local_planning::SimStatus;
using local_planning::simulate;

namespace {

std::string config_path() { return test::repo_path("configs/local_planning/regulated_pure_pursuit.yaml"); }

ParamSet real_config() { return ParamSet::from_yaml(config_path()); }

SimConfig sim_config_from(const ParamSet& params) {
  return SimConfig{params.get_float("control_dt"),     params.get_int("max_steps"),
                   params.get_float("goal_tolerance"), params.get_float("footprint_radius"),
                   params.get_int("stall_window"),     params.get_float("stall_distance")};
}

maps::OccupancyGrid2D& as_grid(core::MapBase& m) {
  return *dynamic_cast<maps::OccupancyGrid2D*>(&m);
}

// A large fully-free grid, wide enough that a robot near its center never
// approaches the border -- isolates whatever regulation a test wants to
// exercise from the border-as-obstacle EDT effect.
maps::OccupancyGrid2D open_grid(int n = 40) {
  std::vector<bool> free_cells(static_cast<size_t>(n) * static_cast<size_t>(n), true);
  return maps::OccupancyGrid2D(n, n, 1.0, 0.0, 0.0, free_cells);
}

// Locates `"key":<number>` in a raw trace line and parses the number (mirrors
// test_vfh.cpp's helper of the same name).
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

// Runs compute_command once and returns the single candidate_evaluated line's
// raw JSON text.
std::string single_candidate_line(RegulatedPurePursuitPlanner& planner, core::ObstacleQuery& space,
                                  const RobotState& state, const LocalTask& task,
                                  const ParamSet& params) {
  std::ostringstream os;
  TraceRecorder rec(os);
  planner.compute_command(space, state, task, params.get_float("control_dt"), &rec);
  std::vector<std::string> lines = split_lines(os.str());
  for (const std::string& line : lines) {
    if (line.find("\"candidate_evaluated\"") != std::string::npos) return line;
  }
  return "";
}

}  // namespace

// (a) clutter01_s2: reaches the goal without ever colliding ---------------------
TEST(RegulatedPurePursuit, ReachesGoalOnClutterMapWithoutCollision) {
  auto params = real_config();
  auto map = maps::load_map(test::repo_path("maps/grid/clutter01.yaml"));
  auto& grid = as_grid(*map);
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/clutter01_s2.yaml"));
  RegulatedPurePursuitPlanner planner(params);
  RobotState start{Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
  LocalTask task{Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, sc.reference_path};

  auto result = simulate(planner, grid, start, task, sim_config_from(params), nullptr);

  ASSERT_EQ(result.status, SimStatus::REACHED);
  EXPECT_TRUE(result.success);
  EXPECT_GT(result.min_clearance, 0.0);
}

// (b) curvature regulation: tight lookahead turn caps v below v_goal ------------
TEST(RegulatedPurePursuit, TightTurnTriggersCurvatureRegulation) {
  // Path runs due north from the robot while the robot faces east: the
  // lookahead target sits 90 degrees off-heading at L_d=0.5, giving a
  // commanded turn radius of 0.25 m -- well under regulated_min_radius
  // (0.9 m default) so curvature regulation must engage.
  auto params = real_config();
  auto grid = open_grid();
  RegulatedPurePursuitPlanner planner(params);
  RobotState state{Pose{20.0, 20.0, 0.0}, 0.5, 0.0};
  LocalTask task{Pose{20.0, 100.0, 0.0}, {Point{20.0, 20.0}, Point{20.0, 23.0}}};

  std::string line = single_candidate_line(planner, grid, state, task, params);
  ASSERT_FALSE(line.empty());
  core::VelocityCommand cmd =
      planner.compute_command(grid, state, task, params.get_float("control_dt"), nullptr);

  double v_goal = params.get_float("max_speed");  // remaining >> slow_radius
  EXPECT_LT(cmd.v, v_goal);
  EXPECT_LT(find_data_value(line, "curvature_scale"), 1.0);
  EXPECT_DOUBLE_EQ(find_data_value(line, "blocked"), 0.0);
}

// (c) proximity regulation: nearby obstacle caps v below v_goal -----------------
TEST(RegulatedPurePursuit, NearbyObstacleTriggersProximityRegulation) {
  // Obstacle cell sits exactly one grid cell (0.5 m at this resolution) north
  // of the robot's cell; distance_to_nearest quantizes to the probe's own
  // cell so this reads as clearance 0.5-0.2=0.3 m, under proximity_distance
  // (0.6 m default), while the physical footprint disc (radius 0.2 m) stays
  // 0.25 m clear of the obstacle cell's edge -- close enough to regulate, not
  // close enough to collide. Path points straight east (alpha=0) so
  // curvature regulation stays inactive, isolating the proximity effect.
  std::vector<bool> free_cells = {
      true, true, true, true,  true, true, true,  //
      true, true, true, true,  true, true, true,  //
      true, true, true, true,  true, true, true,  //
      true, true, false, true, true, true, true,  //
      true, true, true, true,  true, true, true,  //
      true, true, true, true,  true, true, true,  //
      true, true, true, true,  true, true, true,  //
  };
  maps::OccupancyGrid2D grid(7, 7, 0.5, 0.0, 0.0, free_cells);
  auto params = real_config();
  RegulatedPurePursuitPlanner planner(params);
  RobotState state{Pose{1.25, 1.25, 0.0}, 0.5, 0.0};
  LocalTask task{Pose{100.0, 1.25, 0.0}, {Point{1.25, 1.25}, Point{4.25, 1.25}}};

  std::string line = single_candidate_line(planner, grid, state, task, params);
  ASSERT_FALSE(line.empty());
  core::VelocityCommand cmd =
      planner.compute_command(grid, state, task, params.get_float("control_dt"), nullptr);

  double v_goal = params.get_float("max_speed");
  EXPECT_LT(cmd.v, v_goal);
  EXPECT_LT(find_data_value(line, "proximity_scale"), 1.0);
  EXPECT_DOUBLE_EQ(find_data_value(line, "curvature_scale"), 1.0);
  EXPECT_DOUBLE_EQ(find_data_value(line, "blocked"), 0.0);
}

// (d) blocked: an obstacle sitting on the command arc forces a hard stop --------
TEST(RegulatedPurePursuit, ObstacleOnCommandArcForcesStop) {
  // Obstacle cell one meter due east, robot speed set so L_d (== lookahead
  // collision-check arc length here, since the target is straight ahead)
  // reaches exactly onto it.
  std::vector<bool> free_cells = {
      true, true, true, true,  true,  //
      true, true, true, true,  true,  //
      true, true, true, false, true,  //
      true, true, true, true,  true,  //
      true, true, true, true,  true,  //
  };
  maps::OccupancyGrid2D grid(5, 5, 1.0, 0.0, 0.0, free_cells);
  auto params = real_config();
  RegulatedPurePursuitPlanner planner(params);
  RobotState state{Pose{2.5, 2.5, 0.0}, 1.0, 0.0};
  LocalTask task{Pose{100.0, 2.5, 0.0}, {Point{2.5, 2.5}, Point{4.5, 2.5}}};

  std::string line = single_candidate_line(planner, grid, state, task, params);
  ASSERT_FALSE(line.empty());
  core::VelocityCommand cmd =
      planner.compute_command(grid, state, task, params.get_float("control_dt"), nullptr);

  EXPECT_DOUBLE_EQ(cmd.v, 0.0);
  EXPECT_DOUBLE_EQ(cmd.omega, 0.0);
  EXPECT_DOUBLE_EQ(find_data_value(line, "blocked"), 1.0);
}

// (e) adaptive lookahead follows state.v, clamped to [min, max] -----------------
TEST(RegulatedPurePursuit, LookaheadScalesWithSpeedWithinClamp) {
  auto params = real_config();
  auto grid = open_grid();
  LocalTask task{Pose{100.0, 20.0, 0.0}, {Point{20.0, 20.0}, Point{30.0, 20.0}}};
  double min_lookahead = params.get_float("min_lookahead");
  double max_lookahead = params.get_float("max_lookahead");
  double lookahead_time = params.get_float("lookahead_time");

  // v=0 (stopped) clamps to the floor; a very high v clamps to the ceiling; a
  // moderate v in between reproduces lookahead_time*v exactly.
  std::vector<std::pair<double, double>> cases = {
      {0.0, min_lookahead}, {5.0, max_lookahead}, {0.5, lookahead_time * 0.5}};
  for (const auto& [v, expected] : cases) {
    RegulatedPurePursuitPlanner planner(params);
    RobotState state{Pose{20.0, 20.0, 0.0}, v, 0.0};
    std::string line = single_candidate_line(planner, grid, state, task, params);
    ASSERT_FALSE(line.empty());
    EXPECT_NEAR(find_data_value(line, "lookahead"), expected, 1e-9);
  }
}

// (f) lookahead_time = 0 fails load-time range validation ------------------------
TEST(RegulatedPurePursuit, ZeroLookaheadTimeRejectedAtLoadTime) {
  std::string bad = test::write_temp(
      "regulated_pure_pursuit.yaml",
      "algorithm: regulated_pure_pursuit\ncategory: local_planning\nparams:\n"
      "  - {name: lookahead_time, type: float, default: 0.0, min: 0.01, max: 10, description: lt}\n"
      "  - {name: min_lookahead, type: float, default: 0.25, min: 0.01, max: 100, description: mnl}\n"
      "  - {name: max_lookahead, type: float, default: 1.0, min: 0.01, max: 100, description: mxl}\n"
      "  - {name: regulated_min_radius, type: float, default: 0.9, min: 0.001, max: 100, "
      "description: rmr}\n"
      "  - {name: proximity_distance, type: float, default: 0.6, min: 0.01, max: 100, "
      "description: pd}\n"
      "  - {name: min_regulated_speed, type: float, default: 0.1, min: 0.0, max: 10, "
      "description: mrs}\n"
      "  - {name: collision_check_step, type: float, default: 0.05, min: 0.005, max: 1, "
      "description: ccs}\n"
      "  - {name: max_speed, type: float, default: 0.8, min: 0.01, max: 10, description: ms}\n"
      "  - {name: max_omega, type: float, default: 1.5, min: 0.01, max: 20, description: mo}\n"
      "  - {name: slow_radius, type: float, default: 0.5, min: 0.001, max: 100, description: sr}\n"
      "  - {name: control_dt, type: float, default: 0.1, min: 0.001, max: 1, description: dt}\n"
      "  - {name: max_steps, type: int, default: 1000, min: 1, max: 100000, description: n}\n"
      "  - {name: goal_tolerance, type: float, default: 0.3, min: 0.01, max: 5, description: gt}\n"
      "  - {name: footprint_radius, type: float, default: 0.2, min: 0.01, max: 5, description: fp}\n"
      "  - {name: stall_window, type: int, default: 20, min: 1, max: 100000, description: sw}\n"
      "  - {name: stall_distance, type: float, default: 0.05, min: 0.0, max: 5, "
      "description: sd}\n");
  EXPECT_THROW(ParamSet::from_yaml(bad), std::runtime_error);
}

// (behavior) reset() leaves no progress_index_ cursor leak across reruns --------
TEST(RegulatedPurePursuit, RerunAfterImplicitResetIsDeterministic) {
  auto params = real_config();
  auto map = maps::load_map(test::repo_path("maps/grid/clutter01.yaml"));
  auto& grid = as_grid(*map);
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/clutter01_s2.yaml"));
  RegulatedPurePursuitPlanner planner(params);
  RobotState start{Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
  LocalTask task{Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, sc.reference_path};

  auto first = simulate(planner, grid, start, task, sim_config_from(params), nullptr);
  auto second = simulate(planner, grid, start, task, sim_config_from(params), nullptr);

  EXPECT_EQ(second.status, first.status);
  EXPECT_EQ(second.steps, first.steps);
  EXPECT_NEAR(second.distance_traveled, first.distance_traveled, 1e-9);
  ASSERT_EQ(second.trajectory.size(), first.trajectory.size());
  for (size_t i = 0; i < first.trajectory.size(); ++i) {
    EXPECT_NEAR(first.trajectory[i].x, second.trajectory[i].x, 1e-9);
    EXPECT_NEAR(first.trajectory[i].y, second.trajectory[i].y, 1e-9);
    EXPECT_NEAR(first.trajectory[i].theta, second.trajectory[i].theta, 1e-9);
  }
}
