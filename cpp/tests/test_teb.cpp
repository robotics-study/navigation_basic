#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/core/params.hpp"
#include "navigation/core/trace.hpp"
#include "navigation/core/types.hpp"
#include "navigation/local_planning/band/teb.hpp"
#include "navigation/local_planning/geometry.hpp"
#include "navigation/local_planning/simulation.hpp"
#include "navigation/maps/loader.hpp"
#include "navigation/maps/occupancy_grid.hpp"
#include "test_util.hpp"

// TEB (Rösmann et al. 2012/2017): a fixed-iteration damped gradient-descent
// solver jointly optimizing a timed pose chain against reference tracking,
// obstacle clearance, velocity/acceleration limits, time-optimality, and a
// nonholonomic two-pose-arc constraint -- verified via the closed-loop
// simulator on a real map/scenario (a path skimming an obstacle's edge) and
// via direct compute_command calls for behavior a full episode can't isolate
// cleanly (a single tick's cost-term response to a weight change, resize's
// pose-count bounds).

using namespace navigation;
using core::LocalTask;
using core::ParamSet;
using core::Point;
using core::Pose;
using core::RobotState;
using core::TraceRecorder;
using local_planning::integrate_unicycle;
using local_planning::nearest_occupied;
using local_planning::SimConfig;
using local_planning::SimStatus;
using local_planning::simulate;
using local_planning::TebPlanner;

namespace {

std::string base_config_text() {
  std::string path = test::repo_path("configs/local_planning/teb.yaml");
  std::ifstream in(path);
  std::ostringstream buf;
  buf << in.rdbuf();
  return buf.str();
}

// Overrides a single named float param's default by re-declaring it after the
// original document: ParamSet::from_yaml keeps the last occurrence of a
// repeated key when the yaml parser folds duplicate map keys, mirroring
// test_elastic_bands.cpp's config_with_rho_min() precedent.
ParamSet config_with_override(const std::string& name, double value) {
  std::ostringstream extra;
  extra << "\n  - {name: " << name << ", type: float, default: " << value
        << ", min: -1000.0, max: 1000.0, description: override}\n";
  std::string path = test::write_temp("teb_" + name + ".yaml", base_config_text() + extra.str());
  return ParamSet::from_yaml(path);
}

// Same, but for a param declared `type: int` in the base yaml (max_steps) --
// redeclaring it as float would flip its type and break every get_int() call
// site downstream, since ParamSet::get_int rejects a type mismatch.
ParamSet config_with_int_override(const std::string& name, int value) {
  std::ostringstream extra;
  extra << "\n  - {name: " << name << ", type: int, default: " << value
        << ", min: 1, max: 100000, description: override}\n";
  std::string path = test::write_temp("teb_" + name + ".yaml", base_config_text() + extra.str());
  return ParamSet::from_yaml(path);
}

ParamSet real_config() { return ParamSet::from_yaml(test::repo_path("configs/local_planning/teb.yaml")); }

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

}  // namespace

// (a) skims the clutter map's top-right obstacle edge without colliding -------
TEST(Teb, ReachesGoalOnSkimmingPath) {
  auto params = real_config();
  auto map = maps::load_map(test::repo_path("maps/grid/clutter01.yaml"));
  auto& grid = as_grid(*map);
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/clutter01_s2.yaml"));
  TebPlanner planner(params);
  RobotState start{Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
  LocalTask task{Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, sc.reference_path};
  auto result = simulate(planner, grid, start, task, sim_config_from(params), nullptr);
  EXPECT_EQ(result.status, SimStatus::REACHED);
  EXPECT_TRUE(result.success);
}

// (b) every executed command stays within the declared v/omega limits ---------
TEST(Teb, CommandsRespectLimits) {
  auto params = real_config();
  auto map = maps::load_map(test::repo_path("maps/grid/clutter01.yaml"));
  auto& grid = as_grid(*map);
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/clutter01_s2.yaml"));
  TebPlanner planner(params);
  RobotState start{Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
  LocalTask task{Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, sc.reference_path};

  std::ostringstream os;
  TraceRecorder rec(os);
  auto result = simulate(planner, grid, start, task, sim_config_from(params), &rec);
  EXPECT_EQ(result.status, SimStatus::REACHED);

  const double v_max = params.get_float("v_max");
  const double omega_max = params.get_float("omega_max");
  std::vector<std::string> moves = lines_with_event(split_lines(os.str()), "robot_moved");
  ASSERT_FALSE(moves.empty());
  for (const std::string& line : moves) {
    EXPECT_LE(std::fabs(find_data_value(line, "v")), v_max + 1e-9);
    EXPECT_LE(std::fabs(find_data_value(line, "omega")), omega_max + 1e-9);
  }
}

// (c) raising w_time shortens the optimized band's total horizon time ---------
TEST(Teb, TimeWeightShortensBandTime) {
  auto map = maps::load_map(test::repo_path("maps/grid/clutter01.yaml"));
  auto& grid = as_grid(*map);
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/clutter01_s2.yaml"));
  RobotState state{Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
  LocalTask task{Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, sc.reference_path};

  auto horizon_time = [&](double w_time) {
    ParamSet params = config_with_override("w_time", w_time);
    TebPlanner planner(params);
    std::ostringstream os;
    TraceRecorder rec(os);
    planner.compute_command(grid, state, task, params.get_float("control_dt"), &rec);
    std::vector<std::string> bands = lines_with_event(split_lines(os.str()), "band_updated");
    EXPECT_EQ(bands.size(), 1u);
    return find_data_value(bands[0], "horizon_time");
  };

  EXPECT_LT(horizon_time(5.0), horizon_time(0.0));
}

// (d) raising w_obstacle increases the trajectory-average clearance -----------
TEST(Teb, ObstacleWeightIncreasesClearance) {
  // SimResult.min_clearance is the quantized EDT's trajectory minimum, not
  // sensitive enough to show the obstacle term's effect (see the Python test
  // for the full rationale). This measures the continuous nearest-occupied
  // distance d~ (the same quantity the cost term optimizes) at every
  // trajectory point within a fixed radius of some obstacle, averaged over
  // the whole episode -- a single nearest-block minimum is too path-shape-
  // sensitive for a two-point comparison of a ~40-iteration damped solver
  // that never fully converges.
  auto map = maps::load_map(test::repo_path("maps/grid/clutter01.yaml"));
  auto& grid = as_grid(*map);
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/clutter01_s2.yaml"));
  constexpr double kNearRadius = 1.5;

  auto mean_near_clearance = [&](double w_obstacle) {
    ParamSet params = config_with_override("w_obstacle", w_obstacle);
    TebPlanner planner(params);
    RobotState start{Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
    LocalTask task{Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, sc.reference_path};
    auto result = simulate(planner, grid, start, task, sim_config_from(params), nullptr);
    EXPECT_EQ(result.status, SimStatus::REACHED);
    double sum = 0.0;
    int n = 0;
    for (const Pose& pose : result.trajectory) {
      auto [o, d_tilde] = nearest_occupied(grid, Point{pose.x, pose.y}, kNearRadius);
      if (o) {
        sum += d_tilde;
        ++n;
      }
    }
    EXPECT_GT(n, 0);
    return sum / static_cast<double>(n);
  };

  EXPECT_GT(mean_near_clearance(15.0), mean_near_clearance(0.0));
}

// (e) resize never lets the band's pose count leave [3, max_poses] ------------
TEST(Teb, BandResizingBounds) {
  // A window short enough that the robot stays far from the local goal (so
  // the n<3 degenerate branch, which legitimately reports poses=2 right
  // before REACHED, never triggers) but long enough to exercise several
  // resize passes.
  ParamSet params = config_with_int_override("max_steps", 60);
  auto map = maps::load_map(test::repo_path("maps/grid/clutter01.yaml"));
  auto& grid = as_grid(*map);
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/clutter01_s2.yaml"));
  TebPlanner planner(params);
  RobotState state{Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
  LocalTask task{Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, sc.reference_path};
  const double dt = params.get_float("control_dt");
  const int max_poses = params.get_int("max_poses");

  std::ostringstream os;
  TraceRecorder rec(os);
  for (int i = 0; i < 60; ++i) {
    core::VelocityCommand cmd = planner.compute_command(grid, state, task, dt, &rec);
    Pose new_pose = integrate_unicycle(state.pose, cmd, dt);
    state = RobotState{new_pose, cmd.v, cmd.omega};
  }

  std::vector<std::string> bands = lines_with_event(split_lines(os.str()), "band_updated");
  ASSERT_FALSE(bands.empty());
  for (const std::string& line : bands) {
    double poses = find_data_value(line, "poses");
    EXPECT_GE(poses, 3.0);
    EXPECT_LE(poses, static_cast<double>(max_poses));
  }
}

// (f) an out-of-range override fails load-time validation ---------------------
TEST(Teb, ParamValidation) {
  std::string bad = test::write_temp(
      "teb_w_path.yaml",
      "algorithm: teb\ncategory: local_planning\nparams:\n"
      "  - {name: w_path, type: float, default: -1.0, min: 0.0, max: 100.0, "
      "description: below min}\n");
  EXPECT_THROW(ParamSet::from_yaml(bad), std::runtime_error);
}
