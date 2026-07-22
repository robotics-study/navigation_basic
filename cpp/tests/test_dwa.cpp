#include <cmath>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/core/params.hpp"
#include "navigation/core/trace.hpp"
#include "navigation/core/types.hpp"
#include "navigation/local_planning/reactive/dwa.hpp"
#include "navigation/local_planning/simulation.hpp"
#include "navigation/maps/loader.hpp"
#include "navigation/maps/occupancy_grid.hpp"
#include "test_util.hpp"

// DWA (Fox, Burgard & Thrun 1997): samples the accel-limited (v, omega)
// window, rolls each candidate forward as a constant-command arc, and picks
// the highest-scoring one that can still stop before the nearest obstacle --
// via the closed-loop simulator on real maps/scenarios (clutter weaving, a
// dead-end U-trap) and via direct compute_command calls for the
// admissibility/window bounds a full episode can't isolate.

using namespace navigation;
using core::LocalTask;
using core::ParamSet;
using core::Pose;
using core::RobotState;
using core::TraceRecorder;
using local_planning::DwaPlanner;
using local_planning::SimConfig;
using local_planning::SimStatus;
using local_planning::simulate;

namespace {

std::string config_path() { return test::repo_path("configs/local_planning/dwa.yaml"); }

ParamSet real_config() { return ParamSet::from_yaml(config_path()); }

SimConfig sim_config_from(const ParamSet& params) {
  return SimConfig{params.get_float("control_dt"),     params.get_int("max_steps"),
                   params.get_float("goal_tolerance"), params.get_float("footprint_radius"),
                   params.get_int("stall_window"),     params.get_float("stall_distance")};
}

maps::OccupancyGrid2D& as_grid(core::MapBase& m) {
  return *dynamic_cast<maps::OccupancyGrid2D*>(&m);
}

local_planning::SimResult run_scenario(const std::string& map_name, const std::string& scenario_name,
                                       const ParamSet& params) {
  auto map = maps::load_map(test::repo_path("maps/grid/" + map_name + ".yaml"));
  auto& grid = as_grid(*map);
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/" + scenario_name + ".yaml"));
  DwaPlanner planner(params);
  RobotState start{Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
  LocalTask task{Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, {}};
  return simulate(planner, grid, start, task, sim_config_from(params), nullptr);
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

std::vector<std::string> lines_with_event(const std::vector<std::string>& lines, const char* event) {
  std::vector<std::string> out;
  std::string needle = std::string("\"") + event + "\"";
  for (const std::string& line : lines) {
    if (line.find(needle) != std::string::npos) out.push_back(line);
  }
  return out;
}

}  // namespace

// (a) weaves the clutter map's obstacles to the goal without colliding ----------
TEST(Dwa, ReachesGoalOnClutterMapWeavingObstacles) {
  auto params = real_config();
  auto result = run_scenario("clutter01", "clutter01_s1", params);
  EXPECT_EQ(result.status, SimStatus::REACHED);
  EXPECT_TRUE(result.success);
  EXPECT_GT(result.min_clearance, 0.0);
}

// (b) U-trap dead end: admissibility forces a stop, not a wall graze ------------
TEST(Dwa, DeadEndTrapStallsWithoutColliding) {
  auto params = real_config();
  auto result = run_scenario("pf_trap01", "pf_trap01_s1", params);
  EXPECT_EQ(result.status, SimStatus::STALLED);
  EXPECT_FALSE(result.success);
}

// (c) v_samples below the declared minimum fails load-time validation -----------
TEST(Dwa, VSamplesBelowMinRejectedAtLoadTime) {
  std::string bad = test::write_temp(
      "dwa_v_samples.yaml",
      "algorithm: dwa\ncategory: local_planning\nparams:\n"
      "  - {name: v_samples, type: int, default: 0, min: 2, max: 50, description: below min}\n");
  EXPECT_THROW(ParamSet::from_yaml(bad), std::runtime_error);
}

// (d) admissibility: a close head-on wall forces a slower, bounded command ------
TEST(Dwa, AdmissibleBoundForcesDecelerationFacingACloseWall) {
  // Fine resolution (0.02 m) keeps the EDT-based clearance estimate close to
  // the true continuous distance, so a hand-picked gap reliably separates
  // "max-speed candidates rejected" from "some slower candidate admissible" --
  // a coarser grid's cell-center quantization would make this razor-thin.
  auto params = real_config();
  const double resolution = 0.02;
  const int cols = 3000, rows = 300;
  const double x0 = 5.0, y0 = 3.0, gap = 1.7;
  const int wall_col = static_cast<int>(std::round((x0 + gap) / resolution - 0.5));
  std::vector<bool> free_cells(static_cast<size_t>(rows) * cols, true);
  for (int r = 0; r < rows; ++r) {
    for (int c = wall_col; c < cols; ++c) free_cells[static_cast<size_t>(r) * cols + c] = false;
  }
  maps::OccupancyGrid2D grid(rows, cols, resolution, 0.0, 0.0, std::move(free_cells));

  DwaPlanner planner(params);
  const double v_a = params.get_float("max_speed");
  RobotState state{Pose{x0, y0, 0.0}, v_a, 0.0};
  LocalTask task{Pose{x0 + 100.0, y0, 0.0}, {}};  // straight ahead, past the wall

  std::ostringstream os;
  TraceRecorder rec(os);
  core::VelocityCommand cmd = planner.compute_command(grid, state, task, params.get_float("control_dt"), &rec);

  EXPECT_LT(cmd.v, v_a);
  std::vector<std::string> selected = lines_with_event(split_lines(os.str()), "candidate_evaluated");
  std::vector<std::string> selected_only;
  for (const std::string& line : selected) {
    if (find_data_value(line, "selected") == 1.0) selected_only.push_back(line);
  }
  ASSERT_EQ(selected_only.size(), 1u);
  const std::string& sel = selected_only[0];
  EXPECT_DOUBLE_EQ(find_data_value(sel, "admissible"), 1.0);
  EXPECT_NEAR(cmd.v, find_data_value(sel, "v"), 1e-9);
  double bound = std::sqrt(2.0 * find_data_value(sel, "clearance") * params.get_float("accel"));
  EXPECT_LE(cmd.v, bound + 1e-9);
}

// (d2) no admissible candidate at all: maximum-braking fallback -----------------
TEST(Dwa, AllCandidatesCollidingFallsBackToMaximumBraking) {
  // Wall so close (gap 0.25 m vs footprint 0.2 m) that every candidate's very
  // first rollout pose already collides: the window's v floor is positive at
  // this speed, so no candidate can stand still and none survives scoring.
  auto params = real_config();
  const double resolution = 0.02;
  const int cols = 300, rows = 300;
  const double x0 = 3.0, y0 = 3.0;
  const int wall_col = static_cast<int>(std::round((x0 + 0.25) / resolution - 0.5));
  std::vector<bool> free_cells(static_cast<size_t>(rows) * cols, true);
  for (int r = 0; r < rows; ++r) {
    for (int c = wall_col; c < cols; ++c) free_cells[static_cast<size_t>(r) * cols + c] = false;
  }
  maps::OccupancyGrid2D grid(rows, cols, resolution, 0.0, 0.0, std::move(free_cells));

  DwaPlanner planner(params);
  const double dt = params.get_float("control_dt");
  const double v_a = params.get_float("max_speed");
  const double omega_a = 1.0;
  RobotState state{Pose{x0, y0, 0.0}, v_a, omega_a};
  LocalTask task{Pose{x0 + 100.0, y0, 0.0}, {}};

  std::ostringstream os;
  TraceRecorder rec(os);
  core::VelocityCommand cmd = planner.compute_command(grid, state, task, dt, &rec);

  // Braking at the kinematic limits: v drops by exactly one tick's accel
  // budget and omega decays toward zero without flipping sign.
  EXPECT_NEAR(cmd.v, v_a - params.get_float("accel") * dt, 1e-9);
  const double expected_omega = omega_a - std::min(omega_a, params.get_float("accel_omega") * dt);
  EXPECT_NEAR(cmd.omega, expected_omega, 1e-9);
  EXPECT_GE(cmd.omega, 0.0);
  EXPECT_LT(cmd.omega, omega_a);
  std::vector<std::string> candidates = lines_with_event(split_lines(os.str()), "candidate_evaluated");
  ASSERT_FALSE(candidates.empty());
  for (const std::string& line : candidates) {
    EXPECT_DOUBLE_EQ(find_data_value(line, "admissible"), 0.0);
    EXPECT_DOUBLE_EQ(find_data_value(line, "selected"), 0.0);
  }
}

// (e) dynamic window: stopped robot cannot exceed one tick's accel budget -------
TEST(Dwa, DynamicWindowCapsAccelerationFromAStop) {
  auto params = real_config();
  std::vector<bool> free_cells(static_cast<size_t>(200) * 200, true);
  maps::OccupancyGrid2D grid(200, 200, 0.1, 0.0, 0.0, std::move(free_cells));
  DwaPlanner planner(params);
  RobotState state{Pose{5.0, 5.0, 0.0}, 0.0, 0.0};
  LocalTask task{Pose{50.0, 5.0, 0.0}, {}};

  core::VelocityCommand cmd =
      planner.compute_command(grid, state, task, params.get_float("control_dt"), nullptr);

  EXPECT_LE(cmd.v, params.get_float("accel") * params.get_float("control_dt") + 1e-9);
}

// (f) reset/determinism: identical rerun produces the identical trajectory ------
TEST(Dwa, RerunAfterImplicitResetIsDeterministic) {
  // simulate() calls planner.reset() itself at the top of every episode; Dwa
  // carries no cursor to leak, but the deterministic (v outer, omega inner)
  // sampling grid must still reproduce bit-identical candidate selection.
  auto params = real_config();
  auto map = maps::load_map(test::repo_path("maps/grid/clutter01.yaml"));
  auto& grid = as_grid(*map);
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/clutter01_s1.yaml"));
  DwaPlanner planner(params);
  RobotState start{Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
  LocalTask task{Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, {}};
  SimConfig config = sim_config_from(params);

  auto first = simulate(planner, grid, start, task, config, nullptr);
  auto second = simulate(planner, grid, start, task, config, nullptr);

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

// (g) trace: candidates carry a rollout, and selected == best admissible --------
TEST(Dwa, RecorderEmitsRolloutsAndSelectsTheBestAdmissibleCandidate) {
  auto params = real_config();
  auto map = maps::load_map(test::repo_path("maps/grid/clutter01.yaml"));
  auto& grid = as_grid(*map);
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/clutter01_s1.yaml"));
  DwaPlanner planner(params);
  RobotState state{Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
  LocalTask task{Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, {}};

  std::ostringstream os;
  TraceRecorder rec(os);
  planner.compute_command(grid, state, task, params.get_float("control_dt"), &rec);

  std::vector<std::string> candidates = lines_with_event(split_lines(os.str()), "candidate_evaluated");
  int expected = params.get_int("v_samples") * params.get_int("omega_samples");
  ASSERT_EQ(static_cast<int>(candidates.size()), expected);
  for (const std::string& line : candidates) {
    EXPECT_NE(line.find("\"rollout\":"), std::string::npos);
  }

  double best_admissible_cost = -std::numeric_limits<double>::infinity();
  int selected_count = 0;
  double selected_cost = std::nan("");
  bool any_admissible = false;
  for (const std::string& line : candidates) {
    double cost = find_data_value(line, "cost");
    if (find_data_value(line, "admissible") == 1.0) {
      any_admissible = true;
      best_admissible_cost = std::max(best_admissible_cost, cost);
    }
    if (find_data_value(line, "selected") == 1.0) {
      ++selected_count;
      selected_cost = cost;
    }
  }
  EXPECT_TRUE(any_admissible);
  EXPECT_EQ(selected_count, 1);
  EXPECT_DOUBLE_EQ(selected_cost, best_admissible_cost);
}
