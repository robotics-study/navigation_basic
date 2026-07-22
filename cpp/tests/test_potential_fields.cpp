#include <cmath>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "navigation/core/params.hpp"
#include "navigation/core/trace.hpp"
#include "navigation/core/types.hpp"
#include "navigation/local_planning/reactive/potential_fields.hpp"
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
using local_planning::PotentialFieldsPlanner;
using local_planning::SimConfig;
using local_planning::SimStatus;
using local_planning::simulate;

namespace {

ParamSet real_config() {
  return ParamSet::from_yaml(test::repo_path("configs/local_planning/potential_fields.yaml"));
}

SimConfig sim_config_from(const ParamSet& params) {
  return SimConfig{params.get_float("control_dt"),     params.get_int("max_steps"),
                   params.get_float("goal_tolerance"), params.get_float("footprint_radius"),
                   params.get_int("stall_window"),     params.get_float("stall_distance")};
}

maps::OccupancyGrid2D& as_grid(core::MapBase& m) {
  return *dynamic_cast<maps::OccupancyGrid2D*>(&m);
}

local_planning::SimResult run_scenario(const std::string& map_name, const std::string& scenario_name) {
  auto params = real_config();
  auto map = maps::load_map(test::repo_path("maps/grid/" + map_name + ".yaml"), 0, 8);
  auto& grid = as_grid(*map);
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/" + scenario_name + ".yaml"));
  PotentialFieldsPlanner planner(params);
  RobotState start{Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
  LocalTask task{Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, {}};
  return simulate(planner, grid, start, task, sim_config_from(params), nullptr);
}

// Locates `"key":<number>` in a raw trace line and parses the number -- the
// scalar counterpart of the array-field extraction helper in test_sampling.cpp.
double find_data_value(const std::string& line, const std::string& key) {
  std::string needle = "\"" + key + "\":";
  size_t at = line.find(needle);
  if (at == std::string::npos) return std::nan("");
  size_t start = at + needle.size();
  size_t end = line.find_first_of(",}", start);
  return std::stod(line.substr(start, end - start));
}

}  // namespace

// (a) reaches goal on open/clutter maps without collision -----------------------

TEST(PotentialFields, ReachesGoalOnOpenMapWithoutCollision) {
  auto r = run_scenario("open01", "open01_s1");
  EXPECT_EQ(r.status, SimStatus::REACHED);
  EXPECT_TRUE(r.success);
  EXPECT_GT(r.min_clearance, 0.0);
}

TEST(PotentialFields, ReachesGoalOnClutterMapWithoutCollision) {
  auto r = run_scenario("clutter01", "clutter01_s1");
  EXPECT_EQ(r.status, SimStatus::REACHED);
  EXPECT_TRUE(r.success);
  EXPECT_GT(r.min_clearance, 0.0);
}

// (b) U-trap: correct local-minimum stall, not a bug ----------------------------

TEST(PotentialFields, StallsInUTrapLocalMinimum) {
  // pf_trap01_s1: the goal sits straight through the U-trap's back wall, in
  // line with the opening. The attractive pull drives the robot in, and at
  // this row the top/bottom wall repulsion is symmetric so it cancels the
  // attractive pull head-on against the back wall -- a textbook local
  // minimum (Khatib 1986), not a simulator or tuning defect.
  auto r = run_scenario("pf_trap01", "pf_trap01_s1");
  EXPECT_EQ(r.status, SimStatus::STALLED);
  EXPECT_FALSE(r.success);
}

// (c) param validation failure ---------------------------------------------------

TEST(PotentialFields, BadKAttThrows) {
  std::string bad = test::write_temp(
      "potential_fields.yaml",
      "algorithm: potential_fields\ncategory: local_planning\nparams:\n"
      "  - name: k_att\n    type: float\n    default: 200.0\n    min: 0.001\n    max: 100.0\n"
      "    description: over max\n");
  EXPECT_THROW(ParamSet::from_yaml(bad), std::runtime_error);
}

// (behavior) repulsive force points away from a single obstacle -----------------

TEST(PotentialFields, RepulsiveForcePointsAwayFromObstacle) {
  // Single obstacle at ascii row 2 / col 2 -> world center (2.5, 2.5) (see
  // OccupancyGrid2D::cell_to_world). Robot sits one cell due north (2.5, 3.5),
  // so FIRAS repulsion must point further north (+y), away from the obstacle
  // -- an influence_radius just above the 1.0 m gap keeps this the only
  // source, isolating F_rep from the (goal-independent) attractive term.
  auto grid = test::make_grid({".......", ".......", "..#....", ".......", "......."});
  std::string cfg_path = test::write_temp(
      "potential_fields.yaml",
      "algorithm: potential_fields\ncategory: local_planning\nparams:\n"
      "  - {name: k_att, type: float, default: 0.5, min: 0.001, max: 100, description: a}\n"
      "  - {name: k_rep, type: float, default: 1.0, min: 0.0, max: 100, description: r}\n"
      "  - {name: influence_radius, type: float, default: 1.05, min: 0.05, max: 20, description: i}\n"
      "  - {name: k_v, type: float, default: 1.0, min: 0.001, max: 100, description: v}\n"
      "  - {name: k_omega, type: float, default: 4.0, min: 0.001, max: 100, description: o}\n"
      "  - {name: max_speed, type: float, default: 0.8, min: 0.01, max: 10, description: s}\n"
      "  - {name: max_omega, type: float, default: 5.0, min: 0.01, max: 20, description: w}\n"
      "  - {name: control_dt, type: float, default: 0.1, min: 0.001, max: 1, description: dt}\n"
      "  - {name: max_steps, type: int, default: 1000, min: 1, max: 100000, description: n}\n"
      "  - {name: goal_tolerance, type: float, default: 0.3, min: 0.01, max: 5, description: g}\n"
      "  - {name: footprint_radius, type: float, default: 0.2, min: 0.01, max: 5, description: f}\n"
      "  - {name: stall_window, type: int, default: 20, min: 1, max: 100000, description: sw}\n"
      "  - {name: stall_distance, type: float, default: 0.05, min: 0.0, max: 5, description: sd}\n");
  auto params = ParamSet::from_yaml(cfg_path);
  PotentialFieldsPlanner planner(params);
  RobotState state{Pose{2.5, 3.5, 0.0}, 0.0, 0.0};
  LocalTask task{Pose{2.5, 3.5, 0.0}, {}};  // coincident with start: F_att == 0

  std::ostringstream os;
  TraceRecorder rec(os);
  planner.compute_command(grid, state, task, params.get_float("control_dt"), &rec);

  std::istringstream in(os.str());
  std::string line;
  std::getline(in, line);
  ASSERT_NE(line.find("\"force_computed\""), std::string::npos);
  EXPECT_NEAR(find_data_value(line, "fx_rep"), 0.0, 1e-9);
  EXPECT_GT(find_data_value(line, "fy_rep"), 0.0);  // pushed north, away from the obstacle
}
