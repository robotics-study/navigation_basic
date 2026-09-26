#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/core/params.hpp"
#include "navigation/core/trace.hpp"
#include "navigation/core/types.hpp"
#include "navigation/local_planning/predictive/mpc.hpp"
#include "navigation/local_planning/simulation.hpp"
#include "navigation/maps/loader.hpp"
#include "navigation/maps/occupancy_grid.hpp"
#include "test_util.hpp"

// MPC (Klančar & Škrjanc 2007): fixed-iteration projected gradient descent on
// the shared receding-horizon cost J(U), verified via the closed-loop simulator
// on an open-field interior-goal scenario (goal-seeking around an obstacle), the
// executed-command limits, a single tick's descent lowering J, and load-time
// parameter validation.

using namespace navigation;
using core::LocalTask;
using core::ParamSet;
using core::Pose;
using core::RobotState;
using core::TraceRecorder;
using local_planning::MpcPlanner;
using local_planning::SimConfig;
using local_planning::SimStatus;
using local_planning::simulate;

namespace {

constexpr const char* kMap = "maps/grid/open01.yaml";
constexpr const char* kScenario = "maps/scenarios/open01_s4.yaml";

std::string base_config_text() {
  std::ifstream in(test::repo_path("configs/local_planning/mpc.yaml"));
  std::ostringstream buf;
  buf << in.rdbuf();
  return buf.str();
}

// Overrides a single int param's default by re-declaring it after the original
// document: ParamSet::from_yaml keeps the last occurrence of a repeated key,
// mirroring test_teb.cpp's config_with_int_override precedent.
ParamSet config_with_int_override(const std::string& name, int value) {
  std::ostringstream extra;
  extra << "\n  - {name: " << name << ", type: int, default: " << value
        << ", min: 1, max: 100000, description: override}\n";
  std::string path = test::write_temp("mpc_" + name + ".yaml", base_config_text() + extra.str());
  return ParamSet::from_yaml(path);
}

ParamSet real_config() { return ParamSet::from_yaml(test::repo_path("configs/local_planning/mpc.yaml")); }

SimConfig sim_config_from(const ParamSet& p) {
  return SimConfig{p.get_float("control_dt"),     p.get_int("max_steps"),
                   p.get_float("goal_tolerance"), p.get_float("footprint_radius"),
                   p.get_int("stall_window"),     p.get_float("stall_distance")};
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

// One cold-start compute_command from the scenario start, returning its single
// band_updated line's total_cost.
double single_tick_cost(const ParamSet& params) {
  auto map = maps::load_map(test::repo_path(kMap));
  auto& grid = as_grid(*map);
  maps::Scenario sc = maps::load_scenario(test::repo_path(kScenario));
  MpcPlanner planner(params);
  RobotState state{Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
  LocalTask task{Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, sc.reference_path};
  std::ostringstream os;
  TraceRecorder rec(os);
  planner.compute_command(grid, state, task, params.get_float("control_dt"), &rec);
  std::vector<std::string> bands = lines_with_event(split_lines(os.str()), "band_updated");
  EXPECT_EQ(bands.size(), 1u);
  return find_data_value(bands[0], "total_cost");
}

}  // namespace

// (a) closed-loop MPC reaches an open-field interior goal ---------------------
TEST(Mpc, ReachesGoalOpenField) {
  auto params = real_config();
  auto map = maps::load_map(test::repo_path(kMap));
  auto& grid = as_grid(*map);
  maps::Scenario sc = maps::load_scenario(test::repo_path(kScenario));
  MpcPlanner planner(params);
  RobotState start{Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
  LocalTask task{Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, sc.reference_path};
  auto result = simulate(planner, grid, start, task, sim_config_from(params), nullptr);
  EXPECT_EQ(result.status, SimStatus::REACHED);
  EXPECT_TRUE(result.success);
  // Positive clearance means it threaded past the obstacle, not grazed a wall.
  EXPECT_GT(result.min_clearance, 0.0);
}

// (b) every executed command stays within the declared v/omega limits ---------
TEST(Mpc, CommandsRespectLimits) {
  auto params = real_config();
  auto map = maps::load_map(test::repo_path(kMap));
  auto& grid = as_grid(*map);
  maps::Scenario sc = maps::load_scenario(test::repo_path(kScenario));
  MpcPlanner planner(params);
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

// (c) more descent iterations never raise the optimized cost ------------------
TEST(Mpc, MoreIterationsLowerCost) {
  double low = single_tick_cost(config_with_int_override("iterations", 1));
  double high = single_tick_cost(config_with_int_override("iterations", 30));
  EXPECT_LE(high, low + 1e-6);
}

// (f) an out-of-range weight fails load-time validation -----------------------
TEST(Mpc, ParamValidation) {
  std::string bad = test::write_temp(
      "mpc_w_goal.yaml",
      "algorithm: mpc\ncategory: local_planning\nparams:\n"
      "  - {name: w_goal, type: float, default: -1.0, min: 0.0, max: 100.0, "
      "description: below min}\n");
  EXPECT_THROW(ParamSet::from_yaml(bad), std::runtime_error);
}
