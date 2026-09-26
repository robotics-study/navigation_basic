#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/core/params.hpp"
#include "navigation/core/trace.hpp"
#include "navigation/core/types.hpp"
#include "navigation/local_planning/predictive/mppi.hpp"
#include "navigation/local_planning/simulation.hpp"
#include "navigation/maps/loader.hpp"
#include "navigation/maps/occupancy_grid.hpp"
#include "test_util.hpp"

// MPPI (Williams et al. 2016/2018): softmax importance-weighted sampling on the
// shared receding-horizon cost J(U), verified via the closed-loop simulator on an
// open-field interior-goal scenario (goal-seeking around an obstacle), the
// executed-command limits, C++-own-seed reproducibility (same seed replays
// identically; a different seed diverges -- C++ uses std::mt19937, its own
// stream, so this asserts behavioral reproducibility, not a byte match to
// Python), and load-time parameter validation.

using namespace navigation;
using core::LocalTask;
using core::ParamSet;
using core::Pose;
using core::RobotState;
using core::TraceRecorder;
using local_planning::MppiPlanner;
using local_planning::SimConfig;
using local_planning::SimStatus;
using local_planning::simulate;

namespace {

constexpr const char* kMap = "maps/grid/open01.yaml";
constexpr const char* kScenario = "maps/scenarios/open01_s4.yaml";

std::string base_config_text() {
  std::ifstream in(test::repo_path("configs/local_planning/mppi.yaml"));
  std::ostringstream buf;
  buf << in.rdbuf();
  return buf.str();
}

// Overrides int param defaults by re-declaring them after the original document:
// ParamSet::from_yaml keeps the last occurrence of a repeated key, mirroring
// test_teb.cpp's config_with_int_override precedent.
ParamSet config_with_int_overrides(const std::vector<std::pair<std::string, int>>& overrides) {
  std::ostringstream extra;
  for (const auto& [name, value] : overrides) {
    extra << "\n  - {name: " << name << ", type: int, default: " << value
          << ", min: 0, max: 100000, description: override}";
  }
  extra << "\n";
  std::string path = test::write_temp("mppi_override.yaml", base_config_text() + extra.str());
  return ParamSet::from_yaml(path);
}

ParamSet real_config() { return ParamSet::from_yaml(test::repo_path("configs/local_planning/mppi.yaml")); }

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

// The `"state":[...]` array substring of a trace line -- the executed pose, free
// of the non-deterministic wall-clock `t` and the global `seq` counter, so two
// runs are compared on trajectory alone (mirrors the Python test's pose compare).
std::string state_field(const std::string& line) {
  size_t at = line.find("\"state\":");
  size_t lb = line.find('[', at);
  size_t rb = line.find(']', lb);
  return line.substr(lb, rb - lb + 1);
}

double find_data_value(const std::string& line, const std::string& key) {
  std::string needle = "\"" + key + "\":";
  size_t at = line.find(needle);
  if (at == std::string::npos) return std::nan("");
  size_t start = at + needle.size();
  size_t end = line.find_first_of(",}", start);
  return std::stod(line.substr(start, end - start));
}

local_planning::SimResult run(const ParamSet& params, TraceRecorder* recorder) {
  auto map = maps::load_map(test::repo_path(kMap));
  auto& grid = as_grid(*map);
  maps::Scenario sc = maps::load_scenario(test::repo_path(kScenario));
  MppiPlanner planner(params);
  RobotState start{Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
  LocalTask task{Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, sc.reference_path};
  return simulate(planner, grid, start, task, sim_config_from(params), recorder);
}

}  // namespace

// (a) closed-loop MPPI reaches an open-field interior goal --------------------
TEST(Mppi, ReachesGoalOpenField) {
  auto result = run(real_config(), nullptr);
  EXPECT_EQ(result.status, SimStatus::REACHED);
  EXPECT_TRUE(result.success);
  // Positive clearance means it threaded past the obstacle, not grazed a wall.
  EXPECT_GT(result.min_clearance, 0.0);
}

// (b) every executed command stays within the declared v/omega limits ---------
TEST(Mppi, CommandsRespectLimits) {
  // The sample box-clip, the box projection on the weighted update, and the
  // accel clamp on the executed command hold every tick, so a short capped run
  // exercises the invariant cheaply.
  ParamSet params = config_with_int_overrides({{"max_steps", 40}});
  std::ostringstream trace;
  TraceRecorder rec(trace);
  run(params, &rec);

  const double v_max = params.get_float("v_max");
  const double omega_max = params.get_float("omega_max");
  std::vector<std::string> moves = lines_with_event(split_lines(trace.str()), "robot_moved");
  ASSERT_FALSE(moves.empty());
  for (const std::string& line : moves) {
    EXPECT_LE(std::fabs(find_data_value(line, "v")), v_max + 1e-9);
    EXPECT_LE(std::fabs(find_data_value(line, "omega")), omega_max + 1e-9);
  }
}

// (c) a fixed seed replays identically; a different seed diverges --------------
TEST(Mppi, SameSeedDeterministic) {
  auto moved = [](int seed) {
    ParamSet params = config_with_int_overrides({{"seed", seed}, {"max_steps", 30}});
    std::ostringstream trace;
    TraceRecorder rec(trace);
    run(params, &rec);
    std::vector<std::string> states;
    for (const std::string& line : lines_with_event(split_lines(trace.str()), "robot_moved")) {
      states.push_back(state_field(line));
    }
    return states;
  };
  std::vector<std::string> a = moved(1);
  std::vector<std::string> b = moved(1);
  std::vector<std::string> c = moved(2);
  ASSERT_FALSE(a.empty());
  EXPECT_EQ(a, b);  // same seed -> identical closed-loop trajectory
  EXPECT_NE(a, c);  // different seed -> a different sampled noise stream
}

// (f) an out-of-range parameter fails load-time validation --------------------
TEST(Mppi, ParamValidation) {
  std::string bad = test::write_temp(
      "mppi_temperature.yaml",
      "algorithm: mppi\ncategory: local_planning\nparams:\n"
      "  - {name: temperature, type: float, default: 0.0, min: 0.001, max: 100.0, "
      "description: below min}\n");
  EXPECT_THROW(ParamSet::from_yaml(bad), std::runtime_error);
}
