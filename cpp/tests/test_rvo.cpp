#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/core/params.hpp"
#include "navigation/core/types.hpp"
#include "navigation/local_planning/simulation.hpp"
#include "navigation/local_planning/velocity/agent_scenario.hpp"
#include "navigation/local_planning/velocity/agent_sim.hpp"
#include "navigation/local_planning/velocity/rvo.hpp"
#include "navigation/local_planning/velocity/velocity_obstacle.hpp"
#include "navigation/maps/loader.hpp"
#include "navigation/maps/occupancy_grid.hpp"
#include "test_util.hpp"

// RVO (van den Berg, Lin & Manocha 2008): VO with the cone apex shifted
// toward the midpoint of both agents' velocities so a symmetric encounter
// doesn't oscillate. Verified the same way as VO: reciprocal avoidance on the
// shared multi-body scenarios, and an honest failure when there is no room
// to avoid.

using namespace navigation;
using core::ParamSet;
using core::Point;
using core::Pose;
using core::RobotState;
using local_planning::AgentResult;
using local_planning::AgentSpec;
using local_planning::load_agent_scenario;
using local_planning::rvo_apex;
using local_planning::simulate_agents;
using local_planning::Rvo;
using local_planning::SimConfig;
using local_planning::SimStatus;
using local_planning::VelocityObstaclePlanner;

namespace {

std::string config_path() { return test::repo_path("configs/local_planning/rvo.yaml"); }

std::string read_file(const std::string& path) {
  std::ifstream in(path);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

ParamSet config_with_override(const std::string& param_name, const std::string& new_value) {
  std::string text = read_file(config_path());
  std::string needle = "name: " + param_name + "\n";
  size_t pos = text.find(needle);
  if (pos == std::string::npos) throw std::runtime_error("test: param not found: " + param_name);
  size_t default_pos = text.find("default:", pos);
  size_t line_end = text.find('\n', default_pos);
  text = text.substr(0, default_pos) + "default: " + new_value + text.substr(line_end);
  return ParamSet::from_yaml(test::write_temp("rvo_override.yaml", text));
}

SimConfig sim_config_from(const ParamSet& params) {
  return SimConfig{params.get_float("control_dt"),     params.get_int("max_steps"),
                   params.get_float("goal_tolerance"), params.get_float("footprint_radius"),
                   params.get_int("stall_window"),     params.get_float("stall_distance")};
}

maps::OccupancyGrid2D& as_grid(core::MapBase& m) { return *dynamic_cast<maps::OccupancyGrid2D*>(&m); }

std::vector<AgentResult> run_scenario(const std::string& scenario_name, const ParamSet& params) {
  auto scenario =
      load_agent_scenario(test::repo_path("maps/scenarios/velocity/" + scenario_name + ".yaml"));
  auto map = maps::load_map(scenario.map_path);
  auto& grid = as_grid(*map);
  std::vector<Rvo> planners;
  planners.reserve(scenario.agents.size());
  for (size_t i = 0; i < scenario.agents.size(); ++i) planners.emplace_back(params);
  std::vector<VelocityObstaclePlanner*> planner_ptrs;
  planner_ptrs.reserve(planners.size());
  for (Rvo& p : planners) planner_ptrs.push_back(&p);
  return simulate_agents(planner_ptrs, scenario.agents, grid, sim_config_from(params));
}

}  // namespace

// --- (a) reciprocal avoidance succeeds on the shared multi-body scenarios ----
TEST(Rvo, HeadOnTwoAgentsAvoidAndReach) {
  auto params = ParamSet::from_yaml(config_path());
  auto results = run_scenario("head_on", params);
  for (const AgentResult& r : results) {
    EXPECT_NE(r.status, SimStatus::COLLISION);
    EXPECT_GE(r.min_pair_clearance, 0.0);
    EXPECT_EQ(r.status, SimStatus::REACHED);
  }
}

TEST(Rvo, CircleSwapFourAgentsAvoidAndReach) {
  auto params = ParamSet::from_yaml(config_path());
  auto results = run_scenario("circle_swap", params);
  for (const AgentResult& r : results) {
    EXPECT_NE(r.status, SimStatus::COLLISION);
    EXPECT_GE(r.min_pair_clearance, 0.0);
    EXPECT_EQ(r.status, SimStatus::REACHED);
  }
}

// --- (b) an honest failure: no room to avoid a head-on scripted mover --------
TEST(Rvo, NarrowCorridorHeadOnMoverFailsHonestly) {
  auto params = config_with_override("max_steps", "300");
  const double resolution = 0.1;
  const int band_rows = 8;  // 0.8 m: below the 1.2 m two 0.3 m-radius bodies need to pass
  const int margin_rows = 10;
  const int cols = 100;
  std::vector<bool> free_cells(static_cast<size_t>(band_rows + 2 * margin_rows) * cols, false);
  for (int r = margin_rows; r < margin_rows + band_rows; ++r) {
    for (int c = 0; c < cols; ++c) free_cells[static_cast<size_t>(r) * cols + c] = true;
  }
  maps::OccupancyGrid2D grid(band_rows + 2 * margin_rows, cols, resolution, 0.0, 0.0,
                             std::move(free_cells));
  double corridor_y =
      grid.cell_to_world(core::Cell{margin_rows + band_rows / 2, 0}).y;

  AgentSpec planner_spec{RobotState{Pose{1.0, corridor_y, 0.0}, 0.0, 0.0},
                        Pose{9.0, corridor_y, 0.0}, 0.3, std::nullopt};
  AgentSpec mover_spec{RobotState{Pose{9.0, corridor_y, 0.0}, 0.0, 0.0}, Pose{1.0, corridor_y, 0.0},
                      0.3, Point{-1.0, 0.0}};

  Rvo planner(params);
  std::vector<VelocityObstaclePlanner*> planners{&planner, nullptr};
  std::vector<AgentSpec> specs{planner_spec, mover_spec};
  auto results = simulate_agents(planners, specs, grid, sim_config_from(params));

  EXPECT_TRUE(results[0].status == SimStatus::COLLISION || results[0].status == SimStatus::STALLED);
  EXPECT_NE(results[0].status, SimStatus::REACHED);
}

// --- (c) parameter validation failures at load time ---------------------------
TEST(Rvo, TimeHorizonAtZeroRejectedAtLoadTime) {
  std::string bad = test::write_temp(
      "rvo_time_horizon.yaml",
      "algorithm: rvo\ncategory: local_planning\nparams:\n"
      "  - {name: time_horizon, type: float, default: 0.0, min: 0.05, max: 20.0, description: at zero}\n");
  EXPECT_THROW(ParamSet::from_yaml(bad), std::runtime_error);
}

TEST(Rvo, SpeedSamplesZeroRejectedAtLoadTime) {
  std::string bad = test::write_temp(
      "rvo_speed_samples.yaml",
      "algorithm: rvo\ncategory: local_planning\nparams:\n"
      "  - {name: speed_samples, type: int, default: 0, min: 1, max: 50, description: at zero}\n");
  EXPECT_THROW(ParamSet::from_yaml(bad), std::runtime_error);
}

// --- (d) determinism: identical rerun -> bit-identical per-agent trajectories -
TEST(Rvo, RerunIsDeterministic) {
  auto params = ParamSet::from_yaml(config_path());
  auto scenario = load_agent_scenario(test::repo_path("maps/scenarios/velocity/circle_swap.yaml"));
  auto map = maps::load_map(scenario.map_path);
  auto& grid = as_grid(*map);
  SimConfig config = sim_config_from(params);

  auto run_once = [&]() {
    std::vector<Rvo> storage;
    storage.reserve(scenario.agents.size());
    for (size_t i = 0; i < scenario.agents.size(); ++i) storage.emplace_back(params);
    std::vector<VelocityObstaclePlanner*> ptrs;
    ptrs.reserve(storage.size());
    for (Rvo& p : storage) ptrs.push_back(&p);
    return simulate_agents(ptrs, scenario.agents, grid, config);
  };

  auto first = run_once();
  auto second = run_once();

  ASSERT_EQ(second.size(), first.size());
  for (size_t i = 0; i < first.size(); ++i) {
    EXPECT_EQ(second[i].status, first[i].status);
    EXPECT_EQ(second[i].steps, first[i].steps);
    ASSERT_EQ(second[i].trajectory.size(), first[i].trajectory.size());
    for (size_t j = 0; j < first[i].trajectory.size(); ++j) {
      EXPECT_NEAR(first[i].trajectory[j].x, second[i].trajectory[j].x, 1e-9);
      EXPECT_NEAR(first[i].trajectory[j].y, second[i].trajectory[j].y, 1e-9);
      EXPECT_NEAR(first[i].trajectory[j].theta, second[i].trajectory[j].theta, 1e-9);
    }
  }
}

// --- reciprocity=0 collapses onto plain VO (behavioral contract, not an echo) -
TEST(Rvo, ReciprocityZeroMatchesPlainVoApex) {
  core::Point v_self{1.0, 0.5};
  core::Point v_other{-0.5, 0.2};
  core::Point at_zero = rvo_apex(v_self, v_other, 0.0);
  core::Point at_one = rvo_apex(v_self, v_other, 1.0);
  EXPECT_NEAR(at_zero.x, v_other.x, 1e-9);
  EXPECT_NEAR(at_zero.y, v_other.y, 1e-9);
  EXPECT_NEAR(at_one.x, v_self.x, 1e-9);
  EXPECT_NEAR(at_one.y, v_self.y, 1e-9);
}
