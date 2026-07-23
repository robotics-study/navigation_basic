#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/core/params.hpp"
#include "navigation/core/types.hpp"
#include "navigation/local_planning/simulation.hpp"
#include "navigation/local_planning/velocity/agent_scenario.hpp"
#include "navigation/local_planning/velocity/agent_sim.hpp"
#include "navigation/local_planning/velocity/orca.hpp"
#include "navigation/local_planning/velocity/velocity_obstacle.hpp"
#include "navigation/maps/loader.hpp"
#include "navigation/maps/occupancy_grid.hpp"
#include "test_util.hpp"

// ORCA (van den Berg, Guy, Lin & Manocha 2011): an exact half-plane per
// obstacle plus a deterministic 2D linear program, falling back to a
// penetration-minimizing 3D solve when jointly infeasible. Verified via the
// multi-agent harness (reciprocal avoidance on the shared scenarios) and
// direct linear-program unit tests (closest-feasible-point selection, and
// the hot-path-never-raises fallback contract).

using namespace navigation;
using core::ParamSet;
using core::Point;
using local_planning::AgentResult;
using local_planning::HalfPlane;
using local_planning::linear_program_2d;
using local_planning::linear_program_3d;
using local_planning::load_agent_scenario;
using local_planning::Orca;
using local_planning::simulate_agents;
using local_planning::SimConfig;
using local_planning::SimStatus;
using local_planning::VelocityObstaclePlanner;

namespace {

std::string config_path() { return test::repo_path("configs/local_planning/orca.yaml"); }

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
  std::vector<Orca> planners;
  planners.reserve(scenario.agents.size());
  for (size_t i = 0; i < scenario.agents.size(); ++i) planners.emplace_back(params);
  std::vector<VelocityObstaclePlanner*> planner_ptrs;
  planner_ptrs.reserve(planners.size());
  for (Orca& p : planners) planner_ptrs.push_back(&p);
  return simulate_agents(planner_ptrs, scenario.agents, grid, sim_config_from(params));
}

}  // namespace

// --- (a) reciprocal avoidance succeeds on the shared multi-body scenarios ----
TEST(Orca, HeadOnTwoAgentsAvoidAndReach) {
  auto params = ParamSet::from_yaml(config_path());
  auto results = run_scenario("head_on", params);
  for (const AgentResult& r : results) {
    EXPECT_NE(r.status, SimStatus::COLLISION);
    EXPECT_GE(r.min_pair_clearance, 0.0);
    EXPECT_EQ(r.status, SimStatus::REACHED);
  }
}

TEST(Orca, CircleSwapFourAgentsAvoidAndReach) {
  auto params = ParamSet::from_yaml(config_path());
  auto results = run_scenario("circle_swap", params);
  for (const AgentResult& r : results) {
    EXPECT_NE(r.status, SimStatus::COLLISION);
    EXPECT_GE(r.min_pair_clearance, 0.0);
    EXPECT_EQ(r.status, SimStatus::REACHED);
  }
}

// --- (b) / ORCA LP unit: feasible set -> closest point to v_pref -------------
TEST(Orca, LinearProgram2dSelectsClosestFeasiblePoint) {
  // Two half-planes (x >= 1, y >= 1) intersecting in a quarter-plane whose
  // nearest corner to v_pref=(0,0) is (1,1) -- verifies the LP actually
  // projects onto the feasible region rather than just clamping to max_speed.
  std::vector<HalfPlane> planes{{Point{1.0, 0.0}, Point{1.0, 0.0}},
                                {Point{0.0, 1.0}, Point{0.0, 1.0}}};
  auto lp = linear_program_2d(planes, Point{0.0, 0.0}, 10.0);
  EXPECT_TRUE(lp.ok);
  EXPECT_EQ(lp.fail_index, static_cast<int>(planes.size()));
  EXPECT_NEAR(lp.velocity.x, 1.0, 1e-9);
  EXPECT_NEAR(lp.velocity.y, 1.0, 1e-9);
}

// --- (b) / ORCA LP unit: over-constrained set -> 3D fallback never raises ----
TEST(Orca, LinearProgram3dFallbackNeverRaisesOnOverConstrainedSet) {
  // A third half-plane (x + y <= -6) makes the intersection with (x>=1, y>=1)
  // empty: linear_program_2d must report failure, and linear_program_3d must
  // still return a finite Point (penetration-min fallback) instead of
  // raising.
  std::vector<HalfPlane> planes{{Point{1.0, 0.0}, Point{1.0, 0.0}},
                                {Point{0.0, 1.0}, Point{0.0, 1.0}},
                                {Point{-3.0, -3.0}, Point{-1.0, -1.0}}};
  auto lp = linear_program_2d(planes, Point{0.0, 0.0}, 10.0);
  EXPECT_FALSE(lp.ok);
  EXPECT_LT(lp.fail_index, static_cast<int>(planes.size()));

  core::Point fallback = linear_program_3d(planes, lp.fail_index, Point{0.0, 0.0}, 10.0);
  EXPECT_TRUE(std::isfinite(fallback.x));
  EXPECT_TRUE(std::isfinite(fallback.y));
}

// --- (c) parameter validation failure at load time ----------------------------
TEST(Orca, TimeHorizonAtZeroRejectedAtLoadTime) {
  std::string bad = test::write_temp(
      "orca_time_horizon.yaml",
      "algorithm: orca\ncategory: local_planning\nparams:\n"
      "  - {name: time_horizon, type: float, default: 0.0, min: 0.05, max: 20.0, description: at zero}\n");
  EXPECT_THROW(ParamSet::from_yaml(bad), std::runtime_error);
}

// --- (d) determinism: identical rerun -> bit-identical per-agent trajectories -
TEST(Orca, RerunIsDeterministic) {
  auto params = ParamSet::from_yaml(config_path());
  auto scenario = load_agent_scenario(test::repo_path("maps/scenarios/velocity/circle_swap.yaml"));
  auto map = maps::load_map(scenario.map_path);
  auto& grid = as_grid(*map);
  SimConfig config = sim_config_from(params);

  auto run_once = [&]() {
    std::vector<Orca> storage;
    storage.reserve(scenario.agents.size());
    for (size_t i = 0; i < scenario.agents.size(); ++i) storage.emplace_back(params);
    std::vector<VelocityObstaclePlanner*> ptrs;
    ptrs.reserve(storage.size());
    for (Orca& p : storage) ptrs.push_back(&p);
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
