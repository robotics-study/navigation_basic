#pragma once

#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation/core/params.hpp"
#include "navigation/core/planner.hpp"
#include "navigation/core/trace.hpp"
#include "navigation/local_planning/simulation.hpp"
#include "navigation/local_planning/velocity/agent_scenario.hpp"
#include "navigation/local_planning/velocity/agent_sim.hpp"
#include "navigation/local_planning/velocity/velocity_obstacle.hpp"
#include "navigation/maps/loader.hpp"
#include "navigation/maps/occupancy_grid.hpp"

// Assembly-only scaffold shared by the demo executables: parse CLI args, wire a
// planner to a loaded map + scenario, emit a trace, and print a one-line metrics
// summary. No planning logic lives here.
namespace demo {

struct Args {
  std::string map;
  std::string scenario;
  std::string params;
  std::string trace;
  unsigned seed = 0;
  int connectivity = 8;
};

inline Args parse_args(int argc, char** argv) {
  Args a;
  auto need = [&](int& i) -> std::string {
    if (i + 1 >= argc) throw std::runtime_error("demo: missing value for " + std::string(argv[i]));
    return argv[++i];
  };
  for (int i = 1; i < argc; ++i) {
    std::string f = argv[i];
    if (f == "--map") {
      a.map = need(i);
    } else if (f == "--scenario") {
      a.scenario = need(i);
    } else if (f == "--params") {
      a.params = need(i);
    } else if (f == "--trace") {
      a.trace = need(i);
    } else if (f == "--seed") {
      a.seed = static_cast<unsigned>(std::stoul(need(i)));
    } else if (f == "--connectivity") {
      a.connectivity = std::stoi(need(i));
    } else {
      throw std::runtime_error("demo: unknown flag " + f);
    }
  }
  if (a.map.empty() || a.scenario.empty() || a.params.empty() || a.trace.empty()) {
    throw std::runtime_error("demo: --map --scenario --params --trace are required");
  }
  return a;
}

inline navigation::maps::OccupancyGrid2D& as_grid(navigation::core::MapBase& map) {
  auto* grid = dynamic_cast<navigation::maps::OccupancyGrid2D*>(&map);
  if (!grid) throw std::runtime_error("demo: map is not an occupancy grid");
  return *grid;
}

// Seed sampling from the config when present (reproducibility), else the CLI seed.
inline unsigned resolve_seed(const Args& a, const navigation::core::ParamSet& params) {
  return params.has("seed") ? static_cast<unsigned>(params.get_int("seed")) : a.seed;
}

// Templated on the planner so any discrete-family planner binds: the loaded
// OccupancyGrid2D& is passed as whichever Space& the planner's plan() wants
// (DiscreteSpace<Cell>& for A*/Dijkstra/BFS, LineOfSightSpace<Cell>& for Theta*,
// DynamicGridSpace<Cell>& for D* Lite).
// Existing callers deduce Planner unchanged.
template <class Planner>
inline int run_discrete(const Args& a, const navigation::core::ParamSet& params, Planner& planner) {
  auto map = navigation::maps::load_map(a.map, resolve_seed(a, params), a.connectivity);
  auto& grid = as_grid(*map);
  navigation::maps::Scenario sc = navigation::maps::load_scenario(a.scenario);
  navigation::core::Cell start = grid.world_to_cell(sc.start.x, sc.start.y);
  navigation::core::Cell goal = grid.world_to_cell(sc.goal.x, sc.goal.y);

  std::ofstream fs(a.trace);
  if (!fs) throw std::runtime_error("demo: cannot open trace file " + a.trace);
  navigation::core::TraceRecorder rec(fs);
  rec.planning_started(planner.name(), a.map, params.values());
  auto res = planner.plan(grid, start, goal, &rec);

  std::cout << "{\"algorithm\":\"" << planner.name() << "\",\"success\":"
            << (res.success ? "true" : "false") << ",\"path_cost\":" << res.cost
            << ",\"path_len\":" << res.path.size() << ",\"expanded_nodes\":"
            << res.stats.expanded_nodes << "}\n";
  return 0;
}

inline int run_sampling(const Args& a, const navigation::core::ParamSet& params,
                        navigation::core::SamplingPlanner& planner) {
  auto map = navigation::maps::load_map(a.map, resolve_seed(a, params), a.connectivity);
  auto& grid = as_grid(*map);
  navigation::maps::Scenario sc = navigation::maps::load_scenario(a.scenario);

  std::ofstream fs(a.trace);
  if (!fs) throw std::runtime_error("demo: cannot open trace file " + a.trace);
  navigation::core::TraceRecorder rec(fs);
  rec.planning_started(planner.name(), a.map, params.values());
  auto res = planner.plan(grid, sc.start, sc.goal, &rec);

  std::cout << "{\"algorithm\":\"" << planner.name() << "\",\"success\":"
            << (res.success ? "true" : "false") << ",\"path_cost\":" << res.cost
            << ",\"path_len\":" << res.path.size() << ",\"samples\":" << res.stats.samples
            << ",\"tree_size\":" << res.stats.tree_size << ",\"iterations\":"
            << res.stats.iterations << "}\n";
  return 0;
}

// Kinodynamic (SE(2)) demo: builds Pose start/goal directly from the scenario (world
// x, y + optional start_theta/goal_theta) and binds the loaded grid as an
// SE2CollisionSpace<Pose>&. No world_to_cell — the state is a continuous Pose.
template <class Planner>
inline int run_kinodynamic(const Args& a, const navigation::core::ParamSet& params,
                           Planner& planner) {
  auto map = navigation::maps::load_map(a.map, resolve_seed(a, params), a.connectivity);
  auto& grid = as_grid(*map);  // binds to SE2CollisionSpace<Pose>&
  navigation::maps::Scenario sc = navigation::maps::load_scenario(a.scenario);
  navigation::core::Pose start{sc.start.x, sc.start.y, sc.start_theta};
  navigation::core::Pose goal{sc.goal.x, sc.goal.y, sc.goal_theta};

  std::ofstream fs(a.trace);
  if (!fs) throw std::runtime_error("demo: cannot open trace file " + a.trace);
  navigation::core::TraceRecorder rec(fs);
  rec.planning_started(planner.name(), a.map, params.values());
  auto res = planner.plan(grid, start, goal, &rec);

  std::cout << "{\"algorithm\":\"" << planner.name() << "\",\"success\":"
            << (res.success ? "true" : "false") << ",\"path_cost\":" << res.cost
            << ",\"path_len\":" << res.path.size() << ",\"expanded_nodes\":"
            << res.stats.expanded_nodes << "}\n";
  return 0;
}

// `SimStatus` is exhaustively matched (no default:) so adding a new terminal
// status trips -Wswitch here as a forcing function to update this table.
inline const char* sim_status_str(navigation::local_planning::SimStatus status) {
  using navigation::local_planning::SimStatus;
  switch (status) {
    case SimStatus::REACHED: return "reached";
    case SimStatus::COLLISION: return "collision";
    case SimStatus::STALLED: return "stalled";
    case SimStatus::TIMEOUT: return "timeout";
  }
  return "timeout";
}

// Local-planning demo assembly. Unlike run_discrete/run_sampling/run_kinodynamic
// (which receive an already-parsed Args and an already-constructed planner,
// because the planner's constructor needs the loaded ParamSet before the caller
// gets here), run_local parses argv and builds the planner itself via `factory`
// -- the closed-loop episode has no plan() result to summarize, so assembly
// instead wires a SimConfig from the yaml's shared sim-params block and hands
// the run to simulate(), which owns tick order and termination. `Factory` is any
// callable `(const ParamSet&) -> ConcretePlanner` (deduced, not std::function)
// so a demo main() stays a one-line call with no type to spell out.
template <class Factory>
inline int run_local(int argc, char** argv, const std::string& name, Factory factory) {
  Args a = parse_args(argc, argv);
  auto params = navigation::core::ParamSet::from_yaml(a.params);
  auto map = navigation::maps::load_map(a.map, resolve_seed(a, params), a.connectivity);
  auto& grid = as_grid(*map);
  navigation::maps::Scenario sc = navigation::maps::load_scenario(a.scenario);
  auto planner = factory(params);

  navigation::core::LocalTask task{
      navigation::core::Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, sc.reference_path};
  if (planner.requires_reference_path() && task.reference_path.empty()) {
    throw std::runtime_error(name + " requires a reference_path, but " + a.scenario +
                             " declares none");
  }

  navigation::local_planning::SimConfig config{
      params.get_float("control_dt"),     params.get_int("max_steps"),
      params.get_float("goal_tolerance"), params.get_float("footprint_radius"),
      params.get_int("stall_window"),     params.get_float("stall_distance")};
  navigation::core::RobotState start{
      navigation::core::Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};

  std::ofstream fs(a.trace);
  if (!fs) throw std::runtime_error("demo: cannot open trace file " + a.trace);
  navigation::core::TraceRecorder rec(fs);
  rec.planning_started(name, a.map, params.values(), a.scenario);
  auto result = navigation::local_planning::simulate(planner, grid, start, task, config, &rec);

  std::cout << "{\"algorithm\":\"" << name << "\",\"status\":\"" << sim_status_str(result.status)
            << "\",\"success\":" << (result.success ? "true" : "false")
            << ",\"time_to_goal\":" << result.time_to_goal
            << ",\"distance_traveled\":" << result.distance_traveled
            << ",\"min_clearance\":" << result.min_clearance << ",\"steps\":" << result.steps
            << "}\n";
  return 0;
}

// Multi-agent assembly for the velocity-obstacle family (VO/RVO/ORCA): wires
// an AgentScenario (N bodies, one goal each, some possibly scripted
// non-cooperative movers) instead of run_local's single-agent Scenario, and
// hands the run to simulate_agents, whose tick loop owns termination + the
// per-body trace order. `Factory` is any callable `(const ParamSet&) ->
// ConcretePlanner` (deduced, not std::function), mirroring run_local.
template <class Factory>
inline int run_agents(int argc, char** argv, const std::string& name, Factory factory) {
  Args a = parse_args(argc, argv);
  auto params = navigation::core::ParamSet::from_yaml(a.params);
  auto map = navigation::maps::load_map(a.map, resolve_seed(a, params), a.connectivity);
  auto& grid = as_grid(*map);
  auto scenario = navigation::local_planning::load_agent_scenario(a.scenario);

  navigation::local_planning::SimConfig config{
      params.get_float("control_dt"),     params.get_int("max_steps"),
      params.get_float("goal_tolerance"), params.get_float("footprint_radius"),
      params.get_int("stall_window"),     params.get_float("stall_distance")};

  // `planner_ptrs[k]` is nullptr iff the agent is scripted (non-cooperative
  // mover), matching simulate_agents' contract. `owned_planners` is reserved
  // to its upper bound (every agent could be planner-driven) up front so
  // later push_back calls never reallocate and invalidate the pointers taken
  // into planner_ptrs.
  using ConcretePlanner = decltype(factory(params));
  std::vector<ConcretePlanner> owned_planners;
  owned_planners.reserve(scenario.agents.size());
  std::vector<navigation::local_planning::VelocityObstaclePlanner*> planner_ptrs;
  planner_ptrs.reserve(scenario.agents.size());
  for (const auto& spec : scenario.agents) {
    if (spec.scripted_velocity.has_value()) {
      planner_ptrs.push_back(nullptr);
    } else {
      owned_planners.push_back(factory(params));
      planner_ptrs.push_back(&owned_planners.back());
    }
  }

  std::ofstream fs(a.trace);
  if (!fs) throw std::runtime_error("demo: cannot open trace file " + a.trace);
  navigation::core::TraceRecorder rec(fs);
  rec.planning_started(name, a.map, params.values());
  auto results =
      navigation::local_planning::simulate_agents(planner_ptrs, scenario.agents, grid, config, &rec);

  std::cout << "[";
  for (size_t k = 0; k < results.size(); ++k) {
    if (k) std::cout << ",";
    char clearance[32];
    std::snprintf(clearance, sizeof(clearance), "%.4f", results[k].min_pair_clearance);
    std::cout << "{\"agent\":" << k << ",\"status\":\"" << sim_status_str(results[k].status)
              << "\",\"steps\":" << results[k].steps << ",\"min_pair_clearance\":" << clearance
              << "}";
  }
  std::cout << "]\n";
  return 0;
}

}  // namespace demo
