#pragma once

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "navigation/core/params.hpp"
#include "navigation/core/planner.hpp"
#include "navigation/core/trace.hpp"
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

}  // namespace demo
