#pragma once

#include <memory>
#include <string>

#include "navigation/core/capabilities.hpp"
#include "navigation/core/types.hpp"

namespace navigation::maps {

// Single-agent problem definition resolved from a scenario yaml. start/goal are
// world coordinates for grid/continuous maps.
struct Scenario {
  std::string map_path;  // absolute, resolved relative to the scenario file
  core::Point start;
  core::Point goal;
  // Optional SE(2) start/goal heading (radians, world) for kinodynamic planners.
  // Defaulted so existing scenarios (no theta) load unchanged; discrete/sampling ignore.
  double start_theta = 0.0;
  double goal_theta = 0.0;
};

// Dispatches on the yaml `type` field. Only occupancy_grid is implemented in
// this task; other types throw a clear error.
std::unique_ptr<core::MapBase> load_map(const std::string& path, unsigned seed = 0,
                                        int connectivity = 8);

Scenario load_scenario(const std::string& path);

}  // namespace navigation::maps
