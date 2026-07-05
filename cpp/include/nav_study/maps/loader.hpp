#pragma once

#include <memory>
#include <string>

#include "nav_study/core/capabilities.hpp"
#include "nav_study/core/types.hpp"

namespace nav_study::maps {

// Single-agent problem definition resolved from a scenario yaml. start/goal are
// world coordinates for grid/continuous maps.
struct Scenario {
  std::string map_path;  // absolute, resolved relative to the scenario file
  core::Point start;
  core::Point goal;
};

// Dispatches on the yaml `type` field. Only occupancy_grid is implemented in
// this task; other types throw a clear error.
std::unique_ptr<core::MapBase> load_map(const std::string& path, unsigned seed = 0,
                                        int connectivity = 8);

Scenario load_scenario(const std::string& path);

}  // namespace nav_study::maps
