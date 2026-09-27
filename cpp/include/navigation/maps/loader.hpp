#pragma once

#include <memory>
#include <string>
#include <vector>

#include "navigation/core/capabilities.hpp"
#include "navigation/core/types.hpp"

namespace navigation::maps {

// One cost-change batch (spec/map_formats.md `revisions`): every cell containing
// one of `cells` (world coords) flips to `blocked` between two plan() rounds. Only
// lifelong replanners (LPA*) consume revisions; static planners ignore the field.
struct Revision {
  std::vector<core::Point> cells;
  bool blocked = true;
};

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
  // Optional reference path (world points) for path-tracking local planners.
  // Empty = no reference path; defaulted so existing scenarios load unchanged.
  std::vector<core::Point> reference_path;
  // Optional cost-change batches for lifelong replanners (LPA*). World coords like
  // start/goal — the demo driver converts to cells via world_to_cell. Defaulted to
  // empty so every pre-LPA* scenario loads unchanged.
  std::vector<Revision> revisions;
};

// Dispatches on the yaml `type` field. Only occupancy_grid is implemented in
// this task; other types throw a clear error.
std::unique_ptr<core::MapBase> load_map(const std::string& path, unsigned seed = 0,
                                        int connectivity = 8);

Scenario load_scenario(const std::string& path);

}  // namespace navigation::maps
