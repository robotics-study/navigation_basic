#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// Jump Point Search (Harabor & Grastien 2011): A* over grid "jump points". On a
// uniform-cost 8-connected grid it breaks path symmetry by scanning each canonical
// direction until an obstacle / the goal / a cell with a forced neighbour, so a
// node's successors are those jump points instead of every neighbour. Returns the
// same optimal paths as 8-connected A* while expanding far fewer nodes.
//
// Binds to DynamicGridSpace: the jump primitive is a single-cell occupancy/bounds
// oracle (is_blocked = occupied OR out of bounds), exactly what DynamicGridSpace
// exposes. Octile costs are computed in the .cpp (the bound space has no heuristic),
// with the same (hi-lo)+sqrt2*lo formula as OccupancyGrid2D::heuristic so the
// returned cost equals the 8-connected A* optimum. This grid forbids corner-cutting,
// so the no-corner-cutting variant of the pruning rules is used.
class JpsPlanner final : public core::DynamicGridPlanner {
 public:
  explicit JpsPlanner(core::ParamSet params) : core::DynamicGridPlanner(std::move(params)) {}
  std::string name() const override { return "jps"; }
  std::set<core::Capability> required_capabilities() const override {
    // DISCRETE_SPACE: JPS is 8-connected grid graph search (same optimum as A*).
    // DYNAMIC_GRID_SPACE: is_blocked is the single-cell oracle the jumps need.
    return {core::Capability::DISCRETE_SPACE, core::Capability::DYNAMIC_GRID_SPACE};
  }
  core::PlanResult<core::Cell> plan(core::DynamicGridSpace<core::Cell>& space,
                                    const core::Cell& start, const core::Cell& goal,
                                    core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
