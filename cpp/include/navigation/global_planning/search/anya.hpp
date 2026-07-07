#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// Anya: optimal Euclidean any-angle pathfinding via interval search (Harabor,
// Grastien, Öz & Aksakalli, JAIR 56, 2016). Unlike Theta*/Visibility A* (which pin
// turning points to cell CENTRES and are only any-angle approximations), Anya
// searches over (root, interval) nodes whose turning points are grid CORNERS
// (vertices). A shortest Euclidean path in a blocked-cell domain is a taut string
// that bends only at convex obstacle corners, so rooting turns exactly there lets
// Anya return the TRUE continuous Euclidean shortest any-angle path, not a
// cell-centre approximation.
//
// A search node is a corner root plus the visibility intervals it projects across
// the grid; expanding a root sweeps its visibility row by row (cone successors)
// and along its own row (flat successors), reaching successor obstacle corners.
// g(root) is the accumulated start->root Euclidean distance; the frontier is
// ordered by f = g(root) + ||root - goal|| (admissible straight-line bound). Path
// cost is Euclidean on cell-index deltas, matching the Theta*/Visibility model and
// identical across the Python mirror. Occupancy is observed only via the
// LineOfSightSpace capability's neighbors() (the reachable free component); the
// corner-level line-of-sight / projection is computed here (the capability's
// line_of_sight answers only cell-centre pairs, which cannot express corner turns).
class AnyaPlanner final : public core::LineOfSightPlanner {
 public:
  explicit AnyaPlanner(core::ParamSet params)
      : core::LineOfSightPlanner(std::move(params)) {}
  std::string name() const override { return "anya"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE, core::Capability::LINE_OF_SIGHT_SPACE};
  }
  core::PlanResult<core::Cell> plan(core::LineOfSightSpace<core::Cell>& space,
                                    const core::Cell& start, const core::Cell& goal,
                                    core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
