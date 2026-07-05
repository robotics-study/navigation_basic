#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// Theta* (Nash, Daniel, Koenig & Felner 2007): any-angle path planning on grids.
// Like A* but on each relaxation it tries a straight line-of-sight shortcut from
// the expanded node's parent (path 2), so returned paths hug true straight
// segments instead of being locked to grid edges. The heuristic is Euclidean
// (straight-line), consistent with the any-angle g-values; w > 1 is weighted
// Theta* (Pohl 1970).
class ThetaStarPlanner final : public core::LineOfSightPlanner {
 public:
  explicit ThetaStarPlanner(core::ParamSet params)
      : core::LineOfSightPlanner(std::move(params)) {}
  std::string name() const override { return "theta_star"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE, core::Capability::LINE_OF_SIGHT_SPACE};
  }
  core::PlanResult<core::Cell> plan(core::LineOfSightSpace<core::Cell>& space,
                                    const core::Cell& start, const core::Cell& goal,
                                    core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
