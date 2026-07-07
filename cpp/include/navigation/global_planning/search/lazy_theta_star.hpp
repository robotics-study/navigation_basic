#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// Lazy Theta* (Nash & Koenig 2010): any-angle path planning that defers the
// line-of-sight check. Like Theta* (Nash et al. 2007) but on generating a
// successor it OPTIMISTICALLY assumes the grandparent is visible (Path 2) without
// checking; the single line-of-sight query is deferred to set_vertex when the
// vertex is popped, and the parent is repaired to the cheapest settled grid
// neighbour only if the assumption fails. One check per expanded vertex instead of
// per edge — same any-angle paths, fewer checks. w > 1 is weighted (Pohl 1970).
class LazyThetaStarPlanner final : public core::LineOfSightPlanner {
 public:
  explicit LazyThetaStarPlanner(core::ParamSet params)
      : core::LineOfSightPlanner(std::move(params)) {}
  std::string name() const override { return "lazy_theta_star"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE, core::Capability::LINE_OF_SIGHT_SPACE};
  }
  core::PlanResult<core::Cell> plan(core::LineOfSightSpace<core::Cell>& space,
                                    const core::Cell& start, const core::Cell& goal,
                                    core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
