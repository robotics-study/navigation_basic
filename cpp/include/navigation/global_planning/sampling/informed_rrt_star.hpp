#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// Informed RRT* (Gammell, Srinivasa & Barfoot 2014): RRT* with informed ellipse
// sampling. The tree-growth mechanics (choose-parent + rewire + anytime incumbent
// tracking, Karaman & Frazzoli 2011) are identical to RRT*; only the sampling step
// changes. Once an incumbent exists, samples are drawn from the ellipse (foci
// start/goal, transverse diameter = best cost) so they focus on the region that can
// still improve the incumbent — faster/tighter convergence, same guarantee.
class InformedRrtStarPlanner final : public core::SamplingPlanner {
 public:
  explicit InformedRrtStarPlanner(core::ParamSet params)
      : core::SamplingPlanner(std::move(params)) {}
  std::string name() const override { return "informed_rrt_star"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::SAMPLING_SPACE};
  }
  core::PlanResult<core::Point> plan(core::SamplingSpace<core::Point>& space,
                                     const core::Point& start, const core::Point& goal,
                                     core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
