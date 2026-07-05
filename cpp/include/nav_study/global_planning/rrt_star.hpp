#pragma once

#include <set>
#include <string>

#include "nav_study/core/planner.hpp"

namespace nav_study::global_planning {

// RRT* (Karaman & Frazzoli 2011): asymptotically optimal RRT. Chooses the
// min-cost parent within neighbor_radius and rewires the neighborhood; anytime,
// keeps improving the best path until max_iterations.
class RrtStarPlanner final : public core::SamplingPlanner {
 public:
  explicit RrtStarPlanner(core::ParamSet params) : core::SamplingPlanner(std::move(params)) {}
  std::string name() const override { return "rrt_star"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::SAMPLING_SPACE};
  }
  core::PlanResult<core::Point> plan(core::SamplingSpace<core::Point>& space,
                                     const core::Point& start, const core::Point& goal,
                                     core::TraceRecorder* recorder) override;
};

}  // namespace nav_study::global_planning
