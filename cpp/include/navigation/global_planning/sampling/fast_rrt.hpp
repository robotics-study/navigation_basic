#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// Fast-RRT (Wu et al. 2021, Applied Sciences 11(24):11777). Improved-RRT plus
// Fast-Optimal: Fast-Sampling only accepts samples in unreached space, Random
// Steering retries blocked extensions in random directions, and the feasible
// path is shortcut-pruned and improved anytime.
class FastRrtPlanner final : public core::SamplingPlanner {
 public:
  explicit FastRrtPlanner(core::ParamSet params) : core::SamplingPlanner(std::move(params)) {}
  std::string name() const override { return "fast_rrt"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::SAMPLING_SPACE};
  }
  core::PlanResult<core::Point> plan(core::SamplingSpace<core::Point>& space,
                                     const core::Point& start, const core::Point& goal,
                                     core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
