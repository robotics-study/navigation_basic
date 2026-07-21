#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// Fast-RRT (Wu et al. 2021, Applied Sciences 11(24):11777). Improved-RRT plus
// Fast-Optimal: Fast-Sampling only accepts samples in unreached space, Random
// Steering retries blocked extensions in random directions, and the feasible
// path is shortcut-pruned and improved anytime.
// Divergence from the paper (deliberate): Wu et al. re-initialise a plain RRT per
// outer iteration and fuse the resulting multiple paths (Alg. 3/4/7); here one
// persistent RRT*-style tree (choose-parent + rewire) replaces that framework and a
// single-path triangle-inequality shortcut replaces the multi-path fusion. Asymptotic
// optimality is carried by the RRT* rewiring. Fast-Sampling / Random Steering follow
// the paper (Alg. 5/6, resample loop capped by steering_attempts).
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
