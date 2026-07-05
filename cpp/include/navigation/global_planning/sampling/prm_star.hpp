#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// PRM* — asymptotically optimal PRM (Karaman & Frazzoli 2011). Identical to PRM
// except the connection radius is not fixed: it shrinks with the sample count as
// r_n = gamma*(log n / n)^(1/2), keeping the expected neighbour count at
// Theta(log n), which is what buys almost-sure convergence to the optimal path.
class PrmStarPlanner final : public core::SamplingPlanner {
 public:
  explicit PrmStarPlanner(core::ParamSet params) : core::SamplingPlanner(std::move(params)) {}
  std::string name() const override { return "prm_star"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::SAMPLING_SPACE};
  }
  core::PlanResult<core::Point> plan(core::SamplingSpace<core::Point>& space,
                                     const core::Point& start, const core::Point& goal,
                                     core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
