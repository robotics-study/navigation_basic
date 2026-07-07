#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// RRT-Connect (Kuffner & LaValle 2000). Bidirectional single-query planner: one
// tree grows from the start, another from the goal; each iteration EXTENDs one
// tree toward a random sample and greedily CONNECTs the other toward the new
// node until the two trees meet. Feasible, not optimal.
class RrtConnectPlanner final : public core::SamplingPlanner {
 public:
  explicit RrtConnectPlanner(core::ParamSet params) : core::SamplingPlanner(std::move(params)) {}
  std::string name() const override { return "rrt_connect"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::SAMPLING_SPACE};
  }
  core::PlanResult<core::Point> plan(core::SamplingSpace<core::Point>& space,
                                     const core::Point& start, const core::Point& goal,
                                     core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
