#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// SST / SST* — Stable Sparse RRT (Li, Littlefield & Bekris 2016). A kinodynamic
// sampling planner that needs NO steering / BVP solver: it grows a tree by forward-
// propagating a RANDOM control for a random duration (unicycle dynamics) and
// collision-checking the arc. A WITNESS set with sparsification radius delta_s keeps
// at most one ACTIVE representative (the lowest-cost node) per witness ball and prunes
// dominated leaves, bounding the active-node count and improving the incumbent cost
// monotonically (anytime). SST* shrinks delta_BN / delta_s over iterations to recover
// asymptotic optimality. required_capabilities stays {SAMPLING_SPACE} — the planner
// owns its dynamics, the map only answers state / motion validity.
class SstPlanner final : public core::SamplingPlanner {
 public:
  explicit SstPlanner(core::ParamSet params) : core::SamplingPlanner(std::move(params)) {}
  std::string name() const override { return "sst"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::SAMPLING_SPACE};
  }
  core::PlanResult<core::Point> plan(core::SamplingSpace<core::Point>& space,
                                     const core::Point& start, const core::Point& goal,
                                     core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
