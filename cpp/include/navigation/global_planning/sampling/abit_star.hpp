#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// ABIT* — Advanced Batch Informed Trees (Strub & Gammell 2020). Extends BIT*
// (Gammell, Srinivasa & Barfoot 2015) — same batch RGG, vertex/edge queues, lazy
// edge collision checks, informed-ellipse sampling, prune, and subtree cost
// propagation — with two mechanisms that trade a bounded, closing suboptimality
// gap for a faster first solution and cheaper batches:
//  1. Inflation factor ε_infl >= 1 inflates the cost-to-go term of the queue keys
//     (weighted-A*/ARA* over the RGG; Likhachev, Gordon & Thrun 2003), so early
//     batches order edges greedily toward the goal. ε_infl decays
//     inflation_factor -> inflation_final across batches.
//  2. Truncation factor ε_trunc >= 1 stops a batch once no edge can improve the
//     incumbent past c_best / ε_trunc, skipping lazy collision checks on edges
//     that can only shave the last sliver of cost. ε_trunc decays
//     truncation_factor -> 1.0 across batches.
// The admissibility gates stay un-inflated, so with ε_infl = ε_trunc = 1 the last
// batch reduces exactly to BIT* and the planner stays asymptotically optimal.
class AbitStarPlanner final : public core::SamplingPlanner {
 public:
  explicit AbitStarPlanner(core::ParamSet params) : core::SamplingPlanner(std::move(params)) {}
  std::string name() const override { return "abit_star"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::SAMPLING_SPACE};
  }
  core::PlanResult<core::Point> plan(core::SamplingSpace<core::Point>& space,
                                     const core::Point& start, const core::Point& goal,
                                     core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
