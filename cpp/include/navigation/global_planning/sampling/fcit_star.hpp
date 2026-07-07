#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// FCIT* — Fully Connected Informed Trees (Wilson, Strub & Gammell 2025, ICRA).
// Instead of restricting the candidate graph to a shrinking radius-neighbour RGG
// (as BIT*/AIT*/EIT* do to bound edge count), FCIT* searches the FULLY CONNECTED
// graph over the current informed batch — every accumulated sample pairs with
// every other — and runs an informed best-first search over it directly, with the
// same lazy edge validation. Dropping the radius trades more (cheap,
// un-collision-checked) candidate edges for a search that can take shortcuts a
// radius graph would miss. A reverse Dijkstra from the goal supplies the adaptive
// cost-to-go heuristic (the AIT* idea, Strub & Gammell 2020). Probabilistically
// complete, anytime, asymptotically optimal.
//
// Simplified core (documented so the scope is explicit): the reverse search and
// the forward tree are recomputed fresh each batch rather than repaired
// incrementally; only c_best and the invalid-motion set persist across batches.
// The sample budget is kept modest since the complete graph has O(n^2) edges.
class FcitStarPlanner final : public core::SamplingPlanner {
 public:
  explicit FcitStarPlanner(core::ParamSet params) : core::SamplingPlanner(std::move(params)) {}
  std::string name() const override { return "fcit_star"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::SAMPLING_SPACE};
  }
  core::PlanResult<core::Point> plan(core::SamplingSpace<core::Point>& space,
                                     const core::Point& start, const core::Point& goal,
                                     core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
