#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// EIT* — Effort Informed Trees (Strub & Gammell 2022, IJRR). Extends the AIT*
// idea (a reverse search over the random geometric graph yields an adaptive
// cost-to-go heuristic for a forward best-first search, with edges found in
// collision fed back) by ALSO estimating validation effort — how expensive the
// remaining path is to collision-check — so that among near-equal-cost candidates
// the forward search prefers cheaper-to-validate ones, surfacing feasible
// solutions sooner.
//
// Faithful core implemented here: per batch, grow the RGG with informed samples
// (Gammell et al. 2014), filter against a persistent set of edges found in
// collision, run two independent single-criterion Dijkstra passes from the goal
// (cost-to-go via distance, effort-to-go via a step_size discretization proxy),
// then a lazy-deletion forward best-first search keyed lexicographically by
// (g + h_hat, effort_g + e_hat) — cost primary, effort the tie-break.
//
// Documented simplifications vs. the paper: the reverse search is recomputed each
// batch rather than repaired incrementally; the two heuristics come from two clean
// independent Dijkstra passes rather than a joint treatment; effort is the simple
// distance/step_size proxy rather than a learned validator-cost model.
class EitStarPlanner final : public core::SamplingPlanner {
 public:
  explicit EitStarPlanner(core::ParamSet params) : core::SamplingPlanner(std::move(params)) {}
  std::string name() const override { return "eit_star"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::SAMPLING_SPACE};
  }
  core::PlanResult<core::Point> plan(core::SamplingSpace<core::Point>& space,
                                     const core::Point& start, const core::Point& goal,
                                     core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
