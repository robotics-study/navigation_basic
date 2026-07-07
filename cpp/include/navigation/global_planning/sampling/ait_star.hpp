#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// AIT* — Adaptively Informed Trees (Strub & Gammell 2020, ICRA; extended IJRR
// 2022). Differs from BIT* in the cost-to-go heuristic: instead of the raw
// straight-line distance, h_hat is computed by a reverse search over the current
// random geometric graph, so it follows the graph's real connectivity (detours
// around obstacles) and is adaptive — edges found invalid during forward
// validation are permanently excluded from the graph the reverse search runs
// over, so the heuristic self-corrects as obstacles are discovered.
//
// Deliberate simplification vs. the paper: each batch recomputes the reverse
// search and the forward g/parent from scratch over all accumulated samples,
// rather than the paper's incremental bidirectional LPA* that repairs the reverse
// tree and reuses forward g values. This drops the LPA* incrementality (an
// optimisation of how the searches update) while preserving AIT*'s defining
// behaviour — a forward search guided by an obstacle-aware, self-correcting
// reverse heuristic. Same guarantee class as BIT*: probabilistically complete,
// almost-surely asymptotically optimal, anytime.
class AitStarPlanner final : public core::SamplingPlanner {
 public:
  explicit AitStarPlanner(core::ParamSet params) : core::SamplingPlanner(std::move(params)) {}
  std::string name() const override { return "ait_star"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::SAMPLING_SPACE};
  }
  core::PlanResult<core::Point> plan(core::SamplingSpace<core::Point>& space,
                                     const core::Point& start, const core::Point& goal,
                                     core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
