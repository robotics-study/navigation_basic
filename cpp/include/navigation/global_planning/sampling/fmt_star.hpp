#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// FMT* — Fast Marching Tree (Janson, Schmerling, Clark & Pavone 2015). Marches a
// single tree outward from the start over one fixed batch of samples, in order of
// cost-to-come, using lazy dynamic programming: at each step it takes the
// lowest-cost frontier node z and, for every unvisited sample near z, connects it
// through its locally cheapest open neighbour — checking collision on only that
// one edge. One batch, no rewiring.
class FmtStarPlanner final : public core::SamplingPlanner {
 public:
  explicit FmtStarPlanner(core::ParamSet params) : core::SamplingPlanner(std::move(params)) {}
  std::string name() const override { return "fmt_star"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::SAMPLING_SPACE};
  }
  core::PlanResult<core::Point> plan(core::SamplingSpace<core::Point>& space,
                                     const core::Point& start, const core::Point& goal,
                                     core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
