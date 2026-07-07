#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// BIT* — Batch Informed Trees (Gammell, Srinivasa & Barfoot 2015). Processes
// samples in batches, expanding an implicit random geometric graph in order of
// estimated solution cost with a best-first edge queue. Collision checks are lazy
// (deferred to edge dequeue), and once a solution exists new batches are drawn
// from the informed ellipse (Gammell et al. 2014). Anytime: keeps tightening the
// path across batches.
class BitStarPlanner final : public core::SamplingPlanner {
 public:
  explicit BitStarPlanner(core::ParamSet params) : core::SamplingPlanner(std::move(params)) {}
  std::string name() const override { return "bit_star"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::SAMPLING_SPACE};
  }
  core::PlanResult<core::Point> plan(core::SamplingSpace<core::Point>& space,
                                     const core::Point& start, const core::Point& goal,
                                     core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
