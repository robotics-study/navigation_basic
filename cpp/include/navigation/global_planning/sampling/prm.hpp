#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// PRM — Probabilistic Roadmap (Kavraki, Svestka, Latombe & Overmars 1996). Used
// here single-query: sample free states, connect each pair within a fixed
// connection_radius by a collision-free straight motion, then answer start->goal
// with Dijkstra over the roadmap.
class PrmPlanner final : public core::SamplingPlanner {
 public:
  explicit PrmPlanner(core::ParamSet params) : core::SamplingPlanner(std::move(params)) {}
  std::string name() const override { return "prm"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::SAMPLING_SPACE};
  }
  core::PlanResult<core::Point> plan(core::SamplingSpace<core::Point>& space,
                                     const core::Point& start, const core::Point& goal,
                                     core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
