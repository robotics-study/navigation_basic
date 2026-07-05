#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// Breadth-first search: FIFO frontier. Optimal in number of moves (unit-step),
// though the reported path cost uses true edge costs.
class BfsPlanner final : public core::DiscretePlanner {
 public:
  explicit BfsPlanner(core::ParamSet params) : core::DiscretePlanner(std::move(params)) {}
  std::string name() const override { return "bfs"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE};
  }
  core::PlanResult<core::Cell> plan(core::DiscreteSpace<core::Cell>& space, const core::Cell& start,
                                    const core::Cell& goal, core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
