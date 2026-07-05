#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// A* (Hart, Nilsson & Raphael 1968): best-first on f = g + w*h. w == 1 is
// optimal with an admissible heuristic; w > 1 is weighted A* (Pohl 1970).
class AstarPlanner final : public core::DiscretePlanner {
 public:
  explicit AstarPlanner(core::ParamSet params) : core::DiscretePlanner(std::move(params)) {}
  std::string name() const override { return "astar"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE};
  }
  core::PlanResult<core::Cell> plan(core::DiscreteSpace<core::Cell>& space, const core::Cell& start,
                                    const core::Cell& goal, core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
