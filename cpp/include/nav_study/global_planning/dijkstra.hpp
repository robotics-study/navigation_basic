#pragma once

#include <set>
#include <string>

#include "nav_study/core/planner.hpp"

namespace nav_study::global_planning {

// Dijkstra's algorithm (Dijkstra 1959): uniform-cost search, optimal, no
// heuristic.
class DijkstraPlanner final : public core::DiscretePlanner {
 public:
  explicit DijkstraPlanner(core::ParamSet params) : core::DiscretePlanner(std::move(params)) {}
  std::string name() const override { return "dijkstra"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE};
  }
  core::PlanResult<core::Cell> plan(core::DiscreteSpace<core::Cell>& space, const core::Cell& start,
                                    const core::Cell& goal, core::TraceRecorder* recorder) override;
};

}  // namespace nav_study::global_planning
