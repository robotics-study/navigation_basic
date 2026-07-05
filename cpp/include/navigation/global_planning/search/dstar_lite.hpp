#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// D* Lite (Koenig & Likhachev 2002): incremental replanning for a robot that starts
// with no map (freespace assumption) and discovers obstacles with a local sensor as
// it moves. plan() simulates the whole move -> sense -> incremental-repair loop
// internally and returns the *executed trajectory* (NOT a from-start plan): a
// backward A*-like search from the goal maintains g/rhs values that are cheaply
// repaired when a sensor reading contradicts the belief, so each replan reuses the
// previous search instead of starting over. The k_m key offset keeps queue keys
// monotone as the robot (the heuristic's reference point) moves.
class DStarLitePlanner final : public core::DynamicGridPlanner {
 public:
  explicit DStarLitePlanner(core::ParamSet params)
      : core::DynamicGridPlanner(std::move(params)) {}
  std::string name() const override { return "dstar_lite"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DYNAMIC_GRID_SPACE};
  }
  // PlanResult::path is the executed trajectory (start -> ... -> goal); cost is its
  // realized length. stats.expanded_nodes is cumulative over every replan and
  // stats.iterations is the replan count.
  core::PlanResult<core::Cell> plan(core::DynamicGridSpace<core::Cell>& space,
                                    const core::Cell& start, const core::Cell& goal,
                                    core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
