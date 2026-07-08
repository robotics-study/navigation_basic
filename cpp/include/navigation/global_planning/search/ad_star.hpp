#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// AD* (Likhachev, Ferguson, Gordon, Stentz & Thrun 2005): Anytime Dynamic A*. Fuses
// ARA*'s inflated heuristic (a fast, bounded-suboptimal first solution repaired as ε
// shrinks) with D* Lite's backward, incrementally repaired search (goal -> start,
// g/rhs + the k_m key offset so a moving robot reuses the previous search). A robot
// starts with no map (freespace assumption) and a local sensor; plan() simulates the
// whole improve -> move -> sense -> repair loop and returns the *executed trajectory*
// (NOT a from-start plan).
//
// Layered on the D* Lite skeleton: (1) an *over-consistent* vertex's priority inflates
// its heuristic by ε, while an *under-consistent* vertex keeps an un-inflated key (the
// paper's key(s)); (2) a vertex made inconsistent after it was already expanded goes to
// INCONS instead of back into OPEN, and INCONS union OPEN is reopened with recomputed
// keys whenever ε is lowered or an edge cost changes. The robot steps only once ε has
// reached eps_final (belief-optimal plan), so the executed trajectory matches D* Lite.
class AdStarPlanner final : public core::DynamicGridPlanner {
 public:
  explicit AdStarPlanner(core::ParamSet params)
      : core::DynamicGridPlanner(std::move(params)) {}
  std::string name() const override { return "ad_star"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DYNAMIC_GRID_SPACE};
  }
  // PlanResult::path is the executed trajectory (start -> ... -> goal); cost is its
  // realized length. stats.expanded_nodes is cumulative over every ComputeOrImprovePath
  // pass and stats.iterations is the replan (sensed-change) count.
  core::PlanResult<core::Cell> plan(core::DynamicGridSpace<core::Cell>& space,
                                    const core::Cell& start, const core::Cell& goal,
                                    core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
