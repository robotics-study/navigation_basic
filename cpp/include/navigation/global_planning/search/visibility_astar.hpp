#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// Visibility A*: plain A* whose successor relation is line-of-sight visibility
// instead of grid adjacency. From an expanded cell it relaxes every free cell it
// can see along an obstacle-free straight line, at Euclidean cost, settling the
// shortest path over the cell-centre visibility graph (V = reachable free cells,
// E = mutually LOS-visible pairs) with the admissible+consistent Euclidean
// heuristic. w > 1 trades that optimality for speed (Pohl 1970).
//
// This is a cell-centre approximation, NOT a true Euclidean any-angle optimum:
// turns are restricted to cell centres, so the genuinely shortest route (which
// may turn at obstacle corners no cell centre lands on) is not recovered. It is
// therefore not an optimal any-angle search. What it guarantees is a valid
// any-angle path — every leg a
// LOS-clear straight segment (list<Cell>, as in Theta*) — at a cost no worse than
// Theta* on the same instance, since any Theta* output is itself one path in this
// same cell-centre visibility graph.
class VisibilityAStarPlanner final : public core::LineOfSightPlanner {
 public:
  explicit VisibilityAStarPlanner(core::ParamSet params)
      : core::LineOfSightPlanner(std::move(params)) {}
  std::string name() const override { return "visibility_astar"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE, core::Capability::LINE_OF_SIGHT_SPACE};
  }
  core::PlanResult<core::Cell> plan(core::LineOfSightSpace<core::Cell>& space,
                                    const core::Cell& start, const core::Cell& goal,
                                    core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
