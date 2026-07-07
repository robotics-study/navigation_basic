#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// Anya (Harabor, Grastien, Öz & Aksakalli 2016, "Optimal Any-Angle Pathfinding
// In Practice", JAIR 56): the provably Euclidean-shortest any-angle planner.
// Anya searches over (root, interval) nodes — an interval is a contiguous run of
// same-row points visible from the root — and expands by projecting the interval
// through the grid to enumerate observable successors. Unlike Theta* (any-angle
// but not optimal), it returns the true shortest any-angle path.
//
// Adapted to this repository's cell-centre vertex model (turning points are cell
// centres, as in Theta*, so the path is a list<Cell> of LOS-clear straight
// legs): roots are cell centres settled in best-first order (A* with the
// admissible+consistent Euclidean heuristic), and a root's successors are the
// LOS-visible free cells grouped, per grid row, into maximal column intervals.
// Relaxing the whole visible set settles the cell-centre visibility-graph
// optimum, which is therefore always <= the Theta* cost. w > 1 trades optimality
// for speed (Pohl 1970).
class AnyaPlanner final : public core::LineOfSightPlanner {
 public:
  explicit AnyaPlanner(core::ParamSet params)
      : core::LineOfSightPlanner(std::move(params)) {}
  std::string name() const override { return "anya"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE, core::Capability::LINE_OF_SIGHT_SPACE};
  }
  core::PlanResult<core::Cell> plan(core::LineOfSightSpace<core::Cell>& space,
                                    const core::Cell& start, const core::Cell& goal,
                                    core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
