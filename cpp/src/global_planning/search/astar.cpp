#include "navigation/global_planning/search/astar.hpp"

#include "navigation/global_planning/search/discrete_search.hpp"

namespace navigation::global_planning {

core::PlanResult<Cell> AstarPlanner::plan(DiscreteSpace<Cell>& space, const Cell& start,
                                          const Cell& goal, TraceRecorder* recorder) {
  // Weighted A*: f = g + w*h. w == 1 stays admissible/optimal (Pohl 1970).
  double w = params_.get_float("heuristic_weight");
  return best_first_search(space, start, goal, recorder,
                           [&](const Cell& c) { return w * space.heuristic(c, goal); });
}

}  // namespace navigation::global_planning
