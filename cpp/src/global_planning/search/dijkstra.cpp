#include "navigation/global_planning/search/dijkstra.hpp"

#include "navigation/global_planning/search/discrete_search.hpp"

namespace navigation::global_planning {

core::PlanResult<Cell> DijkstraPlanner::plan(DiscreteSpace<Cell>& space, const Cell& start,
                                             const Cell& goal, TraceRecorder* recorder) {
  // Dijkstra is best-first search with a zero heuristic (Dijkstra 1959).
  return best_first_search(space, start, goal, recorder, [](const Cell&) { return 0.0; });
}

}  // namespace navigation::global_planning
