#include "navigation/global_planning/sampling/prm_star.hpp"

#include <chrono>

#include "navigation/global_planning/sampling/roadmap_common.hpp"
#include "navigation/global_planning/sampling/sampling_common.hpp"

namespace navigation::global_planning {

core::PlanResult<Point> PrmStarPlanner::plan(SamplingSpace<Point>& space, const Point& start,
                                             const Point& goal, TraceRecorder* recorder) {
  const int num_samples = params_.get_int("num_samples");
  const double gamma = params_.get_float("gamma");

  auto t0 = std::chrono::steady_clock::now();
  Roadmap roadmap;
  int start_idx = roadmap.add_node(start);
  int goal_idx = roadmap.add_node(goal);
  sample_free(space, roadmap, num_samples, recorder);
  // PRM* radius: computed once from the final node count (Karaman & Frazzoli 2011).
  double radius = rgg_radius(gamma, static_cast<int>(roadmap.nodes.size()));
  for (int idx = 1; idx < static_cast<int>(roadmap.nodes.size()); ++idx) {
    connect(space, roadmap, idx, radius, recorder);
  }

  double cost = 0.0;
  int expanded = 0;
  std::vector<Point> path = dijkstra(roadmap, start_idx, goal_idx, recorder, cost, expanded);
  double rt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  bool success = !path.empty();
  if (success && recorder) recorder->path_found(path);

  int nnodes = static_cast<int>(roadmap.nodes.size());
  emit_finished_batch(recorder, success, cost, expanded, nnodes, nnodes, rt);
  core::PlanResult<Point> result;
  result.success = success;
  result.path = std::move(path);
  result.cost = success ? cost : 0.0;
  result.stats = {expanded, nnodes, 0, nnodes};
  return result;
}

}  // namespace navigation::global_planning
