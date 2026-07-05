#include "navigation/global_planning/rrt.hpp"

#include <chrono>
#include <random>

#include "navigation/global_planning/sampling_common.hpp"

namespace navigation::global_planning {

core::PlanResult<Point> RrtPlanner::plan(SamplingSpace<Point>& space, const Point& start,
                                         const Point& goal, TraceRecorder* recorder) {
  const int max_iterations = params_.get_int("max_iterations");
  const double step_size = params_.get_float("step_size");
  const double goal_bias = params_.get_float("goal_bias");
  const double goal_tolerance = params_.get_float("goal_tolerance");
  std::mt19937 rng(static_cast<unsigned>(params_.get_int("seed")));
  std::uniform_real_distribution<double> unit(0.0, 1.0);

  auto t0 = std::chrono::steady_clock::now();
  Tree tree;
  tree.add(start, -1, 0.0);
  int goal_idx = -1;
  int samples = 0;
  int iterations = 0;

  for (int it = 0; it < max_iterations; ++it) {
    ++iterations;
    Point q_rand = (unit(rng) < goal_bias) ? goal : space.sample();
    if (recorder) recorder->sample_drawn(q_rand);
    ++samples;

    int ni = nearest(tree, space, q_rand);
    // 값 복사 필수: 아래 tree.add()의 push_back 이 벡터를 재할당하면 참조가 무효화되어
    // trace 방출(edge_added)이 해제된 메모리를 읽는다.
    Point q_near = tree.nodes[ni];
    Point q_new = space.steer(q_near, q_rand, step_size);
    if (!space.is_motion_valid(q_near, q_new)) continue;

    double edge = space.distance(q_near, q_new);
    int ci = tree.add(q_new, ni, tree.cost[ni] + edge);
    if (recorder) recorder->edge_added(q_new, q_near, edge);

    // First feasible solution: the goal must be both within tolerance AND actually
    // connectable by a collision-free motion (LaValle 1998). Being inside the
    // tolerance ball is not enough if a wall lies between q_new and the goal.
    if (space.distance(q_new, goal) <= goal_tolerance && space.is_motion_valid(q_new, goal)) {
      double goal_edge = space.distance(q_new, goal);
      goal_idx = tree.add(goal, ci, tree.cost[ci] + goal_edge);
      if (recorder) recorder->edge_added(goal, q_new, goal_edge);
      break;
    }
  }

  core::PlanResult<Point> result;
  if (goal_idx >= 0) {
    result.success = true;
    result.path = extract_path(tree, goal_idx);
    result.cost = path_length(space, result.path);
    if (recorder) recorder->path_found(result.path);
  }

  int tree_size = static_cast<int>(tree.nodes.size());
  result.stats = {tree_size - 1, samples, iterations, tree_size};
  double rt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  emit_finished_sampling(recorder, result.success, result.cost, tree_size - 1, samples, tree_size,
                         iterations, rt);
  return result;
}

}  // namespace navigation::global_planning
