#include "navigation/global_planning/sampling/informed_rrt_star.hpp"

#include <chrono>
#include <limits>
#include <random>

#include "navigation/global_planning/sampling/sampling_common.hpp"

namespace navigation::global_planning {

core::PlanResult<Point> InformedRrtStarPlanner::plan(SamplingSpace<Point>& space,
                                                     const Point& start, const Point& goal,
                                                     TraceRecorder* recorder) {
  const int max_iterations = params_.get_int("max_iterations");
  const double step_size = params_.get_float("step_size");
  const double goal_bias = params_.get_float("goal_bias");
  const double goal_tolerance = params_.get_float("goal_tolerance");
  const double neighbor_radius = params_.get_float("neighbor_radius");
  std::mt19937 rng(static_cast<unsigned>(params_.get_int("seed")));
  std::uniform_real_distribution<double> unit(0.0, 1.0);

  auto t0 = std::chrono::steady_clock::now();
  Tree tree;
  tree.add(start, -1, 0.0);
  int best_goal_parent = -1;
  double best_goal_cost = std::numeric_limits<double>::infinity();
  int samples = 0;
  int iterations = 0;

  for (int it = 0; it < max_iterations; ++it) {
    ++iterations;
    // Before a solution exists, sample uniformly (goal-biased) like RRT*; after,
    // draw from the informed ellipse so samples focus on the region that can still
    // beat the incumbent (Gammell, Srinivasa & Barfoot 2014).
    Point q_rand;
    if (best_goal_parent >= 0) {
      q_rand = informed_sample(space, start, goal, best_goal_cost, rng);
    } else {
      q_rand = (unit(rng) < goal_bias) ? goal : space.sample();
    }
    if (recorder) recorder->sample_drawn(q_rand);
    ++samples;

    int ni = nearest(tree, space, q_rand);
    Point q_new = space.steer(tree.nodes[ni], q_rand, step_size);
    if (!space.is_motion_valid(tree.nodes[ni], q_new)) continue;

    std::vector<int> nbrs = near(tree, space, q_new, neighbor_radius);
    double new_cost = 0.0;
    int parent = choose_parent(tree, space, q_new, nbrs, ni, new_cost, recorder);
    int ci = tree.add(q_new, parent, new_cost);
    if (recorder) recorder->edge_added(q_new, tree.nodes[parent], space.distance(tree.nodes[parent], q_new));
    rewire_neighbors(tree, space, ci, nbrs, recorder);

    // Track the best goal connection; anytime improvement emits path_found.
    if (space.distance(q_new, goal) <= goal_tolerance && space.is_motion_valid(q_new, goal)) {
      double gc = tree.cost[ci] + space.distance(q_new, goal);
      if (gc < best_goal_cost) {
        best_goal_cost = gc;
        best_goal_parent = ci;
        std::vector<Point> path = extract_path(tree, ci);
        path.push_back(goal);
        if (recorder) recorder->path_found(path);
      }
    }
  }

  core::PlanResult<Point> result;
  if (best_goal_parent >= 0) {
    result.success = true;
    result.path = extract_path(tree, best_goal_parent);
    result.path.push_back(goal);
    // Recompute from the final polyline: rewiring after the goal connection can
    // change an ancestor, so the snapshot best_goal_cost may no longer match the
    // real path length.
    result.cost = path_length(space, result.path);
  }

  int tree_size = static_cast<int>(tree.nodes.size());
  result.stats = {tree_size - 1, samples, iterations, tree_size};
  double rt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  emit_finished_sampling(recorder, result.success, result.cost, tree_size - 1, samples, tree_size,
                         iterations, rt);
  return result;
}

}  // namespace navigation::global_planning
