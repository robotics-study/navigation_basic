#include "navigation/global_planning/fast_rrt.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <random>

#include "navigation/global_planning/sampling_common.hpp"

namespace navigation::global_planning {
namespace {

// True if some tree node lies within `radius` of q (Fast-Sampling reject test).
bool within_reached(const Tree& tree, const SamplingSpace<Point>& space, const Point& q,
                    double radius) {
  for (const Point& n : tree.nodes) {
    if (space.distance(n, q) <= radius) return true;
  }
  return false;
}

// Fast-Optimal shortcut pruning: drop any waypoint whose bypass segment is
// collision-free (triangle inequality shortens the path).
std::vector<Point> shortcut_prune(const SamplingSpace<Point>& space, std::vector<Point> path) {
  bool changed = true;
  while (changed && path.size() > 2) {
    changed = false;
    for (size_t i = 1; i + 1 < path.size(); ++i) {
      if (space.is_motion_valid(path[i - 1], path[i + 1])) {
        path.erase(path.begin() + static_cast<long>(i));
        changed = true;
        break;
      }
    }
  }
  return path;
}

}  // namespace

core::PlanResult<Point> FastRrtPlanner::plan(SamplingSpace<Point>& space, const Point& start,
                                             const Point& goal, TraceRecorder* recorder) {
  const int max_iterations = params_.get_int("max_iterations");
  const double step_size = params_.get_float("step_size");
  const double goal_bias = params_.get_float("goal_bias");
  const double goal_tolerance = params_.get_float("goal_tolerance");
  const double neighbor_radius = params_.get_float("neighbor_radius");
  const double reached_radius = params_.get_float("reached_radius");
  const int steering_attempts = params_.get_int("steering_attempts");
  std::mt19937 rng(static_cast<unsigned>(params_.get_int("seed")));
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> angle(0.0, 2.0 * std::numbers::pi);

  auto t0 = std::chrono::steady_clock::now();
  Tree tree;
  tree.add(start, -1, 0.0);
  std::vector<Point> best_path;
  double best_cost = std::numeric_limits<double>::infinity();
  int samples = 0;
  int iterations = 0;

  for (int it = 0; it < max_iterations; ++it) {
    ++iterations;

    // Fast-Sampling: only accept a sample in unreached space; the bounded retry
    // count reuses steering_attempts. Goal-biased samples are always accepted.
    bool goal_sample = unit(rng) < goal_bias;
    Point q_rand = goal_sample ? goal : space.sample();
    if (!goal_sample) {
      for (int a = 0; a < steering_attempts && within_reached(tree, space, q_rand, reached_radius);
           ++a) {
        q_rand = space.sample();
      }
    }
    if (recorder) recorder->sample_drawn(q_rand);
    ++samples;

    int ni = nearest(tree, space, q_rand);
    Point q_near = tree.nodes[ni];
    Point q_new = space.steer(q_near, q_rand, step_size);
    bool extended = space.is_motion_valid(q_near, q_new);

    // Random Steering: if the straight extension is blocked, try random
    // directions and take the first collision-free step (helps narrow passages).
    if (!extended) {
      for (int a = 0; a < steering_attempts; ++a) {
        double th = angle(rng);
        Point dir{q_near.x + step_size * std::cos(th), q_near.y + step_size * std::sin(th)};
        Point cand = space.steer(q_near, dir, step_size);
        if (space.is_motion_valid(q_near, cand)) {
          q_new = cand;
          extended = true;
          break;
        }
      }
    }
    if (!extended) continue;

    std::vector<int> nbrs = near(tree, space, q_new, neighbor_radius);
    double new_cost = 0.0;
    int parent = choose_parent(tree, space, q_new, nbrs, ni, new_cost, recorder);
    int ci = tree.add(q_new, parent, new_cost);
    if (recorder) recorder->edge_added(q_new, tree.nodes[parent], space.distance(tree.nodes[parent], q_new));
    rewire_neighbors(tree, space, ci, nbrs, recorder);

    if (space.distance(q_new, goal) <= goal_tolerance && space.is_motion_valid(q_new, goal)) {
      std::vector<Point> path = extract_path(tree, ci);
      path.push_back(goal);
      path = shortcut_prune(space, std::move(path));
      double c = path_length(space, path);
      if (c < best_cost) {
        best_cost = c;
        best_path = path;
        if (recorder) recorder->path_found(best_path);
      }
    }
  }

  core::PlanResult<Point> result;
  if (!best_path.empty()) {
    result.success = true;
    result.path = best_path;
    result.cost = best_cost;
  }

  int tree_size = static_cast<int>(tree.nodes.size());
  result.stats = {tree_size - 1, samples, iterations, tree_size};
  double rt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  emit_finished_sampling(recorder, result.success, result.cost, tree_size - 1, samples, tree_size,
                         iterations, rt);
  return result;
}

}  // namespace navigation::global_planning
