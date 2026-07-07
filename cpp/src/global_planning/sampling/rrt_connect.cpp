#include "navigation/global_planning/sampling/rrt_connect.hpp"

#include <algorithm>
#include <chrono>
#include <vector>

#include "navigation/global_planning/sampling/sampling_common.hpp"

namespace navigation::global_planning {

namespace {

// EXTEND (Kuffner & LaValle 2000): one step_size step from the tree's nearest node
// toward target. Returns the new node index on a collision-free step (Advanced),
// or -1 when the step is blocked (Trapped).
int extend(Tree& tree, SamplingSpace<Point>& space, const Point& target, double step_size,
           TraceRecorder* recorder) {
  int ni = nearest(tree, space, target);
  // 값 복사 필수: 아래 tree.add()의 push_back 이 벡터를 재할당하면 참조가 무효화되어
  // trace 방출(edge_added)이 해제된 메모리를 읽는다.
  Point q_near = tree.nodes[ni];
  Point q_new = space.steer(q_near, target, step_size);
  if (!space.is_motion_valid(q_near, q_new)) return -1;
  double edge = space.distance(q_near, q_new);
  int ci = tree.add(q_new, ni, tree.cost[ni] + edge);
  if (recorder) recorder->edge_added(q_new, q_near, edge);
  return ci;
}

// CONNECT (Kuffner & LaValle 2000): greedily EXTEND the tree toward the fixed
// target until it Reaches (within goal_tolerance) or is Trapped. Terminates:
// steer clamps to target once within step_size, so each Advanced step is
// monotonic progress and Reached fires in at most ceil(dist/step_size)+1 steps.
int connect(Tree& tree, SamplingSpace<Point>& space, const Point& target, double step_size,
            double goal_tolerance, TraceRecorder* recorder) {
  while (true) {
    int ci = extend(tree, space, target, step_size, recorder);
    if (ci < 0) return -1;
    if (space.distance(tree.nodes[ci], target) <= goal_tolerance) return ci;
  }
}

}  // namespace

core::PlanResult<Point> RrtConnectPlanner::plan(SamplingSpace<Point>& space, const Point& start,
                                                const Point& goal, TraceRecorder* recorder) {
  const int max_iterations = params_.get_int("max_iterations");
  const double step_size = params_.get_float("step_size");
  const double goal_tolerance = params_.get_float("goal_tolerance");
  // Every draw is a uniform space.sample() (no goal bias / directional RNG), so
  // sampling reproducibility is owned by the map's seed, not a planner-local RNG.

  auto t0 = std::chrono::steady_clock::now();
  Tree tree_start;
  tree_start.add(start, -1, 0.0);
  Tree tree_goal;
  tree_goal.add(goal, -1, 0.0);
  // ta extends toward the sample; tb connects toward ta's new node. They swap
  // every iteration so both trees grow (Kuffner & LaValle 2000). Pointer identity
  // against &tree_start decides splice orientation regardless of the swap parity.
  Tree* ta = &tree_start;
  Tree* tb = &tree_goal;
  core::PlanResult<Point> result;
  int samples = 0;
  int iterations = 0;

  for (int it = 0; it < max_iterations; ++it) {
    ++iterations;
    Point q_rand = space.sample();
    if (recorder) recorder->sample_drawn(q_rand);
    ++samples;

    int new_idx = extend(*ta, space, q_rand, step_size, recorder);
    if (new_idx >= 0) {
      // 값 복사: 아래 tb 확장이 ta 를 건드리진 않지만, connect 로 넘기기 전에 안전하게 고정.
      Point q_new = ta->nodes[new_idx];
      int tb_idx = connect(*tb, space, q_new, step_size, goal_tolerance, recorder);
      if (tb_idx >= 0) {
        std::vector<Point> bridge = extract_path(*ta, new_idx);  // root(ta)...q_new
        std::vector<Point> b_path = extract_path(*tb, tb_idx);   // root(tb)...q_step
        for (auto rit = b_path.rbegin(); rit != b_path.rend(); ++rit) bridge.push_back(*rit);
        // bridge runs root(ta)->...->root(tb); reverse it when goal is the extending
        // tree so the returned path always begins at start.
        if (ta != &tree_start) std::reverse(bridge.begin(), bridge.end());
        result.success = true;
        result.path = std::move(bridge);
        result.cost = path_length(space, result.path);
        if (recorder) recorder->path_found(result.path);
        break;
      }
    }
    std::swap(ta, tb);
  }

  int tree_size = static_cast<int>(tree_start.nodes.size() + tree_goal.nodes.size());
  int expanded = tree_size - 2;  // exclude both roots
  result.stats = {expanded, samples, iterations, tree_size};
  double rt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  emit_finished_sampling(recorder, result.success, result.cost, expanded, samples, tree_size,
                         iterations, rt);
  return result;
}

}  // namespace navigation::global_planning
