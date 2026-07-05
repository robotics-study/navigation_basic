#include "navigation/global_planning/sampling/fmt_star.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <queue>
#include <vector>

#include "navigation/global_planning/sampling/sampling_common.hpp"

namespace navigation::global_planning {

core::PlanResult<Point> FmtStarPlanner::plan(SamplingSpace<Point>& space, const Point& start,
                                             const Point& goal, TraceRecorder* recorder) {
  const int num_samples = params_.get_int("num_samples");
  const double gamma = params_.get_float("gamma");
  const double inf = std::numeric_limits<double>::infinity();

  auto t0 = std::chrono::steady_clock::now();
  // Sample set: index 0 = start, 1 = goal, then free samples.
  std::vector<Point> points{start, goal};
  for (int attempt = 0; attempt < num_samples * 20; ++attempt) {
    if (static_cast<int>(points.size()) - 2 >= num_samples) break;
    Point q = space.sample();
    if (!space.is_state_valid(q)) continue;
    points.push_back(q);
    if (recorder) recorder->sample_drawn(q);
  }
  int n = static_cast<int>(points.size());
  double radius = rgg_radius(gamma, n);
  std::vector<std::vector<int>> neighbors = radius_neighbors(space, points, radius);

  std::vector<double> cost(static_cast<size_t>(n), inf);
  std::vector<int> parent(static_cast<size_t>(n), -1);
  cost[0] = 0.0;
  std::vector<char> in_open(static_cast<size_t>(n), 0);
  in_open[0] = 1;
  std::vector<char> unvisited(static_cast<size_t>(n), 1);
  unvisited[0] = 0;
  // Frontier heap holds (cost, idx); FMT* never lowers an open node's cost, so
  // entries stay valid and lazy membership via in_open is enough.
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> heap;
  int z = 0;
  const int goal_idx = 1;
  int expanded = 0;
  bool success = false;

  while (true) {
    ++expanded;
    for (int x : neighbors[static_cast<size_t>(z)]) {
      if (!unvisited[static_cast<size_t>(x)]) continue;
      int best_y = -1;
      double best_c = inf;
      for (int y : neighbors[static_cast<size_t>(x)]) {
        if (!in_open[static_cast<size_t>(y)]) continue;
        double c = cost[static_cast<size_t>(y)] + space.distance(points[static_cast<size_t>(y)],
                                                                 points[static_cast<size_t>(x)]);
        if (c < best_c) {
          best_c = c;
          best_y = y;
        }
      }
      // Lazy collision check: only the locally optimal edge is tested; if it
      // collides, x stays unvisited and may connect from a later z.
      if (best_y >= 0 && space.is_motion_valid(points[static_cast<size_t>(best_y)],
                                               points[static_cast<size_t>(x)])) {
        parent[static_cast<size_t>(x)] = best_y;
        cost[static_cast<size_t>(x)] = best_c;
        in_open[static_cast<size_t>(x)] = 1;
        unvisited[static_cast<size_t>(x)] = 0;
        heap.emplace(best_c, x);
        if (recorder) {
          recorder->edge_added(points[static_cast<size_t>(x)], points[static_cast<size_t>(best_y)],
                               space.distance(points[static_cast<size_t>(best_y)],
                                              points[static_cast<size_t>(x)]));
        }
      }
    }
    in_open[static_cast<size_t>(z)] = 0;  // z is now closed
    z = -1;
    while (!heap.empty()) {
      int i = heap.top().second;
      heap.pop();
      if (in_open[static_cast<size_t>(i)]) {
        z = i;
        break;
      }
    }
    if (z < 0) break;  // frontier exhausted: goal unreachable on this sample set
    if (recorder) recorder->node_expanded(points[static_cast<size_t>(z)], cost[static_cast<size_t>(z)]);
    if (z == goal_idx) {
      success = true;
      break;
    }
  }

  std::vector<Point> path;
  double total = 0.0;
  if (success) {
    for (int node = goal_idx; node != -1; node = parent[static_cast<size_t>(node)]) {
      path.push_back(points[static_cast<size_t>(node)]);
    }
    std::reverse(path.begin(), path.end());
    total = cost[static_cast<size_t>(goal_idx)];
    if (recorder) recorder->path_found(path);
  }

  double rt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  emit_finished_batch(recorder, success, total, expanded, n, n, rt);
  core::PlanResult<Point> result;
  result.success = success;
  result.path = std::move(path);
  result.cost = success ? total : 0.0;
  result.stats = {expanded, n, expanded, n};
  return result;
}

}  // namespace navigation::global_planning
