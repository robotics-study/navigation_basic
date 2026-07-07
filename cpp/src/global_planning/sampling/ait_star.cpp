#include "navigation/global_planning/sampling/ait_star.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <limits>
#include <queue>
#include <set>
#include <utility>
#include <vector>

#include "navigation/global_planning/sampling/sampling_common.hpp"

namespace navigation::global_planning {

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();

// Edges are keyed order-independently: an edge is invalid regardless of the
// direction it was traversed when the collision was detected.
using Edge = std::pair<int, int>;
Edge make_edge(int i, int j) { return i < j ? Edge{i, j} : Edge{j, i}; }

// Adaptive cost-to-go heuristic: Dijkstra from the goal over the RGG minus the
// known-invalid edges. Optimistic — it does not collision-check (that is the
// forward search's job); the forward search's findings shrink this graph next
// batch (Strub & Gammell 2020).
std::vector<double> reverse_search(SamplingSpace<Point>& space, const std::vector<Point>& points,
                                   const std::vector<std::vector<int>>& nbr,
                                   const std::set<Edge>& invalid_edges, int goal_idx) {
  int n = static_cast<int>(points.size());
  std::vector<double> h(static_cast<size_t>(n), kInf);
  std::vector<char> settled(static_cast<size_t>(n), 0);
  h[static_cast<size_t>(goal_idx)] = 0.0;
  using Entry = std::pair<double, int>;  // (dist, vertex)
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> heap;
  heap.emplace(0.0, goal_idx);
  while (!heap.empty()) {
    auto [d_u, u] = heap.top();
    heap.pop();
    if (settled[static_cast<size_t>(u)]) continue;
    settled[static_cast<size_t>(u)] = 1;
    for (int w : nbr[static_cast<size_t>(u)]) {
      if (invalid_edges.count(make_edge(u, w))) continue;
      double nd = d_u + space.distance(points[static_cast<size_t>(u)], points[static_cast<size_t>(w)]);
      if (nd < h[static_cast<size_t>(w)]) {
        h[static_cast<size_t>(w)] = nd;
        heap.emplace(nd, w);
      }
    }
  }
  return h;
}

std::vector<Point> reconstruct(const std::vector<Point>& points, const std::vector<int>& parent,
                               int goal_idx) {
  std::vector<Point> path;
  for (int node = goal_idx; node != -1; node = parent[static_cast<size_t>(node)]) {
    path.push_back(points[static_cast<size_t>(node)]);
  }
  std::reverse(path.begin(), path.end());
  return path;
}
}  // namespace

core::PlanResult<Point> AitStarPlanner::plan(SamplingSpace<Point>& space, const Point& start,
                                             const Point& goal, TraceRecorder* recorder) {
  const int batch_size = params_.get_int("batch_size");
  const int max_batches = params_.get_int("max_batches");
  const double gamma = params_.get_float("gamma");
  std::mt19937 rng(static_cast<unsigned>(params_.get_int("seed")));

  auto t0 = std::chrono::steady_clock::now();
  const int goal_idx = 1;
  std::vector<Point> points{start, goal};  // 0 = start (root), 1 = goal
  std::set<Edge> invalid_edges;            // grows across batches: adaptive feedback
  double c_best = kInf;
  int expanded = 0;
  // Last batch's forward-search result; the final incumbent is read from it.
  std::vector<double> g{0.0, kInf};
  std::vector<int> parent{-1, -1};

  for (int batch = 0; batch < max_batches; ++batch) {
    // --- 1. grow the RGG (informed once a solution exists) ---------------------
    int drawn = 0;
    for (int attempt = 0; attempt < batch_size * 40 && drawn < batch_size; ++attempt) {
      Point q = informed_sample(space, start, goal, c_best, rng);
      if (!space.is_state_valid(q)) continue;
      points.push_back(q);
      ++drawn;
      if (recorder) recorder->sample_drawn(q);
    }

    int n = static_cast<int>(points.size());
    double radius = rgg_radius(gamma, n);
    std::vector<std::vector<int>> nbr = radius_neighbors(space, points, radius);

    // --- 2+3. reverse search over the filtered graph = adaptive heuristic ------
    std::vector<double> h_hat = reverse_search(space, points, nbr, invalid_edges, goal_idx);

    // --- 4. forward A* keyed on g + h_hat, lazily validating each edge ---------
    g.assign(static_cast<size_t>(n), kInf);
    g[0] = 0.0;
    parent.assign(static_cast<size_t>(n), -1);
    std::vector<char> closed(static_cast<size_t>(n), 0);
    using Entry = std::pair<double, int>;  // (key, vertex)
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open_heap;
    open_heap.emplace(h_hat[0], 0);
    while (!open_heap.empty()) {
      auto [key, v] = open_heap.top();
      open_heap.pop();
      // Lazy deletion: skip entries superseded by a cheaper g[v] (same trick BIT*
      // uses for its vertex queue).
      if (closed[static_cast<size_t>(v)] ||
          key > g[static_cast<size_t>(v)] + h_hat[static_cast<size_t>(v)] + 1e-9)
        continue;
      closed[static_cast<size_t>(v)] = 1;
      ++expanded;
      for (int x : nbr[static_cast<size_t>(v)]) {
        if (invalid_edges.count(make_edge(v, x))) continue;
        double d = space.distance(points[static_cast<size_t>(v)], points[static_cast<size_t>(x)]);
        if (!space.is_motion_valid(points[static_cast<size_t>(v)], points[static_cast<size_t>(x)])) {
          // Discovered invalid: exclude it from every future batch's reverse
          // search — this is AIT*'s adaptive feedback loop.
          invalid_edges.insert(make_edge(v, x));
          continue;
        }
        double new_g = g[static_cast<size_t>(v)] + d;
        if (new_g < g[static_cast<size_t>(x)]) {
          bool first = g[static_cast<size_t>(x)] == kInf;
          g[static_cast<size_t>(x)] = new_g;
          parent[static_cast<size_t>(x)] = v;
          if (recorder) {
            // Emit candidate_evaluated only for feasible, improving edges (not
            // every relaxed neighbour) so the trace stays renderable by replay.py
            // — matches BIT*'s emit-on-accept scale rather than exploding on the
            // batch-recomputed graph.
            recorder->candidate_evaluated(points[static_cast<size_t>(x)], new_g);
            if (first)
              recorder->edge_added(points[static_cast<size_t>(x)], points[static_cast<size_t>(v)], d);
            else
              recorder->rewire(points[static_cast<size_t>(x)], points[static_cast<size_t>(v)]);
          }
          open_heap.emplace(new_g + h_hat[static_cast<size_t>(x)], x);
          if (x == goal_idx && new_g < c_best) {
            c_best = new_g;
            if (recorder) recorder->path_found(reconstruct(points, parent, goal_idx));
          }
        }
      }
    }
  }

  // --- extract final incumbent -----------------------------------------------
  int n = static_cast<int>(points.size());
  bool success = g[static_cast<size_t>(goal_idx)] < kInf;
  std::vector<Point> path = success ? reconstruct(points, parent, goal_idx) : std::vector<Point>{};
  double cost = success ? path_length(space, path) : 0.0;

  double rt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  emit_finished_batch(recorder, success, cost, expanded, n, n, rt);
  core::PlanResult<Point> result;
  result.success = success;
  result.path = std::move(path);
  result.cost = cost;
  result.stats = {expanded, n, expanded, n};
  return result;
}

}  // namespace navigation::global_planning
