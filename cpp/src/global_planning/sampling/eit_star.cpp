#include "navigation/global_planning/sampling/eit_star.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include "navigation/global_planning/sampling/sampling_common.hpp"

namespace navigation::global_planning {

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();

using Edge = std::pair<int, int>;

// Undirected edge key: order-independent so (u,v) and (v,u) share one entry in
// the persistent invalid-edge set.
Edge edge_key(int a, int b) { return a < b ? Edge{a, b} : Edge{b, a}; }

// Single-source shortest cost over a fixed undirected graph. EIT* invokes it
// twice from the goal (distance weights, then effort weights) to build the two
// reverse heuristics independently (Strub & Gammell 2022).
std::vector<double> dijkstra_from(int source, const std::vector<std::vector<int>>& adjacency,
                                  const std::vector<std::vector<double>>& weight) {
  int n = static_cast<int>(adjacency.size());
  std::vector<double> dist(static_cast<size_t>(n), kInf);
  dist[static_cast<size_t>(source)] = 0.0;
  using Entry = std::pair<double, int>;  // (cost, idx)
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> heap;
  heap.emplace(0.0, source);
  while (!heap.empty()) {
    auto [d, u] = heap.top();
    heap.pop();
    if (d > dist[static_cast<size_t>(u)]) continue;
    const auto& nbrs = adjacency[static_cast<size_t>(u)];
    const auto& wts = weight[static_cast<size_t>(u)];
    for (size_t k = 0; k < nbrs.size(); ++k) {
      int v = nbrs[k];
      double nd = d + wts[k];
      if (nd < dist[static_cast<size_t>(v)]) {
        dist[static_cast<size_t>(v)] = nd;
        heap.emplace(nd, v);
      }
    }
  }
  return dist;
}
}  // namespace

core::PlanResult<Point> EitStarPlanner::plan(SamplingSpace<Point>& space, const Point& start,
                                             const Point& goal, TraceRecorder* recorder) {
  const int batch_size = params_.get_int("batch_size");
  const int max_batches = params_.get_int("max_batches");
  const double gamma = params_.get_float("gamma");
  const double step_size = params_.get_float("step_size");
  std::mt19937 rng(static_cast<unsigned>(params_.get_int("seed")));

  auto t0 = std::chrono::steady_clock::now();
  // Sample array grows across batches: 0 = start, 1 = goal (permanent).
  std::vector<Point> points{start, goal};
  const int start_idx = 0, goal_idx = 1;
  std::set<Edge> invalid_edges;
  double c_best = kInf;
  int expanded = 0;
  // Forward-search cost/parent from the final batch — the incumbent is read off
  // these once batches are exhausted (batch-recomputed reverse/forward search, per
  // the documented simplification of the AIT*/EIT* lineage).
  std::vector<double> g{kInf, kInf};
  std::vector<int> parent{-1, -1};

  // Number of step_size-sized sub-segments a discretized validator would check on
  // edge (a,b): a capability-free proxy for collision-check cost (Strub & Gammell 2022).
  auto effort = [&](int a, int b) -> double {
    double d = space.distance(points[static_cast<size_t>(a)], points[static_cast<size_t>(b)]);
    return static_cast<double>(std::max(1L, std::lround(d / step_size)));
  };

  auto reconstruct = [&](const std::vector<int>& par) {
    std::vector<Point> path;
    for (int node = goal_idx; node != -1; node = par[static_cast<size_t>(node)]) {
      path.push_back(points[static_cast<size_t>(node)]);
    }
    std::reverse(path.begin(), path.end());
    return path;
  };

  for (int batch = 0; batch < max_batches; ++batch) {
    // --- draw a new batch (informed once a solution exists) --------------------
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

    // --- filtered adjacency + parallel distance/effort weights -----------------
    std::vector<std::vector<int>> adjacency(static_cast<size_t>(n));
    std::vector<std::vector<double>> dist_w(static_cast<size_t>(n));
    std::vector<std::vector<double>> eff_w(static_cast<size_t>(n));
    for (int u = 0; u < n; ++u) {
      for (int v : nbr[static_cast<size_t>(u)]) {
        if (invalid_edges.count(edge_key(u, v))) continue;
        adjacency[static_cast<size_t>(u)].push_back(v);
        dist_w[static_cast<size_t>(u)].push_back(
            space.distance(points[static_cast<size_t>(u)], points[static_cast<size_t>(v)]));
        eff_w[static_cast<size_t>(u)].push_back(effort(u, v));
      }
    }

    // --- reverse search: two independent Dijkstra passes from the goal ---------
    std::vector<double> h_hat = dijkstra_from(goal_idx, adjacency, dist_w);
    std::vector<double> e_hat = dijkstra_from(goal_idx, adjacency, eff_w);

    // --- forward search: lazy-deletion best-first over (cost, effort) ----------
    g.assign(static_cast<size_t>(n), kInf);
    std::vector<double> effort_g(static_cast<size_t>(n), kInf);
    parent.assign(static_cast<size_t>(n), -1);
    std::vector<char> closed(static_cast<size_t>(n), 0);
    g[static_cast<size_t>(start_idx)] = 0.0;
    effort_g[static_cast<size_t>(start_idx)] = 0.0;
    // Key (g + h_hat, effort_g + e_hat, idx): std::tuple compares lexicographically,
    // so cost is primary and effort the tie-break exactly as the paper intends; idx
    // makes the order deterministic.
    using FEntry = std::tuple<double, double, int>;
    std::priority_queue<FEntry, std::vector<FEntry>, std::greater<FEntry>> heap;
    heap.emplace(h_hat[static_cast<size_t>(start_idx)], e_hat[static_cast<size_t>(start_idx)],
                 start_idx);

    while (!heap.empty()) {
      int v = std::get<2>(heap.top());
      heap.pop();
      if (closed[static_cast<size_t>(v)]) continue;
      closed[static_cast<size_t>(v)] = 1;
      ++expanded;
      for (int x : adjacency[static_cast<size_t>(v)]) {
        double d = space.distance(points[static_cast<size_t>(v)], points[static_cast<size_t>(x)]);
        if (!space.is_motion_valid(points[static_cast<size_t>(v)],
                                   points[static_cast<size_t>(x)])) {
          invalid_edges.insert(edge_key(v, x));
          continue;
        }
        double new_g = g[static_cast<size_t>(v)] + d;
        double new_effort = effort_g[static_cast<size_t>(v)] + effort(v, x);
        // Lexicographic acceptance: cost primary, cumulative effort the tie-break,
        // consistent with the priority-queue ordering.
        if (std::make_pair(new_g, new_effort) <
            std::make_pair(g[static_cast<size_t>(x)], effort_g[static_cast<size_t>(x)])) {
          bool first = parent[static_cast<size_t>(x)] == -1;
          g[static_cast<size_t>(x)] = new_g;
          effort_g[static_cast<size_t>(x)] = new_effort;
          parent[static_cast<size_t>(x)] = v;
          closed[static_cast<size_t>(x)] = 0;  // improved: allow re-expansion this batch
          heap.emplace(new_g + h_hat[static_cast<size_t>(x)],
                       new_effort + e_hat[static_cast<size_t>(x)], x);
          if (recorder) {
            // Emit candidate_evaluated only for feasible, improving edges (not
            // every relaxed neighbour) so the trace stays renderable by replay.py
            // — matches BIT*'s emit-on-accept scale rather than exploding on the
            // batch-recomputed graph. Cost is the primary reported metric.
            recorder->candidate_evaluated(points[static_cast<size_t>(x)], new_g);
            if (first)
              recorder->edge_added(points[static_cast<size_t>(x)], points[static_cast<size_t>(v)],
                                   d);
            else
              recorder->rewire(points[static_cast<size_t>(x)], points[static_cast<size_t>(v)]);
          }
          if (x == goal_idx && new_g < c_best) {
            c_best = new_g;
            if (recorder) recorder->path_found(reconstruct(parent));
          }
        }
      }
    }
  }

  // --- extract incumbent from the final batch's forward tree -----------------
  bool success = g[static_cast<size_t>(goal_idx)] < kInf;
  std::vector<Point> path;
  if (success) path = reconstruct(parent);
  double cost = success ? path_length(space, path) : 0.0;
  if (success && recorder) recorder->path_found(path);

  double rt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  int n = static_cast<int>(points.size());
  emit_finished_batch(recorder, success, cost, expanded, n, n, rt);
  core::PlanResult<Point> result;
  result.success = success;
  result.path = std::move(path);
  result.cost = cost;
  result.stats = {expanded, n, expanded, n};
  return result;
}

}  // namespace navigation::global_planning
