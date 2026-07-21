#include "navigation/global_planning/sampling/fcit_star.hpp"

#include <algorithm>
#include <chrono>
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
constexpr int kStart = 0;  // forward-search root
constexpr int kGoal = 1;   // reverse-search source / permanent goal sample

// Normalised undirected edge id so (a,b) and (b,a) share one invalid_edges entry.
std::pair<int, int> edge_key(int a, int b) {
  return a < b ? std::pair<int, int>{a, b} : std::pair<int, int>{b, a};
}

using Pq = std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>,
                               std::greater<std::pair<double, int>>>;

// Reverse search: Dijkstra from the goal over the filtered complete graph. The
// resulting distances lower-bound the collision-free cost-to-go (the graph is a
// superset of the validated edges), so the heuristic is admissible; being a
// shortest-path metric on that graph it is also consistent for the forward search
// — the adaptive-heuristic idea of AIT* (Strub & Gammell 2020).
std::vector<double> reverse_search(SamplingSpace<Point>& space, const std::vector<Point>& points,
                                   const std::vector<std::vector<int>>& nbr) {
  int n = static_cast<int>(points.size());
  std::vector<double> dist(static_cast<size_t>(n), kInf);
  std::vector<char> settled(static_cast<size_t>(n), 0);
  dist[kGoal] = 0.0;
  Pq pq;
  pq.emplace(0.0, kGoal);
  while (!pq.empty()) {
    double d = pq.top().first;
    int u = pq.top().second;
    pq.pop();
    if (settled[static_cast<size_t>(u)]) continue;
    settled[static_cast<size_t>(u)] = 1;
    const Point& pu = points[static_cast<size_t>(u)];
    for (int v : nbr[static_cast<size_t>(u)]) {
      if (settled[static_cast<size_t>(v)]) continue;
      double nd = d + space.distance(pu, points[static_cast<size_t>(v)]);
      if (nd < dist[static_cast<size_t>(v)]) {
        dist[static_cast<size_t>(v)] = nd;
        pq.emplace(nd, v);
      }
    }
  }
  return dist;
}

std::vector<Point> extract_incumbent(const std::vector<Point>& points,
                                     const std::vector<int>& parent) {
  std::vector<Point> path;
  for (int node = kGoal; node != -1; node = parent[static_cast<size_t>(node)]) {
    path.push_back(points[static_cast<size_t>(node)]);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace

core::PlanResult<Point> FcitStarPlanner::plan(SamplingSpace<Point>& space, const Point& start,
                                              const Point& goal, TraceRecorder* recorder) {
  const int batch_size = params_.get_int("batch_size");
  const int max_batches = params_.get_int("max_batches");
  std::mt19937 rng(static_cast<unsigned>(params_.get_int("seed")));

  auto t0 = std::chrono::steady_clock::now();
  // Persistent across batches: sample array (0=start, 1=goal), the adaptive
  // invalid-motion set, and the incumbent cost / path. Everything else is rebuilt
  // fresh per batch.
  std::vector<Point> points{start, goal};
  std::set<std::pair<int, int>> invalid_edges;
  double c_best = kInf;
  std::vector<Point> best_path;
  int expanded = 0;

  for (int batch = 0; batch < max_batches; ++batch) {
    // --- 1. grow the batch (informed once a solution exists) -------------------
    int drawn = 0;
    for (int attempt = 0; attempt < batch_size * 40 && drawn < batch_size; ++attempt) {
      Point q = informed_sample(space, start, goal, c_best, rng);
      if (!space.is_state_valid(q)) continue;
      points.push_back(q);
      ++drawn;
      if (recorder) recorder->sample_drawn(q);
    }

    int n = static_cast<int>(points.size());

    // --- 2. fully connected adjacency, minus known-invalid motions -------------
    // No radius: FCIT* pairs every sample with every other, trading a denser
    // candidate graph for a search that can find shortcuts a radius-limited RGG
    // would miss (Wilson, Thomason, Kingston, Kavraki & Gammell 2025).
    std::vector<std::vector<int>> nbr(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        if (j != i && invalid_edges.find(edge_key(i, j)) == invalid_edges.end()) {
          nbr[static_cast<size_t>(i)].push_back(j);
        }
      }
    }

    // --- 3. reverse search: Dijkstra from the goal gives h_hat -----------------
    std::vector<double> h_hat = reverse_search(space, points, nbr);

    // --- 4. forward search: lazy-validated best-first over g + h_hat -----------
    std::vector<double> g(static_cast<size_t>(n), kInf);
    std::vector<int> parent(static_cast<size_t>(n), -1);
    std::vector<char> closed(static_cast<size_t>(n), 0);
    g[kStart] = 0.0;
    Pq open_heap;
    open_heap.emplace(h_hat[kStart], kStart);

    while (!open_heap.empty()) {
      double f = open_heap.top().first;
      int v = open_heap.top().second;
      open_heap.pop();
      if (closed[static_cast<size_t>(v)]) continue;
      if (f > g[static_cast<size_t>(v)] + h_hat[static_cast<size_t>(v)] + 1e-9) {
        continue;  // stale lazy-deletion entry: a cheaper g[v] superseded it
      }
      closed[static_cast<size_t>(v)] = 1;
      ++expanded;
      if (v == kGoal) break;  // goal settled with its final cost for this batch
      const Point& pv = points[static_cast<size_t>(v)];
      for (int x : nbr[static_cast<size_t>(v)]) {
        if (closed[static_cast<size_t>(x)]) continue;
        double edge_cost = space.distance(pv, points[static_cast<size_t>(x)]);
        double tentative = g[static_cast<size_t>(v)] + edge_cost;
        if (tentative >= g[static_cast<size_t>(x)]) continue;  // no improvement through v
        if (recorder) recorder->candidate_evaluated(points[static_cast<size_t>(x)], tentative);
        std::pair<int, int> ek = edge_key(v, x);
        if (invalid_edges.find(ek) != invalid_edges.end()) continue;
        if (!space.is_motion_valid(pv, points[static_cast<size_t>(x)])) {
          invalid_edges.insert(ek);  // adaptive feedback: never reconsider it
          continue;
        }
        bool was_connected = g[static_cast<size_t>(x)] < kInf;
        g[static_cast<size_t>(x)] = tentative;
        parent[static_cast<size_t>(x)] = v;
        open_heap.emplace(tentative + h_hat[static_cast<size_t>(x)], x);
        if (recorder) {
          if (was_connected) {
            recorder->rewire(points[static_cast<size_t>(x)], pv);
          } else {
            recorder->edge_added(points[static_cast<size_t>(x)], pv, edge_cost);
          }
        }
        if (x == kGoal && g[static_cast<size_t>(x)] < c_best) {
          c_best = g[static_cast<size_t>(x)];
          best_path = extract_incumbent(points, parent);
          if (recorder) recorder->path_found(best_path);
        }
      }
    }
  }

  // --- extract incumbent -----------------------------------------------------
  bool success = c_best < kInf;
  std::vector<Point> path = success ? best_path : std::vector<Point>{};
  double cost = success ? path_length(space, path) : 0.0;

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
