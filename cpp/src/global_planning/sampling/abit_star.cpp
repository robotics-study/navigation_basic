#include "navigation/global_planning/sampling/abit_star.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <set>
#include <tuple>
#include <vector>

#include "navigation/global_planning/sampling/sampling_common.hpp"

namespace navigation::global_planning {

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();

// Linear per-batch decay initial -> final (batch 0 -> last), monotone in `batch`.
// ARA*-style ε schedule (Likhachev, Gordon & Thrun 2003): start inflated for a
// quick first solution, relax to `final_val` on the last batch to recover
// optimality. A single batch keeps the initial value (nothing later can tighten).
double schedule(int batch, int max_batches, double initial, double final_val) {
  if (max_batches <= 1) return initial;
  double frac = static_cast<double>(batch) / static_cast<double>(max_batches - 1);
  return initial + (final_val - initial) * frac;
}
}  // namespace

core::PlanResult<Point> AbitStarPlanner::plan(SamplingSpace<Point>& space, const Point& start,
                                              const Point& goal, TraceRecorder* recorder) {
  const int batch_size = params_.get_int("batch_size");
  const int max_batches = params_.get_int("max_batches");
  const double gamma = params_.get_float("gamma");
  const double inflation0 = params_.get_float("inflation_factor");
  const double inflation_final = params_.get_float("inflation_final");
  const double truncation0 = params_.get_float("truncation_factor");
  std::mt19937 rng(static_cast<unsigned>(params_.get_int("seed")));

  auto t0 = std::chrono::steady_clock::now();
  // Sample array grows across batches. 0 = start (root), 1 = goal (permanent
  // sample). Tree membership + cost-to-come live in parallel arrays.
  std::vector<Point> points{start, goal};
  std::vector<double> g_t{0.0, kInf};  // cost-to-come in the tree (inf = not in tree)
  std::vector<int> parent{-1, -1};
  std::vector<std::set<int>> children(2);
  std::vector<char> in_tree{1, 0};
  std::set<int> samples{1};  // unconnected samples (goal starts here)
  double c_best = kInf;
  int expanded = 0;

  auto h_hat = [&](int i) { return space.distance(points[static_cast<size_t>(i)], goal); };
  auto g_hat = [&](int i) { return space.distance(start, points[static_cast<size_t>(i)]); };

  // Rewiring changed g_t[root]; push the delta to its subtree so queue keys and
  // the reported cost stay consistent.
  auto propagate = [&](int root) {
    std::vector<int> stack{root};
    while (!stack.empty()) {
      int u = stack.back();
      stack.pop_back();
      for (int c : children[static_cast<size_t>(u)]) {
        g_t[static_cast<size_t>(c)] =
            g_t[static_cast<size_t>(u)] +
            space.distance(points[static_cast<size_t>(u)], points[static_cast<size_t>(c)]);
        stack.push_back(c);
      }
    }
  };

  for (int batch = 0; batch < max_batches; ++batch) {
    // ε_infl decays to inflation_final; ε_trunc decays to 1.0 (no truncation) so
    // the final batch runs admissible, untruncated — i.e. exactly BIT*.
    const double eps_infl = schedule(batch, max_batches, inflation0, inflation_final);
    const double eps_trunc = schedule(batch, max_batches, truncation0, 1.0);

    // --- prune samples that can no longer improve the incumbent ----------------
    // Admissible bound (un-inflated): inflation must never drop a sample that a
    // later, less-inflated batch could still route through.
    if (c_best < kInf) {
      std::set<int> kept;
      for (int x : samples) {
        if (g_hat(x) + h_hat(x) < c_best) kept.insert(x);
      }
      samples.swap(kept);
    }

    // --- draw a new batch (informed once a solution exists) --------------------
    int drawn = 0;
    for (int attempt = 0; attempt < batch_size * 40 && drawn < batch_size; ++attempt) {
      Point q = informed_sample(space, start, goal, c_best, rng);
      if (!space.is_state_valid(q)) continue;
      int idx = static_cast<int>(points.size());
      points.push_back(q);
      g_t.push_back(kInf);
      parent.push_back(-1);
      children.emplace_back();
      in_tree.push_back(0);
      samples.insert(idx);
      ++drawn;
      if (recorder) recorder->sample_drawn(q);
    }

    int n = static_cast<int>(points.size());
    double radius = rgg_radius(gamma, n);
    std::vector<std::vector<int>> nbr = radius_neighbors(space, points, radius);

    // --- queues: vertices to expand + candidate edges --------------------------
    // Keys inflate the cost-to-go term by ε_infl (weighted-A*/ARA* over the RGG,
    // Strub & Gammell 2020): early batches order greedily toward the goal.
    using VEntry = std::pair<double, int>;        // (key, v)
    using EEntry = std::tuple<double, int, int>;  // (key, v, x)
    std::priority_queue<VEntry, std::vector<VEntry>, std::greater<VEntry>> q_v;
    std::priority_queue<EEntry, std::vector<EEntry>, std::greater<EEntry>> q_e;
    for (int v = 0; v < n; ++v) {
      if (in_tree[static_cast<size_t>(v)])
        q_v.emplace(g_t[static_cast<size_t>(v)] + eps_infl * h_hat(v), v);
    }
    std::set<int> expanded_v;

    auto expand_vertex = [&](int v) {
      for (int x : nbr[static_cast<size_t>(v)]) {
        double d = space.distance(points[static_cast<size_t>(v)], points[static_cast<size_t>(x)]);
        if (samples.count(x)) {
          // Candidate edge to an unconnected sample. Enqueue gate stays
          // admissible; ordering key is inflated.
          if (g_hat(v) + d + h_hat(x) < c_best) {
            q_e.emplace(g_t[static_cast<size_t>(v)] + d + eps_infl * h_hat(x), v, x);
          }
        } else if (in_tree[static_cast<size_t>(x)] && x != parent[static_cast<size_t>(v)]) {
          // Candidate rewiring of an existing vertex through v.
          if (g_hat(v) + d + h_hat(x) < c_best &&
              g_t[static_cast<size_t>(v)] + d < g_t[static_cast<size_t>(x)]) {
            q_e.emplace(g_t[static_cast<size_t>(v)] + d + eps_infl * h_hat(x), v, x);
          }
        }
      }
    };

    // Pop stale vertex-queue entries (already expanded or key superseded by a
    // cheaper cost-to-come) and return the best remaining key, or inf.
    auto best_v = [&]() -> double {
      while (!q_v.empty()) {
        double key = q_v.top().first;
        int v = q_v.top().second;
        if (expanded_v.count(v) ||
            key > g_t[static_cast<size_t>(v)] + eps_infl * h_hat(v) + 1e-9) {
          q_v.pop();
          continue;
        }
        return key;
      }
      return kInf;
    };
    auto best_e = [&]() -> double { return q_e.empty() ? kInf : std::get<0>(q_e.top()); };

    // Truncation threshold: stop the batch once no edge can pull the incumbent
    // below c_best / ε_trunc (Strub & Gammell 2020). ε_trunc = 1 -> BIT*'s c_best.
    double trunc_bound = c_best / eps_trunc;

    while (true) {
      // Drain vertices whose expansion could beat the best queued edge.
      while (true) {
        double bv = best_v();  // pops stale entries as a side effect
        if (q_v.empty() || bv > best_e()) break;
        int v = q_v.top().second;
        q_v.pop();
        if (expanded_v.count(v)) continue;
        expanded_v.insert(v);
        expand_vertex(v);
      }
      if (q_e.empty()) break;
      int vm = std::get<1>(q_e.top());
      int xm = std::get<2>(q_e.top());
      q_e.pop();
      double d_vm_xm =
          space.distance(points[static_cast<size_t>(vm)], points[static_cast<size_t>(xm)]);
      // Admissible (un-inflated) estimate of the solution through this edge.
      double a_key = g_t[static_cast<size_t>(vm)] + d_vm_xm + h_hat(xm);
      // Truncation: the best-ordered edge can no longer improve past the bound.
      if (a_key >= trunc_bound) break;
      // Can this edge improve the tree cost-to-come of x_m at all?
      if (g_t[static_cast<size_t>(vm)] + d_vm_xm >= g_t[static_cast<size_t>(xm)]) continue;
      if (!space.is_motion_valid(points[static_cast<size_t>(vm)], points[static_cast<size_t>(xm)]))
        continue;
      double edge_cost = d_vm_xm;
      double new_g = g_t[static_cast<size_t>(vm)] + edge_cost;
      if (new_g + h_hat(xm) >= c_best) continue;
      if (new_g >= g_t[static_cast<size_t>(xm)]) continue;
      // Accept the edge: connect a sample or rewire a vertex under v_m.
      if (in_tree[static_cast<size_t>(xm)]) {
        children[static_cast<size_t>(parent[static_cast<size_t>(xm)])].erase(xm);
      } else {
        samples.erase(xm);
        in_tree[static_cast<size_t>(xm)] = 1;
      }
      parent[static_cast<size_t>(xm)] = vm;
      g_t[static_cast<size_t>(xm)] = new_g;
      children[static_cast<size_t>(vm)].insert(xm);
      propagate(xm);
      ++expanded;
      expanded_v.erase(xm);  // improved: allow re-expansion this batch
      q_v.emplace(g_t[static_cast<size_t>(xm)] + eps_infl * h_hat(xm), xm);
      if (recorder)
        recorder->edge_added(points[static_cast<size_t>(xm)], points[static_cast<size_t>(vm)],
                             edge_cost);
      if (in_tree[1] && g_t[1] < c_best) {
        c_best = g_t[1];
        trunc_bound = c_best / eps_trunc;
        if (recorder) recorder->candidate_evaluated(goal, c_best);
      }
    }
  }

  // --- extract incumbent -----------------------------------------------------
  bool success = in_tree[1] && g_t[1] < kInf;
  std::vector<Point> path;
  if (success) {
    for (int node = 1; node != -1; node = parent[static_cast<size_t>(node)]) {
      path.push_back(points[static_cast<size_t>(node)]);
    }
    std::reverse(path.begin(), path.end());
  }
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
