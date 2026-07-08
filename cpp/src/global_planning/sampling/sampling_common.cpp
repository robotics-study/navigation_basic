#include "navigation/global_planning/sampling/sampling_common.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation::global_planning {

int Tree::add(const Point& p, int parent_idx, double cumulative_cost) {
  int idx = static_cast<int>(nodes.size());
  nodes.push_back(p);
  parent.push_back(parent_idx);
  cost.push_back(cumulative_cost);
  children.emplace_back();
  if (parent_idx >= 0) children[parent_idx].push_back(idx);
  return idx;
}

void Tree::reparent(int child, int new_parent, double new_cost,
                    const SamplingSpace<Point>& space) {
  int old = parent[child];
  if (old >= 0) {
    auto& sib = children[old];
    sib.erase(std::remove(sib.begin(), sib.end(), child), sib.end());
  }
  parent[child] = new_parent;
  cost[child] = new_cost;
  children[new_parent].push_back(child);

  // Propagate the cost delta down the subtree so descendant costs stay correct.
  std::vector<int> stack{child};
  while (!stack.empty()) {
    int u = stack.back();
    stack.pop_back();
    for (int c : children[u]) {
      cost[c] = cost[u] + space.distance(nodes[u], nodes[c]);
      stack.push_back(c);
    }
  }
}

int nearest(const Tree& tree, const SamplingSpace<Point>& space, const Point& q) {
  int best = 0;
  double best_d = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(tree.nodes.size()); ++i) {
    double d = space.distance(tree.nodes[i], q);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

std::vector<int> near(const Tree& tree, const SamplingSpace<Point>& space, const Point& q,
                      double radius) {
  std::vector<int> out;
  for (int i = 0; i < static_cast<int>(tree.nodes.size()); ++i) {
    if (space.distance(tree.nodes[i], q) <= radius) out.push_back(i);
  }
  return out;
}

std::vector<Point> extract_path(const Tree& tree, int goal_idx) {
  std::vector<Point> path;
  for (int i = goal_idx; i >= 0; i = tree.parent[i]) path.push_back(tree.nodes[i]);
  std::reverse(path.begin(), path.end());
  return path;
}

double path_length(const SamplingSpace<Point>& space, const std::vector<Point>& path) {
  double total = 0.0;
  for (size_t i = 0; i + 1 < path.size(); ++i) total += space.distance(path[i], path[i + 1]);
  return total;
}

int choose_parent(const Tree& tree, const SamplingSpace<Point>& space, const Point& q_new,
                  const std::vector<int>& nbrs, int nearest_idx, double& out_cost,
                  TraceRecorder* recorder) {
  int best = nearest_idx;
  double best_cost = tree.cost[nearest_idx] + space.distance(tree.nodes[nearest_idx], q_new);
  for (int m : nbrs) {
    if (!space.is_motion_valid(tree.nodes[m], q_new)) continue;
    double c = tree.cost[m] + space.distance(tree.nodes[m], q_new);
    if (recorder) recorder->candidate_evaluated(tree.nodes[m], c);
    if (c < best_cost) {
      best_cost = c;
      best = m;
    }
  }
  out_cost = best_cost;
  return best;
}

void rewire_neighbors(Tree& tree, const SamplingSpace<Point>& space, int new_idx,
                      const std::vector<int>& nbrs, TraceRecorder* recorder) {
  for (int m : nbrs) {
    if (m == tree.parent[new_idx]) continue;
    double through = tree.cost[new_idx] + space.distance(tree.nodes[new_idx], tree.nodes[m]);
    if (through < tree.cost[m] && space.is_motion_valid(tree.nodes[new_idx], tree.nodes[m])) {
      tree.reparent(m, new_idx, through, space);
      if (recorder) recorder->rewire(tree.nodes[m], tree.nodes[new_idx]);
    }
  }
}

void emit_finished_sampling(TraceRecorder* recorder, bool success, double cost, int expanded_nodes,
                            int samples, int tree_size, int iterations, double runtime_sec) {
  if (!recorder) return;
  std::map<std::string, double> metrics{
      {"runtime_sec", runtime_sec},
      {"path_cost", success ? cost : 0.0},
      {"expanded_nodes", static_cast<double>(expanded_nodes)},
      {"samples", static_cast<double>(samples)},
      {"tree_size", static_cast<double>(tree_size)},
      {"iterations", static_cast<double>(iterations)}};
  recorder->planning_finished(success, metrics);
}

std::vector<int> near_points(const SamplingSpace<Point>& space, const std::vector<Point>& points,
                             const std::vector<int>& candidates, const Point& query, double radius) {
  std::vector<int> out;
  for (int i : candidates) {
    if (space.distance(points[i], query) <= radius) out.push_back(i);
  }
  return out;
}

std::vector<std::vector<int>> radius_neighbors(const SamplingSpace<Point>& space,
                                               const std::vector<Point>& points, double radius) {
  int n = static_cast<int>(points.size());
  std::vector<std::vector<int>> out(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      if (space.distance(points[i], points[j]) <= radius) {
        out[static_cast<size_t>(i)].push_back(j);
        out[static_cast<size_t>(j)].push_back(i);
      }
    }
  }
  return out;
}

double rgg_radius(double gamma, int n) {
  if (n <= 1) return std::numeric_limits<double>::infinity();
  double dn = static_cast<double>(n);
  return gamma * std::sqrt(std::log(dn) / dn);
}

double near_radius(const std::string& mode, double fixed_radius, double gamma, int n) {
  // "shrinking" contracts the near-set with tree size (Karaman & Frazzoli 2011);
  // every other declared mode keeps the constant so default runs stay unchanged.
  if (mode == "shrinking") return rgg_radius(gamma, n);
  return fixed_radius;
}

void emit_finished_batch(TraceRecorder* recorder, bool success, double cost, int expanded_nodes,
                         int samples, int tree_size, double runtime_sec) {
  if (!recorder) return;
  std::map<std::string, double> metrics{
      {"runtime_sec", runtime_sec},
      {"path_cost", success ? cost : 0.0},
      {"expanded_nodes", static_cast<double>(expanded_nodes)},
      {"samples", static_cast<double>(samples)},
      {"tree_size", static_cast<double>(tree_size)}};
  recorder->planning_finished(success, metrics);
}

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kTwoPi = 6.283185307179586;
}  // namespace

Point informed_sample(SamplingSpace<Point>& space, const Point& start, const Point& goal,
                      double c_best, std::mt19937& rng) {
  double c_min = space.distance(start, goal);
  if (c_best >= kInf || c_best <= c_min) return space.sample();
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  double cx = (start.x + goal.x) / 2.0;
  double cy = (start.y + goal.y) / 2.0;
  double r1 = c_best / 2.0;
  double r2 = std::sqrt(std::max(c_best * c_best - c_min * c_min, 0.0)) / 2.0;
  double theta = std::atan2(goal.y - start.y, goal.x - start.x);
  double ang = unit(rng) * kTwoPi;
  double rad = std::sqrt(unit(rng));
  double ux = rad * std::cos(ang) * r1;
  double uy = rad * std::sin(ang) * r2;
  double x = cx + std::cos(theta) * ux - std::sin(theta) * uy;
  double y = cy + std::sin(theta) * ux + std::cos(theta) * uy;
  return Point{x, y};
}

}  // namespace navigation::global_planning
