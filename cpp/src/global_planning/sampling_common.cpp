#include "navigation/global_planning/sampling_common.hpp"

#include <algorithm>
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

}  // namespace navigation::global_planning
