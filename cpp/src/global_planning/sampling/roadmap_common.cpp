#include "navigation/global_planning/sampling/roadmap_common.hpp"

#include <algorithm>
#include <limits>
#include <queue>

#include "navigation/global_planning/sampling/sampling_common.hpp"

namespace navigation::global_planning {

int Roadmap::add_node(const Point& p) {
  int idx = static_cast<int>(nodes.size());
  nodes.push_back(p);
  adj.emplace_back();
  return idx;
}

void Roadmap::add_edge(int a, int b, double cost) {
  adj[static_cast<size_t>(a)].emplace_back(b, cost);
  adj[static_cast<size_t>(b)].emplace_back(a, cost);
}

void connect(const SamplingSpace<Point>& space, Roadmap& roadmap, int idx, double radius,
             TraceRecorder* recorder) {
  const Point& node = roadmap.nodes[static_cast<size_t>(idx)];
  std::vector<int> earlier(static_cast<size_t>(idx));
  for (int j = 0; j < idx; ++j) earlier[static_cast<size_t>(j)] = j;
  for (int j : near_points(space, roadmap.nodes, earlier, node, radius)) {
    const Point& other = roadmap.nodes[static_cast<size_t>(j)];
    if (space.is_motion_valid(other, node)) {
      double cost = space.distance(other, node);
      roadmap.add_edge(idx, j, cost);
      if (recorder) recorder->edge_added(node, other, cost);
    }
  }
}

std::vector<Point> dijkstra(const Roadmap& roadmap, int start, int goal, TraceRecorder* recorder,
                            double& out_cost, int& out_expanded) {
  const double inf = std::numeric_limits<double>::infinity();
  int n = static_cast<int>(roadmap.nodes.size());
  std::vector<double> dist(static_cast<size_t>(n), inf);
  std::vector<int> parent(static_cast<size_t>(n), -1);
  dist[static_cast<size_t>(start)] = 0.0;
  using Entry = std::pair<double, int>;  // (dist, node)
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
  pq.emplace(0.0, start);
  int expanded = 0;
  while (!pq.empty()) {
    double d = pq.top().first;
    int u = pq.top().second;
    pq.pop();
    if (d > dist[static_cast<size_t>(u)]) continue;
    ++expanded;
    if (recorder) recorder->node_expanded(roadmap.nodes[static_cast<size_t>(u)], d);
    if (u == goal) break;
    for (const auto& [v, w] : roadmap.adj[static_cast<size_t>(u)]) {
      double nd = d + w;
      if (nd < dist[static_cast<size_t>(v)]) {
        dist[static_cast<size_t>(v)] = nd;
        parent[static_cast<size_t>(v)] = u;
        pq.emplace(nd, v);
      }
    }
  }
  out_expanded = expanded;
  if (dist[static_cast<size_t>(goal)] == inf) {
    out_cost = 0.0;
    return {};
  }
  out_cost = dist[static_cast<size_t>(goal)];
  std::vector<Point> path;
  for (int node = goal; node != -1; node = parent[static_cast<size_t>(node)]) {
    path.push_back(roadmap.nodes[static_cast<size_t>(node)]);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

void sample_free(SamplingSpace<Point>& space, Roadmap& roadmap, int num_samples,
                 TraceRecorder* recorder) {
  int drawn = 0;
  // Cap attempts so a nearly-full map cannot loop forever chasing free states.
  for (int attempt = 0; attempt < num_samples * 20 && drawn < num_samples; ++attempt) {
    Point q = space.sample();
    if (!space.is_state_valid(q)) continue;
    roadmap.add_node(q);
    ++drawn;
    if (recorder) recorder->sample_drawn(q);
  }
}

}  // namespace navigation::global_planning
