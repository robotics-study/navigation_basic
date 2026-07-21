#include "navigation/global_planning/sampling/sst.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include "navigation/global_planning/sampling/sampling_common.hpp"

namespace navigation::global_planning {

namespace {
// Waypoint spacing (m) for arc collision sampling. Small enough that the straight
// chord between consecutive unicycle waypoints stays well under a map cell, so
// is_motion_valid's supercover traversal catches every obstacle the curved arc crosses.
constexpr double kArcWaypointSpacing = 0.2;
// SST* radius decay per doubling of the iteration count (Li, Littlefield & Bekris
// 2016 §V): wide BestNear / witness radius early for exploration, tightened later so
// the tree converges toward the optimum. 0.9 per doubling shrinks to ~0.2x over 2^14
// iterations — fast enough to sharpen the solution, slow enough to keep connectivity.
constexpr double kSstStarShrink = 0.9;
constexpr double kInf = std::numeric_limits<double>::infinity();
}  // namespace

core::PlanResult<Point> SstPlanner::plan(SamplingSpace<Point>& space, const Point& start,
                                         const Point& goal, TraceRecorder* recorder) {
  const int max_iterations = params_.get_int("max_iterations");
  const double goal_bias = params_.get_float("goal_bias");
  const double goal_tol = params_.get_float("goal_tolerance");
  const double delta_bn0 = params_.get_float("delta_bn");
  const double delta_s0 = params_.get_float("delta_s");
  const double max_v = params_.get_float("max_velocity");
  const double max_omega = params_.get_float("max_omega");
  const double prop_min = params_.get_float("prop_duration_min");
  const double prop_max = params_.get_float("prop_duration_max");
  const bool sst_star = params_.get_bool("sst_star");
  // 차체는 inscribed disc — 점이 아니라 몸체가 벽을 비켜 가야 한다. capability 는
  // required_capabilities 로 선언되어 로드 단계에서 검증되고, 여기서는 같은 concrete
  // grid 가 두 view 를 함께 구현하므로 cross-cast 로 SE(2) view 를 얻는다.
  const core::Footprint footprint{params_.get_float("footprint_radius")};
  auto* se2 = dynamic_cast<core::SE2CollisionSpace<core::Pose>*>(&space);
  if (se2 == nullptr) {
    throw std::invalid_argument("sst: map does not provide SE2CollisionSpace");
  }
  std::mt19937 rng(static_cast<unsigned>(params_.get_int("seed")));
  std::uniform_real_distribution<double> unit(0.0, 1.0);

  auto t0 = std::chrono::steady_clock::now();

  // --- tree (parallel arrays; SST needs active / witness / pruning bookkeeping the
  // shared Tree does not model, so it keeps its own node arrays) -------------------
  std::vector<Point> pt{start};
  // Root heading faces the goal so early propagation is productive.
  std::vector<double> th{std::atan2(goal.y - start.y, goal.x - start.x)};
  std::vector<int> parent{-1};
  std::vector<double> cost{0.0};
  std::vector<std::vector<int>> children{{}};
  std::vector<std::vector<Point>> arc{{}};  // incoming dense arc (parent-excl..node-incl)
  std::unordered_set<int> active_ids{0};

  // --- witness set: witness point + its active representative index ----------------
  std::vector<Point> wpt{start};
  std::vector<int> wrep{0};

  auto radii = [&](int it) -> std::pair<double, double> {
    if (!sst_star) return {delta_bn0, delta_s0};
    int k = static_cast<int>(std::floor(std::log2(it + 2.0)));
    double scale = std::pow(kSstStarShrink, k);
    return {delta_bn0 * scale, delta_s0 * scale};
  };

  // BestNear: min-cost active node within delta_bn of the sample; fall back to the
  // nearest active node when the ball is empty (Li, Littlefield & Bekris 2016).
  auto best_near = [&](const Point& s, double delta_bn) -> int {
    int best = -1;
    double best_cost = kInf;
    for (int i : active_ids) {
      if (space.distance(pt[i], s) <= delta_bn && cost[i] < best_cost) {
        best_cost = cost[i];
        best = i;
      }
    }
    if (best != -1) return best;
    int near = -1;
    double near_d = kInf;
    for (int i : active_ids) {
      double d = space.distance(pt[i], s);
      if (d < near_d) {
        near_d = d;
        near = i;
      }
    }
    return near;
  };

  auto nearest_witness = [&](const Point& p) -> std::pair<int, double> {
    int best = 0;
    double best_d = space.distance(wpt[0], p);
    for (int i = 1; i < static_cast<int>(wpt.size()); ++i) {
      double d = space.distance(wpt[i], p);
      if (d < best_d) {
        best_d = d;
        best = i;
      }
    }
    return {best, best_d};
  };

  // Monte-Carlo forward propagation of a unicycle (x, y, theta) under a random constant
  // control (v, omega) held for a random duration. Euler integration; every waypoint is
  // collision-checked on its (x, y) projection and the chord to the previous is tested.
  auto propagate = [&](int from, double& out_theta,
                       std::vector<Point>& out_wps) -> bool {
    double v = unit(rng) * max_v;
    double omega = (unit(rng) * 2.0 - 1.0) * max_omega;
    double duration = prop_min + unit(rng) * (prop_max - prop_min);
    int n_sub = std::max(2, static_cast<int>(std::ceil(v * duration / kArcWaypointSpacing)));
    double dt = duration / n_sub;
    double x = pt[from].x, y = pt[from].y, theta = th[from];
    Point prev = pt[from];
    out_wps.clear();
    for (int k = 0; k < n_sub; ++k) {
      theta += omega * dt;
      x += v * std::cos(theta) * dt;
      y += v * std::sin(theta) * dt;
      Point p{x, y};
      // 웨이포인트 간격(0.2 m)과 반경이 같은 자릿수라 disc 사슬이 몸체 여유를 근사하고,
      // 점 수준 corner-cut 은 supercover chord 검사가 마저 막는다.
      if (se2->is_collision(footprint, core::Pose{x, y, theta})) return false;
      if (!space.is_state_valid(p) || !space.is_motion_valid(prev, p)) return false;
      out_wps.push_back(p);
      prev = p;
    }
    out_theta = theta;
    return true;
  };

  auto add_node = [&](const Point& p, double theta, int par, double c,
                      std::vector<Point> wps) -> int {
    int idx = static_cast<int>(pt.size());
    pt.push_back(p);
    th.push_back(theta);
    parent.push_back(par);
    cost.push_back(c);
    children.emplace_back();
    arc.push_back(std::move(wps));
    children[par].push_back(idx);
    active_ids.insert(idx);
    return idx;
  };

  // Deactivate a dominated representative and drop it + any inactive leaf ancestors
  // from the tree so the active set stays bounded ("sparse").
  auto prune_leaf_chain = [&](int node) {
    active_ids.erase(node);
    int cur = node;
    while (cur != -1 && active_ids.count(cur) == 0 && children[cur].empty()) {
      int par = parent[cur];
      if (par != -1) {
        auto& sib = children[par];
        sib.erase(std::remove(sib.begin(), sib.end(), cur), sib.end());
      }
      cur = par;
    }
  };

  auto reconstruct = [&](int idx) -> std::vector<Point> {
    std::vector<std::vector<Point>> segs;
    for (int node = idx; parent[node] != -1; node = parent[node]) segs.push_back(arc[node]);
    std::vector<Point> path{start};
    for (auto it = segs.rbegin(); it != segs.rend(); ++it)
      path.insert(path.end(), it->begin(), it->end());
    return path;
  };

  int best_goal = -1;
  double best_cost = kInf;
  int total_added = 0;
  int iterations = 0;
  std::vector<Point> waypoints;
  for (int it = 0; it < max_iterations; ++it) {
    ++iterations;
    auto [delta_bn, delta_s] = radii(it);
    Point s_sample = (unit(rng) < goal_bias) ? goal : space.sample();
    if (recorder) recorder->sample_drawn(s_sample);

    int selected = best_near(s_sample, delta_bn);
    double new_theta = 0.0;
    if (!propagate(selected, new_theta, waypoints)) continue;
    const Point& new_pt = waypoints.back();
    std::vector<Point> with_parent{pt[selected]};
    with_parent.insert(with_parent.end(), waypoints.begin(), waypoints.end());
    double new_cost = cost[selected] + path_length(space, with_parent);

    // IsNodeLocallyBest: locate (or create) the governing witness, then keep the node
    // only if it beats that witness's current representative.
    auto [wi, wd] = nearest_witness(new_pt);
    if (wd > delta_s) {
      wi = static_cast<int>(wpt.size());
      wpt.push_back(new_pt);
      wrep.push_back(-1);
    }
    int peer = wrep[wi];
    if (peer != -1 && new_cost >= cost[peer]) continue;

    int ci = add_node(new_pt, new_theta, selected, new_cost, waypoints);
    ++total_added;
    if (recorder) {
      Point prev = pt[selected];
      for (const Point& w : waypoints) {
        recorder->edge_added(w, prev, space.distance(prev, w));
        prev = w;
      }
    }
    wrep[wi] = ci;
    if (peer != -1) {
      // rewire marks the witness representative moving to the cheaper node, so the viz
      // shows sparsification (the old branch is pruned away).
      if (recorder) recorder->rewire(new_pt, pt[peer]);
      prune_leaf_chain(peer);
    }

    if (space.distance(new_pt, goal) <= goal_tol && new_cost < best_cost) {
      best_cost = new_cost;
      best_goal = ci;
      if (recorder) recorder->path_found(reconstruct(ci));
    }
  }

  core::PlanResult<Point> result;
  if (best_goal >= 0) {
    result.success = true;
    result.path = reconstruct(best_goal);
    result.cost = path_length(space, result.path);
  }
  int active_count = static_cast<int>(active_ids.size());
  result.stats = {total_added, iterations, iterations, active_count};
  double rt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  emit_finished_sampling(recorder, result.success, result.cost, total_added, iterations,
                         active_count, iterations, rt);
  return result;
}

}  // namespace navigation::global_planning
