#include "navigation/global_planning/search/hybrid_astar.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "navigation/global_planning/search/discrete_search.hpp"  // emit_finished_discrete

namespace navigation::global_planning {

using core::Footprint;
using core::PlanResult;
using core::PlanStats;
using core::Pose;
using core::SE2CollisionSpace;
using core::TraceRecorder;

namespace {

constexpr double kTwoPi = 2.0 * M_PI;
// Below this |curvature| an arc is integrated as a straight segment. For an odd
// num_steering the middle primitive is exactly kappa = 0; the guard also covers the
// near-zero rounding of the even-split endpoints.
constexpr double kStraightEps = 1e-9;

// Planner-internal closed-set bin: floor(x/xy_res), floor(y/xy_res), heading bucket.
// Pure planner arithmetic on world coords — never map.world_to_cell (no grid-index
// leakage; the map's cell resolution is a different concern from search granularity).
struct PoseKey {
  int gx = 0;
  int gy = 0;
  int gt = 0;
  bool operator==(const PoseKey& o) const { return gx == o.gx && gy == o.gy && gt == o.gt; }
};

// Custom hasher (NOT a std::hash specialization — specializing std for an internal-
// linkage type is ill-formed). Mixes three ints like core::Cell's hash.
struct PoseKeyHash {
  std::size_t operator()(const PoseKey& k) const noexcept {
    std::size_t h = std::hash<int>()(k.gx);
    h ^= std::hash<int>()(k.gy) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    h ^= std::hash<int>()(k.gt) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    return h;
  }
};

// [0, 2*pi). floor delegates to libm; identical to the Python mirror on one machine.
double wrap_angle(double theta) { return theta - kTwoPi * std::floor(theta / kTwoPi); }

// Signed shortest angular difference magnitude |a - b|, in [0, pi]. No trig.
double ang_diff(double a, double b) {
  double d = wrap_angle(a - b);
  if (d > M_PI) d -= kTwoPi;
  return std::abs(d);
}

// One constant-curvature motion primitive: signed curvature, signed arc length
// (negative = reverse), and the reverse flag (drives reverse_penalty).
struct Primitive {
  double kappa;
  double length;
  bool reverse;
};

// Advance a pose by a constant-curvature arc of signed length L. theta is additive
// (no trig for heading); x, y integrate the arc exactly (Dolgov et al. 2008).
Pose integrate(const Pose& p, double kappa, double L) {
  double theta2 = p.theta + kappa * L;
  if (std::abs(kappa) < kStraightEps) {
    return {p.x + L * std::cos(p.theta), p.y + L * std::sin(p.theta), theta2};
  }
  return {p.x + (std::sin(theta2) - std::sin(p.theta)) / kappa,
          p.y - (std::cos(theta2) - std::cos(p.theta)) / kappa, theta2};
}

}  // namespace

PlanResult<Pose> HybridAStarPlanner::plan(SE2CollisionSpace<Pose>& space, const Pose& start,
                                          const Pose& goal, TraceRecorder* recorder) {
  const double min_turn_radius = params_.get_float("min_turn_radius");
  const double arc_step = params_.get_float("arc_step");
  const int num_steering = params_.get_int("num_steering");
  const int theta_bins = params_.get_int("theta_bins");
  const double xy_resolution = params_.get_float("xy_resolution");
  const Footprint footprint{params_.get_float("footprint_radius")};
  const bool allow_reverse = params_.get_bool("allow_reverse");
  const double reverse_penalty = params_.get_float("reverse_penalty");
  const double steer_penalty = params_.get_float("steer_penalty");
  const double goal_pos_tol = params_.get_float("goal_pos_tolerance");
  const double goal_heading_tol = params_.get_float("goal_heading_tolerance");

  const double kappa_max = 1.0 / min_turn_radius;
  const double bin_theta = kTwoPi / static_cast<double>(theta_bins);
  // Sub-sampling spacing tied to CLEARANCE (footprint radius), floored at 2 so
  // consecutive footprint discs overlap (no tunnelling) and every arc emits >=2
  // chords (renders curved). Independent of the closed-set bin resolution.
  const int n_sub =
      std::max(2, static_cast<int>(std::ceil(arc_step / footprint.inscribed_radius)));

  // Fixed emission order (forward fan by ascending kappa index, then reverse fan) so
  // both languages generate successors identically. num_steering >= 2 guards the
  // (num_steering - 1) denominator (config min 2).
  std::vector<Primitive> primitives;
  const double kappa_span = 2.0 * kappa_max / static_cast<double>(num_steering - 1);
  for (int i = 0; i < num_steering; ++i) {
    primitives.push_back({-kappa_max + i * kappa_span, arc_step, false});
  }
  if (allow_reverse) {
    for (int i = 0; i < num_steering; ++i) {
      primitives.push_back({-kappa_max + i * kappa_span, -arc_step, true});
    }
  }

  auto bin_of = [&](const Pose& p) -> PoseKey {
    int gx = static_cast<int>(std::floor(p.x / xy_resolution));
    int gy = static_cast<int>(std::floor(p.y / xy_resolution));
    int gt = static_cast<int>(std::floor(wrap_angle(p.theta) / bin_theta)) % theta_bins;
    return {gx, gy, gt};
  };
  auto h = [&](const Pose& p) -> double {
    double dx = p.x - goal.x, dy = p.y - goal.y;
    return std::sqrt(dx * dx + dy * dy);  // sqrt, NOT hypot (hypot is relaxed-accuracy)
  };
  auto is_goal = [&](const Pose& p) -> bool {
    double dx = p.x - goal.x, dy = p.y - goal.y;
    return dx * dx + dy * dy <= goal_pos_tol * goal_pos_tol &&
           ang_diff(p.theta, goal.theta) <= goal_heading_tol;
  };

  auto t0 = std::chrono::steady_clock::now();

  struct QItem {
    double f;
    unsigned long long seq;  // insertion order; FIFO tie-break, identical to A*/Theta*
    PoseKey bin;
  };
  struct Greater {
    bool operator()(const QItem& a, const QItem& b) const {
      return a.f > b.f || (a.f == b.f && a.seq > b.seq);
    }
  };
  std::priority_queue<QItem, std::vector<QItem>, Greater> open;
  std::unordered_map<PoseKey, double, PoseKeyHash> g;
  std::unordered_map<PoseKey, Pose, PoseKeyHash> pose_of;  // best continuous pose per bin
  // Incoming dense arc per bin: parent bin + the sub-poses (parent-exclusive .. this bin).
  std::unordered_map<PoseKey, std::pair<PoseKey, std::vector<Pose>>, PoseKeyHash> came_from;
  std::unordered_set<PoseKey, PoseKeyHash> closed;
  unsigned long long seq = 0;

  const PoseKey start_bin = bin_of(start);
  g[start_bin] = 0.0;
  pose_of[start_bin] = start;
  open.push({h(start), seq++, start_bin});

  bool found = false;
  PoseKey goal_bin{};
  int expanded = 0;

  while (!open.empty()) {
    PoseKey b = open.top().bin;
    open.pop();
    if (closed.count(b)) continue;  // stale duplicate: bin already settled
    closed.insert(b);
    const Pose p = pose_of.at(b);  // expand the bin's best pose -> came_from stays continuous
    ++expanded;
    if (recorder) recorder->node_expanded(p, g.at(b));
    if (is_goal(p)) {
      found = true;
      goal_bin = b;
      break;
    }
    for (const Primitive& prim : primitives) {
      // Sub-sample the arc: reuse the sub-poses for BOTH collision and viz chords.
      std::vector<Pose> subs;
      subs.reserve(static_cast<std::size_t>(n_sub));
      bool blocked = false;
      for (int j = 1; j <= n_sub; ++j) {
        Pose s = integrate(p, prim.kappa, prim.length * (static_cast<double>(j) / n_sub));
        if (space.is_collision(footprint, s)) {
          blocked = true;
          break;
        }
        subs.push_back(s);
      }
      if (blocked) continue;
      const Pose child = subs.back();
      const PoseKey b2 = bin_of(child);
      if (closed.count(b2)) continue;
      const double abs_l = std::abs(prim.length);
      const double cost = abs_l * (prim.reverse ? reverse_penalty : 1.0) +
                          steer_penalty * std::abs(prim.kappa) * abs_l;
      const double cand = g.at(b) + cost;
      auto it = g.find(b2);
      if (it == g.end() || cand < it->second) {
        g[b2] = cand;
        pose_of[b2] = child;
        came_from[b2] = {b, subs};
        if (recorder) {
          recorder->candidate_evaluated(child, cand);
          // Arc renders as a chain of straight edge_added chords (dense -> smooth curve;
          // no replay arc logic). Parent of the first sub-pose is the expanded pose.
          const double chord = abs_l / static_cast<double>(n_sub);
          const Pose* parent = &p;
          for (const Pose& s : subs) {
            recorder->edge_added(s, *parent, chord);
            parent = &s;
          }
        }
        open.push({cand + h(child), seq++, b2});
      }
    }
  }

  PlanResult<Pose> result;
  if (found) {
    // Walk came_from goal->start, collecting dense arcs, then emit start + arcs forward.
    std::vector<std::vector<Pose>> arcs;
    PoseKey b = goal_bin;
    while (!(b == start_bin)) {
      const auto& entry = came_from.at(b);
      arcs.push_back(entry.second);
      b = entry.first;
    }
    std::vector<Pose> path{start};
    for (auto ri = arcs.rbegin(); ri != arcs.rend(); ++ri) {
      path.insert(path.end(), ri->begin(), ri->end());
    }
    result.success = true;
    result.path = std::move(path);
    result.cost = g.at(goal_bin);
    if (recorder) recorder->path_found(result.path);  // emit_finished_discrete does not
  }
  result.stats.expanded_nodes = expanded;
  double rt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  emit_finished_discrete(recorder, result.success, result.cost, result.stats, rt);
  return result;
}

}  // namespace navigation::global_planning
