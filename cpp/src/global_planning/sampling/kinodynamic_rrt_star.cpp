#include "navigation/global_planning/sampling/kinodynamic_rrt_star.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <limits>
#include <optional>
#include <random>
#include <set>
#include <utility>
#include <vector>

namespace navigation::global_planning {

using core::Point;
using core::SamplingSpace;
using core::TraceRecorder;

namespace {

// Double-integrator planning state: world position + velocity (x, y, vx, vy). Only
// the (x, y) projection is ever handed to the SamplingSpace map (no velocity).
using State4 = std::array<double, 4>;

constexpr double kEps = 1e-9;
// Nearest-neighbour is selected by the exact optimal cost, but computing it against
// every node each iteration (a quartic solve per node) is intractable. The
// double-integrator cost is dominated by the position gap, so a Euclidean prefilter to
// the K closest nodes preserves the true optimum among candidates with high probability;
// the winner is chosen by exact optimal cost. K-nearest RRT* is itself analysed
// (Karaman & Frazzoli 2011).
constexpr int kNearestCandidates = 16;
// Cap the choose-parent / rewire neighbourhood so per-iteration work stays bounded on
// dense trees (k-nearest RRT* variant).
constexpr int kMaxNeighbors = 16;
// Trajectory collision sampling: one waypoint per this many metres of straight
// separation (is_motion_valid supercover fills each sub-segment), floored/capped.
constexpr double kCollisionSpacing = 0.3;
constexpr int kMinWaypoints = 4;
constexpr int kMaxWaypoints = 64;

// Per-axis-summed (C1, C2, C3) of the double-integrator cost-to-go c(t) (Webb & van
// den Berg 2013): c(t) = t + r*(C3/t^3 + C2/t^2 + C1/t).
struct Coeffs {
  double c1, c2, c3;
};

Coeffs steer_coeffs(const State4& x0, const State4& x1) {
  Coeffs k{0.0, 0.0, 0.0};
  const std::array<std::array<double, 3>, 2> axes = {{
      {x1[0] - x0[0], x0[2], x1[2]},  // x-axis: position gap a, endpoint velocities
      {x1[1] - x0[1], x0[3], x1[3]},  // y-axis
  }};
  for (const auto& ax : axes) {
    const double a = ax[0], v0 = ax[1], v1 = ax[2];
    k.c3 += 12.0 * a * a;
    k.c2 += -12.0 * a * (v0 + v1);
    k.c1 += 4.0 * (v0 * v0 + v0 * v1 + v1 * v1);
  }
  return k;
}

double cost_at(double t, double r, const Coeffs& k) {
  return t + r * (k.c3 / (t * t * t) + k.c2 / (t * t) + k.c1 / t);
}

// Real+complex roots of a monic degree-4 polynomial (coeffs highest-first) via
// Durand-Kerner (Weierstrass) iteration; matches numpy.roots to ~1e-14. Used instead
// of Eigen (not a dependency) to keep the quartic solve self-contained.
std::array<std::complex<double>, 4> quartic_roots(const std::array<double, 5>& coeffs) {
  std::array<std::complex<double>, 4> roots{};
  const std::complex<double> seed(0.4, 0.9);
  for (int i = 0; i < 4; ++i) roots[i] = std::pow(seed, i);
  auto eval = [&](std::complex<double> x) {
    std::complex<double> r(0.0, 0.0);
    for (double c : coeffs) r = r * x + c;  // coeffs[0] == 1 (monic)
    return r;
  };
  for (int iter = 0; iter < 100; ++iter) {
    double max_delta = 0.0;
    std::array<std::complex<double>, 4> next = roots;
    for (int i = 0; i < 4; ++i) {
      std::complex<double> denom(1.0, 0.0);
      for (int j = 0; j < 4; ++j) {
        if (j != i) denom *= (roots[i] - roots[j]);
      }
      std::complex<double> corr = eval(roots[i]) / denom;
      next[i] = roots[i] - corr;
      max_delta = std::max(max_delta, std::abs(corr));
    }
    roots = next;
    if (max_delta < 1e-14) break;
  }
  return roots;
}

// Fixed-final-state free-final-time optimal cost and arrival time tau* (Webb & van den
// Berg 2013). Returns {0, 0} when the states coincide, {inf, 0} if no positive root.
std::pair<double, double> optimal_cost(const State4& x0, const State4& x1, double r) {
  Coeffs k = steer_coeffs(x0, x1);
  if (std::abs(k.c1) < kEps && std::abs(k.c2) < kEps && std::abs(k.c3) < kEps) {
    return {0.0, 0.0};
  }
  // c'(t)=0 cleared by t^4 is the depressed quartic (no cubic term):
  //   t^4 - r*C1*t^2 - 2r*C2*t - 3r*C3 = 0.
  std::array<double, 5> coeffs = {1.0, 0.0, -r * k.c1, -2.0 * r * k.c2, -3.0 * r * k.c3};
  double best_cost = std::numeric_limits<double>::infinity();
  double best_tau = 0.0;
  for (const auto& root : quartic_roots(coeffs)) {
    if (std::abs(root.imag()) > kEps * (1.0 + std::abs(root.real()))) continue;
    double t = root.real();
    if (t <= kEps) continue;
    double c = cost_at(t, r, k);
    if (c < best_cost) {
      best_cost = c;
      best_tau = t;
    }
  }
  return {best_cost, best_tau};
}

// Optimal (min-integral||u||^2) cubic position at normalised time s in [0,1] along tau*.
Point hermite_xy(const State4& x0, const State4& x1, double tau, double s) {
  const double s2 = s * s, s3 = s2 * s;
  const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
  const double h10 = s3 - 2.0 * s2 + s;
  const double h01 = -2.0 * s3 + 3.0 * s2;
  const double h11 = s3 - s2;
  return Point{h00 * x0[0] + h10 * tau * x0[2] + h01 * x1[0] + h11 * tau * x1[2],
               h00 * x0[1] + h10 * tau * x0[3] + h01 * x1[1] + h11 * tau * x1[3]};
}

// (x, y) waypoints of the optimal trajectory, parent-exclusive (s>0..1).
std::vector<Point> trajectory_xy(const State4& x0, const State4& x1, double tau) {
  const double gap = std::hypot(x1[0] - x0[0], x1[1] - x0[1]);
  int n = std::max(kMinWaypoints, static_cast<int>(std::ceil(gap / kCollisionSpacing)));
  n = std::min(kMaxWaypoints, n);
  std::vector<Point> traj;
  traj.reserve(static_cast<size_t>(n));
  for (int k = 1; k <= n; ++k) {
    traj.push_back(hermite_xy(x0, x1, tau, static_cast<double>(k) / n));
  }
  return traj;
}

// Search tree over double-integrator states (parallel arrays), mirroring the geometric
// RRT* tree but keyed on optimal-control cost. Each node stores its incoming edge cost
// + dense trajectory so a rewire propagates cumulative cost through a subtree without
// re-solving steering, and path reconstruction emits the true curved trajectory.
struct KinoTree {
  std::vector<State4> states;
  std::vector<int> parent;
  std::vector<double> cost;
  std::vector<double> edge_cost;
  std::vector<std::vector<Point>> incoming;
  std::vector<std::vector<int>> children;

  explicit KinoTree(const State4& root) {
    states.push_back(root);
    parent.push_back(-1);
    cost.push_back(0.0);
    edge_cost.push_back(0.0);
    incoming.emplace_back();
    children.emplace_back();
  }

  int size() const { return static_cast<int>(states.size()); }

  int add(const State4& state, int parent_idx, double e_cost, std::vector<Point> traj) {
    int idx = size();
    states.push_back(state);
    parent.push_back(parent_idx);
    cost.push_back(cost[parent_idx] + e_cost);
    edge_cost.push_back(e_cost);
    incoming.push_back(std::move(traj));
    children.emplace_back();
    children[parent_idx].push_back(idx);
    return idx;
  }

  void reparent(int child, int new_parent, double e_cost, std::vector<Point> traj) {
    int old = parent[child];
    if (old >= 0) {
      auto& sib = children[old];
      sib.erase(std::remove(sib.begin(), sib.end(), child), sib.end());
    }
    parent[child] = new_parent;
    edge_cost[child] = e_cost;
    incoming[child] = std::move(traj);
    cost[child] = cost[new_parent] + e_cost;
    children[new_parent].push_back(child);
    // Push the cost delta down the subtree (edge costs unchanged; only cumulative sums).
    std::vector<int> stack = {child};
    while (!stack.empty()) {
      int u = stack.back();
      stack.pop_back();
      for (int c : children[u]) {
        cost[c] = cost[u] + edge_cost[c];
        stack.push_back(c);
      }
    }
  }

  std::vector<Point> path_xy_to(int idx) const {
    std::vector<int> chain;
    for (int node = idx; node != -1; node = parent[node]) chain.push_back(node);
    std::reverse(chain.begin(), chain.end());
    std::vector<Point> path;
    path.push_back(Point{states[chain[0]][0], states[chain[0]][1]});
    for (size_t i = 1; i < chain.size(); ++i) {
      const auto& seg = incoming[chain[i]];
      path.insert(path.end(), seg.begin(), seg.end());
    }
    return path;
  }
};

// Optimal edge x0->x1: {edge_cost, dense trajectory} if collision-free, else nullopt.
std::optional<std::pair<double, std::vector<Point>>> connect(SamplingSpace<Point>& space,
                                                             const State4& x0, const State4& x1,
                                                             double r) {
  auto [c, tau] = optimal_cost(x0, x1, r);
  if (!std::isfinite(c) || tau <= kEps) return std::nullopt;
  std::vector<Point> traj = trajectory_xy(x0, x1, tau);
  Point prev{x0[0], x0[1]};
  for (const Point& pt : traj) {
    if (!space.is_motion_valid(prev, pt)) return std::nullopt;
    prev = pt;
  }
  return std::make_pair(c, std::move(traj));
}

std::vector<std::pair<double, int>> sorted_by_distance(const KinoTree& tree, const State4& target) {
  std::vector<std::pair<double, int>> d;
  d.reserve(static_cast<size_t>(tree.size()));
  for (int i = 0; i < tree.size(); ++i) {
    d.emplace_back(std::hypot(tree.states[i][0] - target[0], tree.states[i][1] - target[1]), i);
  }
  return d;
}

// Euclidean prefilter to the K closest, then pick the exact optimal-cost minimiser.
int nearest(const KinoTree& tree, const State4& target, double r) {
  auto d = sorted_by_distance(tree, target);
  int k = std::min(kNearestCandidates, static_cast<int>(d.size()));
  std::nth_element(d.begin(), d.begin() + (k - 1), d.end());
  int best_idx = -1;
  double best_cost = std::numeric_limits<double>::infinity();
  for (int i = 0; i < k; ++i) {
    double c = optimal_cost(tree.states[d[i].second], target, r).first;
    if (c < best_cost) {
      best_cost = c;
      best_idx = d[i].second;
    }
  }
  return best_idx;
}

std::vector<int> neighborhood(const KinoTree& tree, const State4& target, double radius) {
  std::vector<std::pair<double, int>> within;
  for (int i = 0; i < tree.size(); ++i) {
    double dist = std::hypot(tree.states[i][0] - target[0], tree.states[i][1] - target[1]);
    if (dist <= radius) within.emplace_back(dist, i);
  }
  if (static_cast<int>(within.size()) > kMaxNeighbors) {
    std::nth_element(within.begin(), within.begin() + kMaxNeighbors, within.end());
    within.resize(static_cast<size_t>(kMaxNeighbors));
  }
  std::vector<int> out;
  out.reserve(within.size());
  for (const auto& p : within) out.push_back(p.second);
  return out;
}

}  // namespace

core::PlanResult<Point> KinodynamicRrtStarPlanner::plan(SamplingSpace<Point>& space,
                                                        const Point& start, const Point& goal,
                                                        TraceRecorder* recorder) {
  const int max_iterations = params_.get_int("max_iterations");
  const double goal_bias = params_.get_float("goal_bias");
  const double goal_tol = params_.get_float("goal_tolerance");
  const double neighbor_radius = params_.get_float("neighbor_radius");
  const double r = params_.get_float("control_weight");
  const double vmax = params_.get_float("max_velocity");
  std::mt19937 rng(static_cast<unsigned>(params_.get_int("seed")));
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> vel(-vmax, vmax);

  auto t0 = std::chrono::steady_clock::now();
  // Fixed final state: start and goal are lifted to rest (zero velocity).
  const State4 x_start{start.x, start.y, 0.0, 0.0};
  const State4 x_goal{goal.x, goal.y, 0.0, 0.0};
  KinoTree tree(x_start);
  // Goal is not a growth/rewire node (Karaman & Frazzoli 2011); track its best parent
  // + incoming trajectory only.
  int best_goal_parent = -1;
  double best_goal_cost = std::numeric_limits<double>::infinity();
  std::vector<Point> best_goal_traj;
  int iterations = 0;

  for (int it = 0; it < max_iterations; ++it) {
    ++iterations;
    // Goal-biasing draws the goal rest-state directly; else a free position with a
    // random velocity in [-vmax, vmax]^2 (a full 4D sample).
    State4 q_rand;
    if (unit(rng) < goal_bias) {
      q_rand = x_goal;
    } else {
      Point p = space.sample();
      q_rand = State4{p.x, p.y, vel(rng), vel(rng)};
    }
    if (recorder) recorder->sample_drawn(Point{q_rand[0], q_rand[1]});

    int near_idx = nearest(tree, q_rand, r);
    // Optimal steering reaches the sampled state exactly (controllable linear system).
    const State4& x_new = q_rand;
    std::vector<int> nbrs = neighborhood(tree, x_new, neighbor_radius);

    // Choose-parent: attach x_new to the min-cost feasible parent.
    std::set<int> candidates(nbrs.begin(), nbrs.end());
    if (near_idx >= 0) candidates.insert(near_idx);
    int best_parent = -1;
    double best_total = std::numeric_limits<double>::infinity();
    std::vector<Point> best_edge;
    for (int j : candidates) {
      auto conn = connect(space, tree.states[j], x_new, r);
      if (!conn) continue;
      double total = tree.cost[j] + conn->first;
      if (recorder) recorder->candidate_evaluated(Point{x_new[0], x_new[1]}, total);
      if (total < best_total) {
        best_total = total;
        best_parent = j;
        best_edge = conn->second;
      }
    }
    if (best_parent < 0) continue;
    double edge_cost = best_total - tree.cost[best_parent];
    int new_idx = tree.add(x_new, best_parent, edge_cost, best_edge);
    if (recorder) {
      // Emit the curved edge as a chain of chords so the trajectory renders.
      Point prev{tree.states[best_parent][0], tree.states[best_parent][1]};
      for (const Point& pt : best_edge) {
        recorder->edge_added(pt, prev);
        prev = pt;
      }
    }

    // Rewire: reroute neighbours through x_new when cheaper and feasible.
    for (int j : nbrs) {
      if (j == tree.parent[new_idx] || j == new_idx) continue;
      auto conn = connect(space, x_new, tree.states[j], r);
      if (!conn) continue;
      if (tree.cost[new_idx] + conn->first < tree.cost[j]) {
        tree.reparent(j, new_idx, conn->first, conn->second);
        if (recorder) {
          recorder->rewire(Point{tree.states[j][0], tree.states[j][1]},
                           Point{x_new[0], x_new[1]});
        }
      }
    }

    // Goal connection: a goal-biased sample lands x_new on the goal (self-connection is
    // degenerate, tau*=0), so the node itself is the arrival; otherwise fly the optimal
    // edge to the goal rest-state and require it collision-free.
    if (std::hypot(x_new[0] - goal.x, x_new[1] - goal.y) <= goal_tol) {
      double cand = std::numeric_limits<double>::infinity();
      std::vector<Point> traj;
      if (std::abs(x_new[0] - x_goal[0]) <= kEps && std::abs(x_new[1] - x_goal[1]) <= kEps &&
          std::abs(x_new[2]) <= kEps && std::abs(x_new[3]) <= kEps) {
        cand = tree.cost[new_idx];
      } else {
        auto conn = connect(space, x_new, x_goal, r);
        if (conn) {
          cand = tree.cost[new_idx] + conn->first;
          traj = conn->second;
        }
      }
      if (cand < best_goal_cost) {
        best_goal_cost = cand;
        best_goal_parent = new_idx;
        best_goal_traj = traj;
        if (recorder) {
          std::vector<Point> path = tree.path_xy_to(new_idx);
          path.insert(path.end(), traj.begin(), traj.end());
          recorder->path_found(path);
        }
      }
    }
  }

  core::PlanResult<Point> result;
  if (best_goal_parent >= 0) {
    result.success = true;
    result.path = tree.path_xy_to(best_goal_parent);
    result.path.insert(result.path.end(), best_goal_traj.begin(), best_goal_traj.end());
    result.cost = best_goal_cost;
  }
  int tree_size = tree.size();
  result.stats = {tree_size - 1, iterations, iterations, tree_size};
  double rt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  if (recorder) {
    recorder->planning_finished(result.success, {{"runtime_sec", rt},
                                                 {"path_cost", result.cost},
                                                 {"expanded_nodes", static_cast<double>(tree_size - 1)},
                                                 {"samples", static_cast<double>(iterations)},
                                                 {"tree_size", static_cast<double>(tree_size)},
                                                 {"iterations", static_cast<double>(iterations)}});
  }
  return result;
}

}  // namespace navigation::global_planning
