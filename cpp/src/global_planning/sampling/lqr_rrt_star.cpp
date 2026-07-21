#include "navigation/global_planning/sampling/lqr_rrt_star.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <random>
#include <stdexcept>
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
// Euclidean position prefilter to the K closest before the exact LQR-metric compare
// in nearest-neighbour (k-nearest RRT* variant, Karaman & Frazzoli 2011).
constexpr int kNearestCandidates = 16;
// Cap the choose-parent / rewire neighbourhood so per-iteration work stays bounded.
constexpr int kMaxNeighbors = 16;
// LQR steering horizon: a rest->rest regulator converges asymptotically, so cap the
// integration steps; blocked/non-converged rolls are rejected.
constexpr int kSteerMaxSteps = 200;
// A rest waypoint is "reached" when position and velocity errors fall under these.
constexpr double kReachPosTol = 0.05;
constexpr double kReachVelTol = 0.05;
// DARE fixed-point iteration budget + convergence tolerance (2x2 problem).
constexpr int kDareMaxIters = 1000;
constexpr double kDareTol = 1e-12;

using Mat2 = std::array<std::array<double, 2>, 2>;
using Vec2 = std::array<double, 2>;

// Per-axis 2x2 discrete-LQR of the double integrator via the Riccati (DARE)
// recursion (Perez et al. 2012). Solves for S (steady-state cost-to-go, symmetric
// 2x2) and K (1x2 optimal gain); both axes share these because the LTI dynamics and
// diagonal Q decouple identically per axis. Discretised over dt:
//   A = [[1, dt], [0, 1]], B = [[dt^2/2], [dt]], Q = diag(q_pos, q_vel), R = r_ctrl.
// Iterating P <- Q + A^T P A - A^T P B (R + B^T P B)^-1 B^T P A from P0 = Q converges
// to the unique stabilising solution (double integrator controllable, Q positive).
struct Lqr {
  Mat2 s;
  Vec2 k;
};

Lqr solve_dlqr(double q_pos, double q_vel, double r_ctrl, double dt) {
  const Mat2 a = {{{1.0, dt}, {0.0, 1.0}}};
  const Vec2 b = {0.5 * dt * dt, dt};
  const Mat2 q = {{{q_pos, 0.0}, {0.0, q_vel}}};
  Mat2 p = q;
  Vec2 gain = {0.0, 0.0};
  for (int iter = 0; iter < kDareMaxIters; ++iter) {
    // B^T P (1x2), symmetric P so p[1][0] == p[0][1].
    const Vec2 bt_p = {b[0] * p[0][0] + b[1] * p[1][0], b[0] * p[0][1] + b[1] * p[1][1]};
    const double bt_p_b = bt_p[0] * b[0] + bt_p[1] * b[1];
    const Vec2 bt_p_a = {bt_p[0] * a[0][0] + bt_p[1] * a[1][0],
                         bt_p[0] * a[0][1] + bt_p[1] * a[1][1]};
    const double denom = r_ctrl + bt_p_b;
    gain = {bt_p_a[0] / denom, bt_p_a[1] / denom};
    // A^T P (2x2).
    const Mat2 at_p = {{{a[0][0] * p[0][0] + a[1][0] * p[1][0],
                         a[0][0] * p[0][1] + a[1][0] * p[1][1]},
                        {a[0][1] * p[0][0] + a[1][1] * p[1][0],
                         a[0][1] * p[0][1] + a[1][1] * p[1][1]}}};
    // A^T P A (2x2).
    const Mat2 at_p_a = {{{at_p[0][0] * a[0][0] + at_p[0][1] * a[1][0],
                           at_p[0][0] * a[0][1] + at_p[0][1] * a[1][1]},
                          {at_p[1][0] * a[0][0] + at_p[1][1] * a[1][0],
                           at_p[1][0] * a[0][1] + at_p[1][1] * a[1][1]}}};
    // A^T P B (2x1).
    const Vec2 at_p_b = {at_p[0][0] * b[0] + at_p[0][1] * b[1],
                         at_p[1][0] * b[0] + at_p[1][1] * b[1]};
    Mat2 p_next = {{{q[0][0] + at_p_a[0][0] - at_p_b[0] * gain[0],
                     q[0][1] + at_p_a[0][1] - at_p_b[0] * gain[1]},
                    {at_p_a[1][0] - at_p_b[1] * gain[0],
                     q[1][1] + at_p_a[1][1] - at_p_b[1] * gain[1]}}};
    double delta = 0.0;
    for (int i = 0; i < 2; ++i)
      for (int j = 0; j < 2; ++j) delta = std::max(delta, std::abs(p_next[i][j] - p[i][j]));
    p = p_next;
    if (delta < kDareTol) break;
  }
  return {p, gain};
}

// LQR distance metric dist(a,b) = (a-b)^T S (a-b) summed over the two decoupled axes
// (Perez et al. 2012). Zero iff a==b, else > 0.
double lqr_cost_to_go(const State4& a, const State4& b, const Mat2& s) {
  const double dpx = a[0] - b[0], dpy = a[1] - b[1];
  const double dvx = a[2] - b[2], dvy = a[3] - b[3];
  return s[0][0] * (dpx * dpx + dpy * dpy) + 2.0 * s[0][1] * (dpx * dvx + dpy * dvy) +
         s[1][1] * (dvx * dvx + dvy * dvy);
}

// Search tree over double-integrator rest states (parallel arrays), mirroring the
// RRT* tree but keyed on LQR cost. Each node stores its incoming edge cost + the
// dense (x, y) trajectory of the LQR roll so a rewire propagates cumulative cost
// through a subtree without re-steering, and path reconstruction emits the true
// curved trajectory. (Duplicated here rather than shared with kinodynamic RRT*: the
// architecture forbids one algorithm module importing another.)
struct LqrTree {
  std::vector<State4> states;
  std::vector<int> parent;
  std::vector<double> cost;
  std::vector<double> edge_cost;
  std::vector<std::vector<Point>> incoming;
  std::vector<std::vector<int>> children;

  explicit LqrTree(const State4& root) {
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

// Rest waypoint at most step_size from x_from toward the sample position (RRT* eta
// cap). Velocity 0 so the LQR feedback regulates to it exactly (a rest equilibrium).
State4 rest_target(const State4& x_from, const State4& sample, double step_size) {
  const double dx = sample[0] - x_from[0], dy = sample[1] - x_from[1];
  const double dist = std::hypot(dx, dy);
  if (dist <= step_size || dist < kEps) return State4{sample[0], sample[1], 0.0, 0.0};
  const double scale = step_size / dist;
  return State4{x_from[0] + dx * scale, x_from[1] + dy * scale, 0.0, 0.0};
}

struct LqrParams {
  Vec2 k;
  double dt, u_max, q_pos, q_vel, r_ctrl;
};

// Integrate the LQR feedback u = -K(x - target), clamped, from x_from until it
// reaches target (rest) or the horizon. Returns {edge_cost, dense (x, y) trajectory}
// if it reaches target collision-free, else nullopt. edge_cost is the realised LQR
// cost sum(x^T Q x + u^T R u) dt of the roll — the true edge cost (Perez et al. 2012).
std::optional<std::pair<double, std::vector<Point>>> roll(SamplingSpace<Point>& space,
                                                          core::SE2CollisionSpace<core::Pose>& se2,
                                                          const core::Footprint& footprint,
                                                          const State4& x_from,
                                                          const State4& target,
                                                          const LqrParams& lp) {
  const double k0 = lp.k[0], k1 = lp.k[1], dt = lp.dt, half_dt2 = 0.5 * dt * dt;
  double px = x_from[0], py = x_from[1], vx = x_from[2], vy = x_from[3];
  const double tx = target[0], ty = target[1], tvx = target[2], tvy = target[3];
  double cost = 0.0;
  std::vector<Point> traj;
  Point prev{px, py};
  for (int step = 0; step < kSteerMaxSteps; ++step) {
    const double ex_p = px - tx, ex_v = vx - tvx;
    const double ey_p = py - ty, ey_v = vy - tvy;
    double ux = -(k0 * ex_p + k1 * ex_v);
    double uy = -(k0 * ey_p + k1 * ey_v);
    ux = std::clamp(ux, -lp.u_max, lp.u_max);
    uy = std::clamp(uy, -lp.u_max, lp.u_max);
    cost += (lp.q_pos * (ex_p * ex_p + ey_p * ey_p) + lp.q_vel * (ex_v * ex_v + ey_v * ey_v) +
             lp.r_ctrl * (ux * ux + uy * uy)) *
            dt;
    px = px + dt * vx + half_dt2 * ux;
    py = py + dt * vy + half_dt2 * uy;
    vx = vx + dt * ux;
    vy = vy + dt * uy;
    Point cur{px, py};
    // disc 는 방향 불변이라 theta 는 형식상 0. 적분 스텝(<= v_max*dt ~ 0.3 m)과 반경이
    // 같은 자릿수라 disc 사슬이 몸체 여유를 근사하고, 점 수준은 supercover 가 마저 막는다.
    if (se2.is_collision(footprint, core::Pose{px, py, 0.0})) return std::nullopt;
    if (!space.is_motion_valid(prev, cur)) return std::nullopt;
    traj.push_back(cur);
    prev = cur;
    if (std::abs(px - tx) <= kReachPosTol && std::abs(py - ty) <= kReachPosTol &&
        std::abs(vx - tvx) <= kReachVelTol && std::abs(vy - tvy) <= kReachVelTol) {
      // Snap the final waypoint onto the target so node joins are exact.
      traj.back() = Point{tx, ty};
      return std::make_pair(cost, std::move(traj));
    }
  }
  return std::nullopt;
}

std::vector<std::pair<double, int>> sorted_by_distance(const LqrTree& tree, const State4& target) {
  std::vector<std::pair<double, int>> d;
  d.reserve(static_cast<size_t>(tree.size()));
  for (int i = 0; i < tree.size(); ++i) {
    d.emplace_back(std::hypot(tree.states[i][0] - target[0], tree.states[i][1] - target[1]), i);
  }
  return d;
}

// Euclidean prefilter to the K closest, then pick the exact LQR cost-to-go minimiser.
int nearest(const LqrTree& tree, const State4& target, const Mat2& s) {
  auto d = sorted_by_distance(tree, target);
  int k = std::min(kNearestCandidates, static_cast<int>(d.size()));
  std::nth_element(d.begin(), d.begin() + (k - 1), d.end());
  int best_idx = -1;
  double best_cost = std::numeric_limits<double>::infinity();
  for (int i = 0; i < k; ++i) {
    double c = lqr_cost_to_go(tree.states[d[i].second], target, s);
    if (c < best_cost) {
      best_cost = c;
      best_idx = d[i].second;
    }
  }
  return best_idx;
}

std::vector<int> neighborhood(const LqrTree& tree, const State4& target, double radius) {
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

core::PlanResult<Point> LqrRrtStarPlanner::plan(SamplingSpace<Point>& space, const Point& start,
                                                const Point& goal, TraceRecorder* recorder) {
  const int max_iterations = params_.get_int("max_iterations");
  const double step_size = params_.get_float("step_size");
  const double goal_bias = params_.get_float("goal_bias");
  const double goal_tol = params_.get_float("goal_tolerance");
  const double neighbor_radius = params_.get_float("neighbor_radius");
  const double q_pos = params_.get_float("q_pos");
  const double q_vel = params_.get_float("q_vel");
  const double r_ctrl = params_.get_float("r_ctrl");
  const double dt = params_.get_float("lqr_dt");
  const double u_max = params_.get_float("control_limit");
  const double vmax = params_.get_float("max_velocity");
  // 차체 disc — required_capabilities 가 SE(2) view 를 선언하므로 로드 단계에서
  // 걸러지고, 같은 concrete grid 가 두 view 를 함께 구현해 cross-cast 로 얻는다.
  const core::Footprint footprint{params_.get_float("footprint_radius")};
  auto* se2 = dynamic_cast<core::SE2CollisionSpace<core::Pose>*>(&space);
  if (se2 == nullptr) {
    throw std::invalid_argument("lqr_rrt_star: map does not provide SE2CollisionSpace");
  }
  std::mt19937 rng(static_cast<unsigned>(params_.get_int("seed")));
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> vel(-vmax, vmax);

  // Riccati solve once: S/K are state-independent for this LTI system.
  const Lqr lqr = solve_dlqr(q_pos, q_vel, r_ctrl, dt);
  const LqrParams lp{lqr.k, dt, u_max, q_pos, q_vel, r_ctrl};

  auto t0 = std::chrono::steady_clock::now();
  // Start and goal are lifted to rest (zero velocity) — LQR equilibria.
  const State4 x_start{start.x, start.y, 0.0, 0.0};
  const State4 x_goal{goal.x, goal.y, 0.0, 0.0};
  LqrTree tree(x_start);
  // Goal is not a growth/rewire node (Karaman & Frazzoli 2011); track its best parent
  // + incoming trajectory only.
  int best_goal_parent = -1;
  double best_goal_cost = std::numeric_limits<double>::infinity();
  std::vector<Point> best_goal_traj;
  int iterations = 0;

  for (int it = 0; it < max_iterations; ++it) {
    ++iterations;
    // Goal-biasing draws the goal rest-state directly; else a free position with a
    // random velocity so the LQR nearest metric is full-state.
    State4 q_rand;
    if (unit(rng) < goal_bias) {
      q_rand = x_goal;
    } else {
      Point p = space.sample();
      q_rand = State4{p.x, p.y, vel(rng), vel(rng)};
    }
    if (recorder) recorder->sample_drawn(Point{q_rand[0], q_rand[1]});

    int near_idx = nearest(tree, q_rand, lqr.s);
    // Extend: steer x_near to a rest waypoint at most step_size toward q_rand.
    State4 x_new = rest_target(tree.states[near_idx], q_rand, step_size);
    auto near_roll = roll(space, *se2, footprint, tree.states[near_idx], x_new, lp);
    if (!near_roll) continue;
    std::vector<int> nbrs = neighborhood(tree, x_new, neighbor_radius);

    // Choose-parent: near_idx with its already-rolled edge is the default; neighbours
    // improve only if their LQR roll also reaches x_new collision-free (min cost).
    int best_parent = near_idx;
    std::vector<Point> best_edge = near_roll->second;
    double best_total = tree.cost[near_idx] + near_roll->first;
    for (int j : nbrs) {
      if (j == near_idx) continue;
      auto conn = roll(space, *se2, footprint, tree.states[j], x_new, lp);
      if (!conn) continue;
      double total = tree.cost[j] + conn->first;
      if (recorder) recorder->candidate_evaluated(Point{x_new[0], x_new[1]}, total);
      if (total < best_total) {
        best_total = total;
        best_parent = j;
        best_edge = conn->second;
      }
    }
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

    // Rewire: reroute neighbours through x_new when the LQR roll reaches them cheaper.
    for (int j : nbrs) {
      if (j == tree.parent[new_idx] || j == new_idx) continue;
      auto conn = roll(space, *se2, footprint, x_new, tree.states[j], lp);
      if (!conn) continue;
      if (tree.cost[new_idx] + conn->first < tree.cost[j]) {
        tree.reparent(j, new_idx, conn->first, conn->second);
        if (recorder) {
          recorder->rewire(Point{tree.states[j][0], tree.states[j][1]},
                           Point{x_new[0], x_new[1]});
        }
      }
    }

    // Goal connection: a goal-biased sample lands x_new on the goal (already at rest),
    // whose self-roll is degenerate, so the node itself is the arrival; otherwise roll
    // the LQR edge to the goal rest-state and require it collision-free.
    if (std::hypot(x_new[0] - goal.x, x_new[1] - goal.y) <= goal_tol) {
      double cand = std::numeric_limits<double>::infinity();
      std::vector<Point> traj;
      if (std::abs(x_new[0] - x_goal[0]) <= kEps && std::abs(x_new[1] - x_goal[1]) <= kEps &&
          std::abs(x_new[2]) <= kEps && std::abs(x_new[3]) <= kEps) {
        cand = tree.cost[new_idx];
      } else {
        auto conn = roll(space, *se2, footprint, x_new, x_goal, lp);
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
    recorder->planning_finished(result.success,
                                {{"runtime_sec", rt},
                                 {"path_cost", result.cost},
                                 {"expanded_nodes", static_cast<double>(tree_size - 1)},
                                 {"samples", static_cast<double>(iterations)},
                                 {"tree_size", static_cast<double>(tree_size)},
                                 {"iterations", static_cast<double>(iterations)}});
  }
  return result;
}

}  // namespace navigation::global_planning
