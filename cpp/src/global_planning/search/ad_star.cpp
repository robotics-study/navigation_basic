#include "navigation/global_planning/search/ad_star.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <set>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace navigation::global_planning {

using core::Cell;
using core::DynamicGridSpace;
using core::PlanResult;
using core::PlanStats;
using core::TraceRecorder;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// Priority key [k1, k2] (Likhachev et al. 2005). std::pair gives the lexicographic
// order CalcKey requires.
using Key = std::pair<double, double>;

// Octile distance on integer cell deltas, in exactly the same operation order —
// (hi - lo) + sqrt(2)*lo, sqrt NOT hypot — as OccupancyGrid2D::heuristic, so keys are
// bit-identical to the Python mirror. The DynamicGridSpace capability exposes no
// heuristic(), so the planner carries its own (admissible for 8-connected moves).
double octile(const Cell& a, const Cell& b) {
  int dr = std::abs(a.row - b.row);
  int dc = std::abs(a.col - b.col);
  int lo = std::min(dr, dc);
  int hi = std::max(dr, dc);
  return static_cast<double>(hi - lo) + std::sqrt(2.0) * static_cast<double>(lo);
}

// The whole improve -> move -> sense -> repair simulation. Holds g/rhs, the belief
// (blocked set), the k_m offset, and the ARA* CLOSED/INCONS bookkeeping across the run;
// plan() drives run() once.
struct AdStarSearch {
  DynamicGridSpace<Cell>& space;
  TraceRecorder* recorder;
  Cell start;
  Cell goal;
  double eps;         // current inflation
  double eps_start;   // reset target on a sensed change (max(eps_start, eps_final))
  double eps_final;
  double eps_step;
  int sensor_radius;
  long long max_expansions;

  Cell s_start{};  // set to `start` in run(); default here silences aggregate-init warnings
  Cell s_last{};
  double k_m = 0.0;
  std::set<Cell> blocked{};              // belief: known blocked cells (empty = freespace)
  std::unordered_map<Cell, double> g{};
  std::unordered_map<Cell, double> rhs{};
  std::unordered_map<Cell, Key> key_of{};  // OPEN membership + current key of each vertex
  std::priority_queue<std::tuple<Key, unsigned long long, Cell>,
                      std::vector<std::tuple<Key, unsigned long long, Cell>>, std::greater<>>
      open{};
  std::set<Cell> closed{};  // expanded (over-consistent) since the last reopen
  std::set<Cell> incons{};  // inconsistent again after expansion; reopened on ε change
  unsigned long long seq = 0;
  PlanStats stats{};
  int sensed_cells = 0;

  double get_g(const Cell& c) const {
    auto it = g.find(c);
    return it == g.end() ? kInf : it->second;
  }
  double get_rhs(const Cell& c) const {
    auto it = rhs.find(c);
    return it == rhs.end() ? kInf : it->second;
  }

  Key calc_key(const Cell& s) const {
    double gs = get_g(s);
    double rs = get_rhs(s);
    if (gs > rs) {  // over-consistent: inflate the heuristic by ε (ARA* weighting)
      return {rs + eps * octile(s_start, s) + k_m, rs};
    }
    // under-consistent / consistent: NO inflation (Likhachev et al. 2005 key), so a
    // raised cost still propagates on an admissible key.
    return {gs + octile(s_start, s) + k_m, gs};
  }

  void queue_insert(const Cell& u, const Key& key) {
    key_of[u] = key;
    open.push({key, seq++, u});
  }

  // Cleans stale heap entries (whose stored key no longer matches key_of) off the top
  // and reports the smallest live entry, if any.
  bool peek_top(Key& out_key, Cell& out_u) {
    while (!open.empty()) {
      const auto& [k, s, u] = open.top();
      auto it = key_of.find(u);
      if (it == key_of.end() || it->second != k) {
        open.pop();
        continue;
      }
      out_key = k;
      out_u = u;
      return true;
    }
    return false;
  }

  void update_state(const Cell& u) {
    if (!(u == goal)) {
      double best = kInf;
      Cell sbest{};
      double best_edge = 0.0;
      bool has = false;
      for (const auto& sc : space.passable_neighbors(u, blocked)) {
        double v = sc.second + get_g(sc.first);
        if (v < best) {
          best = v;
          sbest = sc.first;
          best_edge = sc.second;
          has = true;
        }
      }
      if (best != get_rhs(u)) {  // a real change to the cost-to-goal look-ahead
        rhs[u] = best;
        if (recorder && has && best < kInf) {
          recorder->candidate_evaluated(u, best);
          recorder->edge_added(u, sbest, best_edge);  // sbest = successor toward goal
        }
      }
    }
    // Take u out of OPEN/INCONS, then re-file it by (in)consistency + CLOSED status.
    key_of.erase(u);
    incons.erase(u);
    if (get_g(u) != get_rhs(u)) {
      if (closed.find(u) == closed.end()) {
        queue_insert(u, calc_key(u));
      } else {
        // Already expanded this pass: defer to the next reopen instead of re-expanding
        // now (the ARA* INCONS trick, carried into AD*).
        incons.insert(u);
      }
    }
  }

  void reopen() {
    // Move INCONS into OPEN and recompute every key under the current ε and k_m, then
    // clear CLOSED so improved / cost-changed states can be re-expanded.
    std::vector<Cell> states;
    for (const auto& kv : key_of) states.push_back(kv.first);
    for (const Cell& s : incons) states.push_back(s);
    // Reinsert in a fixed (row, col) order so the queue tie-break counter is assigned
    // identically to the Python mirror — hash-iteration order would diverge.
    std::sort(states.begin(), states.end());
    incons.clear();
    closed.clear();
    key_of.clear();
    open = decltype(open){};
    for (const Cell& s : states) queue_insert(s, calc_key(s));
  }

  // Expand until s_start is consistent and no OPEN key beats its key: g(s_start) is then
  // ε-suboptimal (ARA*/AD* termination). Returns false if the expansion cap tripped, so
  // the caller stops with the best trajectory so far.
  bool compute_or_improve_path() {
    Key ktop;
    Cell u;
    while (peek_top(ktop, u)) {
      Key kstart = calc_key(s_start);
      if (!(ktop < kstart || get_rhs(s_start) != get_g(s_start))) break;
      key_of.erase(u);  // pop u from OPEN
      ++stats.expanded_nodes;
      if (stats.expanded_nodes > max_expansions) return false;
      if (recorder) recorder->node_expanded(u, std::min(get_g(u), get_rhs(u)));
      if (get_g(u) > get_rhs(u)) {
        g[u] = get_rhs(u);  // over-consistent: accept it, settle into CLOSED
        closed.insert(u);
        for (const auto& sc : space.passable_neighbors(u, blocked)) update_state(sc.first);
      } else {
        g[u] = kInf;  // under-consistent: raise, re-evaluate u and predecessors
        update_state(u);
        for (const auto& sc : space.passable_neighbors(u, blocked)) update_state(sc.first);
      }
    }
    return true;
  }

  // The current plan: greedily follow argmin over successors of edge + g from s_start to
  // goal (AD* solution extraction). Valid + finite after ComputeOrImprovePath; the
  // visited guard only defends against a malformed g.
  std::optional<std::vector<Cell>> extract_plan() {
    if (get_g(s_start) == kInf) return std::nullopt;
    std::vector<Cell> path{s_start};
    std::set<Cell> seen{s_start};
    Cell cur = s_start;
    while (!(cur == goal)) {
      double best = kInf;
      Cell nxt{};
      bool has = false;
      for (const auto& sc : space.passable_neighbors(cur, blocked)) {
        double v = sc.second + get_g(sc.first);
        if (v < best) {
          best = v;
          nxt = sc.first;
          has = true;
        }
      }
      if (!has || best == kInf || seen.count(nxt)) return std::nullopt;
      cur = nxt;
      seen.insert(cur);
      path.push_back(cur);
    }
    return path;
  }

  void publish() {
    if (!recorder) return;
    auto plan = extract_plan();
    if (plan) recorder->path_found(*plan);  // anytime: bound = current ε
  }

  // In-bounds predecessors of c (cells that currently plan to move INTO c), or nullopt
  // when c is out of bounds so revealing it changes nothing. The grid is undirected, so
  // an in-bounds c's forward passable neighbours are exactly its predecessors; an
  // out-of-bounds c has forward neighbours but none list c back — the symmetry check
  // drops it without an in-bounds accessor on the capability.
  std::optional<std::vector<std::pair<Cell, double>>> predecessors_if_in_bounds(const Cell& c) {
    auto fwd = space.passable_neighbors(c, blocked);
    if (fwd.empty()) return std::nullopt;
    const Cell& pivot = fwd.front().first;
    for (const auto& sc : space.passable_neighbors(pivot, blocked)) {
      if (sc.first == c) return fwd;
    }
    return std::nullopt;
  }

  // Sense the Euclidean disk of radius `sensor_radius` (cells) around `robot`, reveal
  // newly blocked in-bounds cells into the belief, and return the vertices whose rhs
  // must be repaired. Deterministic scan order (dr outer, dc inner). The immediate
  // 8-neighbourhood is ALWAYS sensed even at radius 1 (whose disk omits diagonals): the
  // robot may step diagonally next, so every reachable cell must be detectable.
  std::vector<Cell> sense(const Cell& robot) {
    std::vector<Cell> to_update;
    int r = sensor_radius;
    for (int dr = -r; dr <= r; ++dr) {
      for (int dc = -r; dc <= r; ++dc) {
        if (dr * dr + dc * dc > r * r && !(dr * dr <= 1 && dc * dc <= 1)) continue;
        Cell c{robot.row + dr, robot.col + dc};
        if (blocked.count(c)) continue;
        if (!space.is_blocked(c)) continue;
        auto preds = predecessors_if_in_bounds(c);
        if (!preds) continue;  // out of bounds: already impassable, nothing to repair
        blocked.insert(c);
        ++sensed_cells;
        if (recorder) recorder->obstacle_revealed(c);
        for (const auto& sc : *preds) to_update.push_back(sc.first);
      }
    }
    return to_update;
  }

  // One belief-optimal move: argmin over successors of edge + g (first-min tie-break).
  bool greedy_step(Cell& out_next, double& out_cost) {
    double best = kInf;
    Cell nxt{};
    double step_cost = 0.0;
    bool has = false;
    for (const auto& sc : space.passable_neighbors(s_start, blocked)) {
      double v = sc.second + get_g(sc.first);
      if (v < best) {
        best = v;
        nxt = sc.first;
        step_cost = sc.second;
        has = true;
      }
    }
    if (!has || best == kInf) return false;
    out_next = nxt;
    out_cost = step_cost;
    return true;
  }

  PlanResult<Cell> run() {
    auto t0 = std::chrono::steady_clock::now();
    s_start = start;
    s_last = start;
    rhs[goal] = 0.0;  // goal is the backward-search root
    queue_insert(goal, calc_key(goal));

    // Fold obstacles visible from the spawn cell into the belief before the first plan
    // (setup — not a replan).
    for (const Cell& v : sense(s_start)) update_state(v);
    bool capped = !compute_or_improve_path();
    publish();  // first ε_start-suboptimal solution from start

    std::vector<Cell> trajectory{s_start};
    double realized_cost = 0.0;
    if (recorder) recorder->robot_moved(s_start);

    bool reached = (s_start == goal);
    while (!reached && !capped) {
      if (eps > eps_final) {
        // Anytime improvement: tighten ε, reopen INCONS∪OPEN, repair. No motion — the
        // robot waits for a belief-optimal plan before stepping.
        eps = std::max(eps_final, eps - eps_step);
        reopen();
        capped = !compute_or_improve_path();
        publish();
        continue;
      }

      // ε == eps_final: the plan is optimal for the current belief → take one step.
      if (get_g(s_start) == kInf) break;  // goal unreachable under the current belief
      Cell next{};
      double step_cost = 0.0;
      if (!greedy_step(next, step_cost)) break;  // boxed in
      s_start = next;
      realized_cost += step_cost;
      trajectory.push_back(s_start);
      if (recorder) recorder->robot_moved(s_start);
      if (s_start == goal) {
        reached = true;
        break;
      }

      std::vector<Cell> changed = sense(s_start);
      if (!changed.empty()) {
        k_m += octile(s_last, s_start);  // keep keys monotone as the reference point moved
        s_last = s_start;
        for (const Cell& v : changed) update_state(v);
        // A sensed cost change is treated as significant: re-inflate ε to fetch a new
        // suboptimal plan fast, then the loop repairs it back to eps_final.
        eps = eps_start;
        ++stats.iterations;  // replan count
        reopen();
        capped = !compute_or_improve_path();
        publish();
      }
    }

    double runtime = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    PlanResult<Cell> result;
    if (reached) {
      result.success = true;
      result.path = trajectory;
      result.cost = realized_cost;
      if (recorder) recorder->path_found(result.path);
    }
    result.stats = stats;
    if (recorder) {
      recorder->planning_finished(
          result.success, {{"runtime_sec", runtime},
                           {"path_cost", result.cost},
                           {"expanded_nodes", static_cast<double>(stats.expanded_nodes)},
                           {"replan_count", static_cast<double>(stats.iterations)},
                           {"sensed_cells", static_cast<double>(sensed_cells)},
                           {"final_eps", eps}});
    }
    return result;
  }
};

}  // namespace

PlanResult<Cell> AdStarPlanner::plan(DynamicGridSpace<Cell>& space, const Cell& start,
                                     const Cell& goal, TraceRecorder* recorder) {
  double eps_start = std::max(params_.get_float("eps_start"), params_.get_float("eps_final"));
  AdStarSearch search{space,
                      recorder,
                      start,
                      goal,
                      eps_start,
                      eps_start,
                      params_.get_float("eps_final"),
                      params_.get_float("eps_step"),
                      params_.get_int("sensor_radius"),
                      params_.get_int("max_expansions")};
  return search.run();
}

}  // namespace navigation::global_planning
