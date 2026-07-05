#include "navigation/global_planning/search/dstar_lite.hpp"

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

// Priority key [k1, k2] (Koenig & Likhachev 2002). std::pair gives the lexicographic
// order the paper's CalcKey requires.
using Key = std::pair<double, double>;

// Octile distance on integer cell deltas, evaluated in exactly the same operation
// order — (hi - lo) + sqrt(2)*lo, sqrt NOT hypot — as OccupancyGrid2D::heuristic, so
// keys are bit-identical to the Python mirror and the two traces the bench compares
// stay in lock-step. Admissible for 8-connected unit/sqrt2 moves.
double octile(const Cell& a, const Cell& b) {
  int dr = std::abs(a.row - b.row);
  int dc = std::abs(a.col - b.col);
  int lo = std::min(dr, dc);
  int hi = std::max(dr, dc);
  return static_cast<double>(hi - lo) + std::sqrt(2.0) * static_cast<double>(lo);
}

// The whole move -> sense -> repair simulation. Holds the g/rhs values, the belief
// (blocked set), and the k_m offset across replans; plan() drives run() once.
struct DStarLiteSearch {
  DynamicGridSpace<Cell>& space;
  TraceRecorder* recorder;
  Cell start;
  Cell goal;
  int sensor_radius;

  Cell s_start{};  // set to `start` in run(); default here silences aggregate-init warnings
  Cell s_last{};
  double k_m = 0.0;
  std::set<Cell> blocked{};              // belief: known blocked cells (empty = freespace)
  std::unordered_map<Cell, double> g{};
  std::unordered_map<Cell, double> rhs{};
  std::unordered_map<Cell, Key> key_of{};  // current key of each vertex in the queue
  std::priority_queue<std::tuple<Key, unsigned long long, Cell>,
                      std::vector<std::tuple<Key, unsigned long long, Cell>>, std::greater<>>
      open{};
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
    double m = std::min(get_g(s), get_rhs(s));
    return {m + octile(s_start, s) + k_m, m};
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

  void update_vertex(const Cell& u) {
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
      double old = get_rhs(u);
      if (best != old) {  // a real relaxation of the cost-to-goal estimate
        rhs[u] = best;
        if (recorder && has && best < kInf) {
          recorder->candidate_evaluated(u, best);
          recorder->edge_added(u, sbest, best_edge);  // sbest = successor toward goal
        }
      }
    }
    key_of.erase(u);  // remove u from the queue; reinsert below iff inconsistent
    if (get_g(u) != get_rhs(u)) queue_insert(u, calc_key(u));
  }

  void compute_shortest_path() {
    Key ktop;
    Cell u;
    while (peek_top(ktop, u)) {
      Key kstart = calc_key(s_start);
      if (!(ktop < kstart || get_rhs(s_start) != get_g(s_start))) break;
      key_of.erase(u);  // pop u (its heap entry is now stale)
      ++stats.expanded_nodes;
      if (recorder) recorder->node_expanded(u, std::min(get_g(u), get_rhs(u)));
      Key knew = calc_key(u);
      if (ktop < knew) {
        queue_insert(u, knew);  // stale key: reinsert with the up-to-date one
      } else if (get_g(u) > get_rhs(u)) {
        g[u] = get_rhs(u);  // over-consistent: accept the improvement, relax predecessors
        for (const auto& sc : space.passable_neighbors(u, blocked)) update_vertex(sc.first);
      } else {
        g[u] = kInf;  // under-consistent: raise, then re-evaluate u and its predecessors
        update_vertex(u);
        for (const auto& sc : space.passable_neighbors(u, blocked)) update_vertex(sc.first);
      }
    }
  }

  // In-bounds predecessors of c (cells that currently plan to move INTO c), or
  // nullopt when c is out of bounds so revealing it changes nothing. The grid is
  // undirected, so for an in-bounds c its forward passable neighbours are exactly its
  // predecessors; an out-of-bounds c still has forward neighbours (its in-bounds free
  // cells) but none of them list c back — the symmetry check is how we drop it without
  // an in-bounds accessor on the capability.
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
  // newly blocked in-bounds cells into the belief, and return the vertices whose
  // rhs must be repaired. Deterministic scan order (dr outer, dc inner).
  // The immediate 8-neighbourhood is ALWAYS sensed even when radius=1 (whose Euclidean
  // disk omits the diagonals): the robot may step diagonally next, so every cell it can
  // move into must be detectable or it could walk into a real obstacle believed free.
  std::vector<Cell> sense(const Cell& robot) {
    std::vector<Cell> to_update;
    int r = sensor_radius;
    for (int dr = -r; dr <= r; ++dr) {
      for (int dc = -r; dc <= r; ++dc) {
        // outside the disk AND outside the 1-ring (|dr|<=1 && |dc|<=1) -> skip.
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

  PlanResult<Cell> run() {
    auto t0 = std::chrono::steady_clock::now();
    s_start = start;
    s_last = start;
    rhs[goal] = 0.0;  // Initialize(): goal is the backward-search root
    queue_insert(goal, calc_key(goal));

    // Fold obstacles visible from the spawn cell into the initial belief before the
    // first plan (setup — not a replan).
    for (const Cell& v : sense(s_start)) update_vertex(v);
    compute_shortest_path();

    std::vector<Cell> trajectory{s_start};
    double realized_cost = 0.0;
    if (recorder) recorder->robot_moved(s_start);

    bool reached = (s_start == goal);
    while (!reached) {
      if (get_g(s_start) == kInf) break;  // goal unreachable under the current belief
      // Greedy step: argmin over successors of edge + g (deterministic first-min tie-break).
      double best = kInf;
      Cell next{};
      double step_cost = 0.0;
      bool has = false;
      for (const auto& sc : space.passable_neighbors(s_start, blocked)) {
        double v = sc.second + get_g(sc.first);
        if (v < best) {
          best = v;
          next = sc.first;
          step_cost = sc.second;
          has = true;
        }
      }
      if (!has || best == kInf) break;  // boxed in

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
        for (const Cell& v : changed) update_vertex(v);
        compute_shortest_path();
        ++stats.iterations;  // replan count
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
                           {"sensed_cells", static_cast<double>(sensed_cells)}});
    }
    return result;
  }
};

}  // namespace

PlanResult<Cell> DStarLitePlanner::plan(DynamicGridSpace<Cell>& space, const Cell& start,
                                        const Cell& goal, TraceRecorder* recorder) {
  DStarLiteSearch search{space, recorder, start, goal, params_.get_int("sensor_radius")};
  return search.run();
}

}  // namespace navigation::global_planning
