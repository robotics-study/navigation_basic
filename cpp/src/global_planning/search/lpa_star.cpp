#include "navigation/global_planning/search/lpa_star.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace navigation::global_planning {

using core::Cell;
using core::DynamicGridSpace;
using core::PlanResult;
using core::PlanStats;
using core::TraceRecorder;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// Octile distance on integer cell deltas, evaluated in exactly the same operation
// order — (hi - lo) + sqrt(2)*lo, sqrt NOT hypot — as OccupancyGrid2D::heuristic and
// the Python mirror, so keys are bit-identical across languages. Admissible for
// 8-connected unit/sqrt2 moves; consistent (a metric), which is what bounds a key
// below by g(goal) at termination.
double octile(const Cell& a, const Cell& b) {
  int dr = std::abs(a.row - b.row);
  int dc = std::abs(a.col - b.col);
  int lo = std::min(dr, dc);
  int hi = std::max(dr, dc);
  return static_cast<double>(hi - lo) + std::sqrt(2.0) * static_cast<double>(lo);
}

}  // namespace

double LpaStarPlanner::get_g(const Cell& c) const {
  auto it = g_.find(c);
  return it == g_.end() ? kInf : it->second;
}

double LpaStarPlanner::get_rhs(const Cell& c) const {
  auto it = rhs_.find(c);
  return it == rhs_.end() ? kInf : it->second;
}

// CalculateKey(u) = [min(g,rhs) + h(goal,u); min(g,rhs)]. No k_m term: the
// heuristic's reference (the goal) never moves, so keys stay comparable across
// replans — the exact contrast with D* Lite's moving-root key.
LpaStarPlanner::Key LpaStarPlanner::calc_key(const Cell& s) const {
  double m = std::min(get_g(s), get_rhs(s));
  return {m + octile(s, goal_), m};
}

void LpaStarPlanner::queue_insert(const Cell& u, const Key& key) {
  key_of_[u] = key;
  open_.push({key, seq_++, u});
}

// Drop stale entries (whose stored key no longer matches key_of — a vertex removed
// from U when it became consistent or blocked, or re-inserted under a new key) and
// report the smallest live one.
bool LpaStarPlanner::peek_top(Key& out_key, Cell& out_u) {
  while (!open_.empty()) {
    const auto& [k, seq, u] = open_.top();
    (void)seq;
    auto it = key_of_.find(u);
    if (it == key_of_.end() || it->second != k) {
      open_.pop();
      continue;
    }
    out_key = k;
    out_u = u;
    return true;
  }
  return false;
}

// UpdateVertex(u): rhs = min over predecessors (the grid is undirected, so the
// passable neighbours of u ARE its predecessors — same symmetry argument as D*
// Lite's belief search). The start keeps rhs=0 forever; a blocked cell never
// re-enters U while blocked (apply_revision pops it on the flip).
void LpaStarPlanner::update_vertex(const Cell& u) {
  if (!(u == start_)) {
    double best = kInf;
    Cell sbest{};
    double best_edge = 0.0;
    bool has = false;
    for (const auto& sc : space_->passable_neighbors(u, blocked_)) {
      double v = sc.second + get_g(sc.first);
      if (v < best) {
        best = v;
        sbest = sc.first;
        best_edge = sc.second;
        has = true;
      }
    }
    double old = get_rhs(u);
    if (best != old) {  // a real relaxation of the start-distance estimate
      rhs_[u] = best;
      if (recorder_ && has && best < kInf) {
        recorder_->candidate_evaluated(u, best);
        recorder_->edge_added(u, sbest, best_edge);  // sbest = predecessor toward start
      }
    }
  }
  key_of_.erase(u);  // remove u from the queue; reinsert below iff inconsistent
  if (get_g(u) != get_rhs(u)) queue_insert(u, calc_key(u));
}

// ComputeShortestPath(): expand in key order until the goal is locally consistent
// AND no live key undercuts the goal's.
void LpaStarPlanner::compute_shortest_path() {
  Key ktop;
  Cell u;
  while (peek_top(ktop, u)) {
    if (!(ktop < calc_key(goal_) || get_rhs(goal_) != get_g(goal_))) break;
    key_of_.erase(u);  // pop u (its heap entry is now stale)
    ++expanded_;
    if (recorder_) recorder_->node_expanded(u, std::min(get_g(u), get_rhs(u)));
    Key knew = calc_key(u);
    if (ktop < knew) {
      // Key-invariant (no k_m here): a popped key can only be stale from the pop
      // itself. Re-inserting it is therefore dead code by proof — kept verbatim
      // from the paper's pseudocode, and provably inert.
      queue_insert(u, knew);
    } else if (get_g(u) > get_rhs(u)) {
      g_[u] = get_rhs(u);  // over-consistent: accept it, relax successors
      for (const auto& sc : space_->passable_neighbors(u, blocked_)) update_vertex(sc.first);
    } else {
      g_[u] = kInf;  // under-consistent: raise, re-evaluate u and successors
      update_vertex(u);
      for (const auto& sc : space_->passable_neighbors(u, blocked_)) update_vertex(sc.first);
    }
  }
}

// Walk the parent chain BACKWARD from the goal and reverse it. At u, the first
// neighbour p with cost + g(p) == rhs(u) is exactly the predecessor whose
// relaxation SET rhs(u) (same operands and operation order as update_vertex — so
// the equality is bit-exact, not a tolerance). Every chain vertex is locally
// consistent when the burst ended: an inconsistent one would sit in U with key <
// key(goal), contradicting termination — so each hop satisfies c = g(u_next) -
// g(u) and the costs telescope to exactly g(goal).
std::vector<Cell> LpaStarPlanner::extract_path(double* out_cost) {
  *out_cost = 0.0;
  if (get_g(goal_) == kInf) return {};
  std::vector<Cell> path_rev{goal_};
  double cost = 0.0;
  Cell cur = goal_;
  while (!(cur == start_)) {
    double rhs_cur = get_rhs(cur);
    bool found = false;
    Cell nxt{};
    double step_cost = 0.0;
    for (const auto& sc : space_->passable_neighbors(cur, blocked_)) {
      double cand = sc.second + get_g(sc.first);
      if (cand == rhs_cur) {
        found = true;
        nxt = sc.first;
        step_cost = sc.second;
        break;
      }
    }
    if (!found) return {};  // unreachable in a correct run; fail honestly, never loop
    cost += step_cost;
    path_rev.push_back(nxt);
    cur = nxt;
  }
  std::reverse(path_rev.begin(), path_rev.end());
  *out_cost = cost;
  return path_rev;
}

core::PlanResult<core::Cell> LpaStarPlanner::plan(DynamicGridSpace<Cell>& space, const Cell& start,
                                                  const Cell& goal, TraceRecorder* recorder) {
  auto t0 = std::chrono::steady_clock::now();
  bool first_round = !seeded_;
  if (first_round) {
    // Initialize(): every g/rhs starts infinite (the maps stay empty; get_g/get_rhs
    // default to inf), the start's rhs pins to 0, and the model seeds from ground
    // truth — LPA* plans on what it KNOWS, not senses.
    space_ = &space;
    recorder_ = recorder;
    start_ = start;
    goal_ = goal;
    seeded_ = true;
    for (const Cell& c : space.occupied_cells()) blocked_.insert(c);
    rhs_[start] = 0.0;
    queue_insert(start, calc_key(start));
  } else {
    // Fixed endpoints are the algorithm: re-rooting is D* Lite's job.
    if (!(start == start_) || !(goal == goal_)) {
      throw std::invalid_argument(
          "LpaStarPlanner is fixed-start/fixed-goal; re-planning with a moved root is "
          "D* Lite, not LPA*");
    }
    if (recorder != nullptr) recorder_ = recorder;  // later rounds may carry a fresh recorder
  }

  compute_shortest_path();
  if (!first_round) ++replans_;

  double cost = 0.0;
  std::vector<Cell> path = extract_path(&cost);
  bool reached = !path.empty();
  runtime_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  PlanStats stats{};
  stats.expanded_nodes = expanded_;
  stats.iterations = replans_;
  if (recorder_) {
    if (reached) recorder_->path_found(path);
    recorder_->planning_finished(reached, {{"runtime_sec", runtime_},
                                           {"path_cost", cost},
                                           {"expanded_nodes", static_cast<double>(expanded_)},
                                           {"replan_count", static_cast<double>(replans_)}});
  }
  PlanResult<Cell> result;
  result.success = reached;
  result.path = std::move(path);
  result.cost = cost;
  result.stats = stats;
  return result;
}

// The paper's Main(): 'wait for changes in edge costs; for all directed edges (u,v)
// with changed edge costs, UpdateVertex(v)'. A cell freed by a flip gets exactly
// that — itself and its passable neighbours, the vertices whose predecessor set
// actually changed. A newly-BLOCKED cell is not updated but popped from U: a live
// entry carrying its stale finite key could otherwise pop mid-burst and churn g/rhs
// pointlessly (wasted expansions, node_expanded noise on a wall cell). Its stale
// values can never reach another cell anyway — every rhs is computed over passable
// predecessors only. No-op flips (already in that state IN THE MODEL) change no edge
// cost, so they emit nothing and update nothing.
void LpaStarPlanner::apply_revision(const std::vector<Cell>& cells, bool blocked) {
  if (!seeded_) {
    throw std::runtime_error(
        "apply_revision before the first plan(): LPA* seeds its model in plan()");
  }
  for (const Cell& c : cells) {
    // The MODEL's state before the flip decides what changed, not ground truth:
    // revisions are reversible cost changes (free -> blocked -> free on the same
    // cell is legal), and only model membership tracks that. Out-of-bounds cells
    // are never members, so freeing one is a no-op; an out-of-bounds block adds a
    // member that passable_neighbors bounds-checks away anyway — a recorded no
    // change to any real edge cost.
    bool was_blocked = blocked_.count(c) > 0;
    if (was_blocked == blocked) continue;
    if (recorder_) recorder_->obstacle_changed(c, blocked);
    if (blocked) {
      blocked_.insert(c);
      key_of_.erase(c);  // a blocked vertex must not sit in U
    } else {
      blocked_.erase(c);
      update_vertex(c);  // c became a vertex again: recompute rhs
    }
    for (const auto& sc : space_->passable_neighbors(c, blocked_)) update_vertex(sc.first);
  }
}

}  // namespace navigation::global_planning
