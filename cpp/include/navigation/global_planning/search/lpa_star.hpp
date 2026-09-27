#pragma once

#include <cstdint>
#include <queue>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// LPA* (Koenig & Likhachev 2002; journal Koenig, Likhachev & Furcy 2004), simple
// version — the forward, fixed-start twin of D* Lite: g/rhs estimate the START
// distance and the heuristic's reference (the goal) never moves, so keys stay
// comparable across replans without any k_m offset. plan() runs one burst per call;
// apply_revision() is the paper's Main() loop step — the cost change the planner is
// TOLD about (vs D* Lite sensing it). State persists ACROSS plan() calls: round 0
// seeds the model from occupied_cells() and later rounds are repair bursts. The
// Python mirror (python/.../search/lpa_star.py) is bit-identical move for move, so
// both languages emit identical traces.
class LpaStarPlanner final : public core::DynamicGridPlanner {
 public:
  explicit LpaStarPlanner(core::ParamSet params) : core::DynamicGridPlanner(std::move(params)) {}
  std::string name() const override { return "lpa_star"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DYNAMIC_GRID_SPACE};
  }
  // Round 0 seeds the model from the map's full ground truth and runs one full
  // burst; later rounds re-run ComputeShortestPath after apply_revision calls.
  // Fixed start/goal IS the algorithm (re-rooting is D* Lite): a moved endpoint
  // throws std::invalid_argument, and calling apply_revision before the first
  // plan() (no model to repair yet) throws std::runtime_error.
  core::PlanResult<core::Cell> plan(core::DynamicGridSpace<core::Cell>& space,
                                    const core::Cell& start, const core::Cell& goal,
                                    core::TraceRecorder* recorder) override;

  // The paper's Main(): flip each cell's passability in the MODEL (a flip is a
  // no-op iff the model already holds that state — ground truth never moves),
  // UpdateVertex the flipped cell and its passable neighbours, and emit one
  // obstacle_changed event per real flip.
  void apply_revision(const std::vector<core::Cell>& cells, bool blocked);

 private:
  using Key = std::pair<double, double>;

  // --- paper primitives (Figure 3, simple version), mirroring the Python file ---
  double get_g(const core::Cell& c) const;
  double get_rhs(const core::Cell& c) const;
  Key calc_key(const core::Cell& s) const;
  void queue_insert(const core::Cell& u, const Key& key);
  bool peek_top(Key& out_key, core::Cell& out_u);
  void update_vertex(const core::Cell& u);
  void compute_shortest_path();
  std::vector<core::Cell> extract_path(double* out_cost);

  // Search state persists across plan() calls — that IS the algorithm.
  core::DynamicGridSpace<core::Cell>* space_ = nullptr;  // set by the first plan()
  core::TraceRecorder* recorder_ = nullptr;
  core::Cell start_{};
  core::Cell goal_{};
  bool seeded_ = false;
  // The model: every blocked cell. Seeded from ground truth at the first plan();
  // revisions flip single cells out of/in it. Out-of-bounds is never in the set
  // and never needs to be — passable_neighbors bounds-checks anyway.
  std::set<core::Cell> blocked_{};
  std::unordered_map<core::Cell, double> g_{};    // start-distance estimate (absent = inf)
  std::unordered_map<core::Cell, double> rhs_{};  // one-step lookahead over predecessors
  std::unordered_map<core::Cell, Key> key_of_{};  // current key of each vertex in the queue
  // Min-heap via greater<> on (key, seq, cell): lexicographic key order, then the
  // monotone sequence counter as a stable tie-break (keeps entries comparable and
  // the pop order deterministic, exactly like Python's heap tuples).
  std::priority_queue<std::tuple<Key, unsigned long long, core::Cell>,
                      std::vector<std::tuple<Key, unsigned long long, core::Cell>>,
                      std::greater<>>
      open_{};
  unsigned long long seq_ = 0;
  int expanded_ = 0;   // cumulative expansions across every burst (stats.expanded_nodes)
  int replans_ = 0;    // completed re-plan rounds (stats.iterations)
  double runtime_ = 0.0;  // cumulative burst seconds across every plan() round
};

}  // namespace navigation::global_planning
