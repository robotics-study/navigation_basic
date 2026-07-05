#pragma once

#include <set>
#include <utility>
#include <vector>

namespace navigation::core {

// OBSTACLE_QUERY is declared for future local planners but implemented by no map
// here; OccupancyGrid2D must not advertise it.
enum class Capability {
  DISCRETE_SPACE,
  SAMPLING_SPACE,
  OBSTACLE_QUERY,
  LINE_OF_SIGHT_SPACE,
  DYNAMIC_GRID_SPACE
};

// Successor enumeration + admissible heuristic for graph-search planners.
template <class State>
class DiscreteSpace {
 public:
  virtual ~DiscreteSpace() = default;
  virtual std::vector<std::pair<State, double>> neighbors(const State& s) const = 0;
  virtual double heuristic(const State& a, const State& b) const = 0;
};

// Any-angle search view: a DiscreteSpace that can also test straight-segment
// visibility between two cells (Nash, Daniel, Koenig & Felner 2007). Extends
// DiscreteSpace so one interface supplies both grid successors (path 1) and the
// LOS shortcut test (path 2) — GlobalPlanner<State, Space> takes exactly one
// space. Only maps with real geometry can answer it, so it is a distinct
// capability, never a DiscreteSpace method (GraphMap/TopologyMap have none).
template <class State>
class LineOfSightSpace : public DiscreteSpace<State> {
 public:
  // True iff the straight segment between cells a and b is collision-free under
  // the same corner-cut-forbidden rule neighbors() uses. Cell in; any world
  // conversion happens inside the map, never here.
  virtual bool line_of_sight(const State& a, const State& b) const = 0;
};

// Continuous sampling + local steering for RRT-family planners.
template <class State>
class SamplingSpace {
 public:
  virtual ~SamplingSpace() = default;
  virtual State sample() = 0;
  virtual bool is_state_valid(const State& s) const = 0;
  virtual bool is_motion_valid(const State& a, const State& b) const = 0;
  virtual double distance(const State& a, const State& b) const = 0;
  virtual State steer(const State& a, const State& b, double eta) const = 0;
};

// Dynamic-replanning search view for D* Lite (Koenig & Likhachev 2002). Standalone
// (NOT a DiscreteSpace): its neighbor query takes a belief — the planner's own set
// of currently-known blocked cells — instead of reading ground truth, so it cannot
// share the truth-baked neighbors() of DiscreteSpace. Only maps with real occupancy
// + geometry can answer it (GraphMap/TopologyMap have none).
template <class State>
class DynamicGridSpace {
 public:
  virtual ~DynamicGridSpace() = default;
  // In-bounds 8-connected moves that are traversable under the BELIEF `blocked`
  // (not ground truth): a cell counts free iff it is in bounds and not in `blocked`,
  // using the same corner-cut-forbidden rule as neighbors(). The map owns the move
  // table + corner rule so the planner never reimplements grid geometry.
  virtual std::vector<std::pair<State, double>> passable_neighbors(
      const State& s, const std::set<State>& blocked) const = 0;
  // Ground-truth sensor: true iff `s` is occupied OR out of bounds. The only method
  // that reads real occupancy; called only on cells inside the sensor footprint.
  virtual bool is_blocked(const State& s) const = 0;
};

class MapBase {
 public:
  virtual ~MapBase() = default;
  virtual std::set<Capability> capabilities() const = 0;
  bool supports(Capability c) const { return capabilities().count(c) > 0; }
};

}  // namespace navigation::core
