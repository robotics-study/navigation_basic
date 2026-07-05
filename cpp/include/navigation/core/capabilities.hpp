#pragma once

#include <set>
#include <utility>
#include <vector>

namespace navigation::core {

// OBSTACLE_QUERY is declared for future local planners but implemented by no map
// here; OccupancyGrid2D must not advertise it.
enum class Capability { DISCRETE_SPACE, SAMPLING_SPACE, OBSTACLE_QUERY, LINE_OF_SIGHT_SPACE };

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

class MapBase {
 public:
  virtual ~MapBase() = default;
  virtual std::set<Capability> capabilities() const = 0;
  bool supports(Capability c) const { return capabilities().count(c) > 0; }
};

}  // namespace navigation::core
