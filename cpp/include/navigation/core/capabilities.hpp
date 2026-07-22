#pragma once

#include <set>
#include <utility>
#include <vector>

#include "navigation/core/types.hpp"  // Footprint / Pose for SE2CollisionSpace (same core layer)

namespace navigation::core {

enum class Capability {
  DISCRETE_SPACE,
  SAMPLING_SPACE,
  OBSTACLE_QUERY,
  LINE_OF_SIGHT_SPACE,
  DYNAMIC_GRID_SPACE,
  SE2_COLLISION_SPACE
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

// Continuous SE(2) collision view for kinodynamic planners (Hybrid A*, Dolgov,
// Thrun, Montemerlo & Diebel 2008). Standalone — extends nothing: the planner
// generates its own motion primitives + heuristic and needs ONLY a footprint
// collision test at a continuous world pose. World<->cell conversion stays in the
// map. Distinct from ObstacleQuery below (local planners; adds a clearance
// query) even though ObstacleQuery extends this class — kinodynamic planners
// only ever need collision, never clearance. Only maps with real occupancy
// geometry can answer it.
template <class State>
class SE2CollisionSpace {
 public:
  virtual ~SE2CollisionSpace() = default;
  // True iff the footprint placed at world pose `pose` overlaps any occupied or
  // out-of-bounds cell. Continuous pose in; grid conversion happens in the map.
  virtual bool is_collision(const Footprint& footprint, const State& pose) const = 0;
};

// Local-planner obstacle view: footprint collision (inherited from
// SE2CollisionSpace) + nearest-obstacle clearance + neighborhood occupancy
// enumeration. Non-template, unlike the other capabilities above — local
// planners always run in world SE(2) (RobotState), so there is no discrete
// state type to parametrize over. Extends SE2CollisionSpace<Pose> instead of
// redeclaring is_collision, mirroring the LineOfSightSpace : DiscreteSpace
// extension above.
class ObstacleQuery : public SE2CollisionSpace<Pose> {
 public:
  virtual ~ObstacleQuery() = default;
  // Euclidean distance (meters) from p's cell center to the center of the
  // nearest non-free (occupied or out-of-bounds) cell.
  virtual double distance_to_nearest(const Point& p) const = 0;
  // Centers (world) of every non-free cell within `radius` of `center`,
  // including out-of-bounds cells, in row-ascending then column-ascending
  // order. The order is part of the contract: potential-field force summation
  // and VFH histogram accumulation both fold over this sequence, and their
  // floating-point sums depend on fold order for cross-language parity.
  virtual std::vector<Point> occupied_within(const Point& center, double radius) const = 0;
};

class MapBase {
 public:
  virtual ~MapBase() = default;
  virtual std::set<Capability> capabilities() const = 0;
  bool supports(Capability c) const { return capabilities().count(c) > 0; }
};

}  // namespace navigation::core
