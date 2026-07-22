#pragma once

#include <set>
#include <string>
#include <utility>

#include "navigation/core/capabilities.hpp"
#include "navigation/core/params.hpp"
#include "navigation/core/trace.hpp"
#include "navigation/core/types.hpp"

namespace navigation::core {

// Base for all global planners. State is the search state (Cell or Point); Space
// is the capability interface the planner searches over. Concrete planners are
// constructed from a validated ParamSet and never see a concrete map type.
template <class State, class Space>
class GlobalPlanner {
 public:
  explicit GlobalPlanner(ParamSet params) : params_(std::move(params)) {}
  virtual ~GlobalPlanner() = default;

  virtual std::string name() const = 0;
  virtual std::set<Capability> required_capabilities() const = 0;
  virtual PlanResult<State> plan(Space& space, const State& start, const State& goal,
                                 TraceRecorder* recorder) = 0;

 protected:
  ParamSet params_;
};

using DiscretePlanner = GlobalPlanner<Cell, DiscreteSpace<Cell>>;
using SamplingPlanner = GlobalPlanner<Point, SamplingSpace<Point>>;
using LineOfSightPlanner = GlobalPlanner<Cell, LineOfSightSpace<Cell>>;
using DynamicGridPlanner = GlobalPlanner<Cell, DynamicGridSpace<Cell>>;
using SE2CollisionPlanner = GlobalPlanner<Pose, SE2CollisionSpace<Pose>>;

// Base for all local planners: one control tick maps (map capability, robot
// state, task) -> a velocity command. Unlike GlobalPlanner there is no State
// template parameter — local-planning state is always RobotState in world
// SE(2); no local planner searches over grid cells, so a second type parameter
// would sit unused on every concrete planner.
template <class Space>
class LocalPlanner {
 public:
  explicit LocalPlanner(ParamSet params) : params_(std::move(params)) {}
  virtual ~LocalPlanner() = default;

  virtual std::string name() const = 0;
  virtual std::set<Capability> required_capabilities() const = 0;
  // Episode-start hook for stateful planners (e.g. Pure Pursuit's monotonic
  // path-progress index). Not abstract: stateless planners (Potential Fields,
  // VFH) would otherwise be forced to implement an empty override for no
  // reason.
  virtual void reset() {}
  // True iff the planner cannot run without task.reference_path. The assembly
  // layer (demo/bench) checks this BEFORE starting the control loop and never
  // inside compute_command, matching the hot-path-no-exceptions convention.
  virtual bool requires_reference_path() const { return false; }
  virtual VelocityCommand compute_command(Space& space, const RobotState& state,
                                          const LocalTask& task, double dt,
                                          TraceRecorder* recorder) = 0;

 protected:
  ParamSet params_;
};

using ObstacleLocalPlanner = LocalPlanner<ObstacleQuery>;

}  // namespace navigation::core
