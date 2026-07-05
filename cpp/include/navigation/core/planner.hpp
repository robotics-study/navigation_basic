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

}  // namespace navigation::core
