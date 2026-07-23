#pragma once

#include <limits>
#include <optional>
#include <vector>

#include "navigation/core/capabilities.hpp"
#include "navigation/core/trace.hpp"
#include "navigation/core/types.hpp"
#include "navigation/local_planning/simulation.hpp"
#include "navigation/local_planning/velocity/velocity_obstacle.hpp"

// Multi-agent closed-loop harness for the velocity-obstacle family.
//
// N bodies share one map; each is either driven by its own
// VelocityObstaclePlanner or moves at a fixed scripted velocity (a
// non-cooperative mover for single-planner demos). Every tick is
// read-all-then-write: every body's command is computed against the SAME
// pre-tick snapshot of every other body, then all bodies integrate together
// -- order-independent and reproducible, which reciprocal avoidance (van den
// Berg et al. 2008) depends on (a sequential update would make agent 0 react
// to agent 1's *old* position while agent 1 reacts to agent 0's *new* one,
// breaking the reciprocity the algorithm assumes).
//
// Reuses simulation.hpp's integrate_unicycle/SimConfig/SimStatus rather than
// re-deriving single-body integration or status enums here.
namespace navigation::local_planning {

struct AgentSpec {
  core::RobotState start;
  core::Pose goal;
  double radius = 0.0;
  // nullopt -> driven by the matching planner; a value -> a non-cooperative
  // constant-velocity mover (the VO/RVO/ORCA demos' scripted crossing traffic).
  std::optional<core::Point> scripted_velocity;
};

struct AgentResult {
  SimStatus status = SimStatus::TIMEOUT;
  int steps = 0;
  std::vector<core::Pose> trajectory;
  double min_pair_clearance = std::numeric_limits<double>::infinity();
};

// Runs every agent in closed loop until every planner-driven agent reaches
// its goal, a collision (pairwise or static) occurs, the group stalls, or the
// step budget runs out. `planners[k] == nullptr` iff `specs[k]` is scripted.
//
// Reached agents are not special-cased out of planning/integration: once an
// agent is near its own goal, preferred_velocity alone drives it toward
// (0, 0), so it settles in place naturally and keeps contributing an
// (approximately stationary) DynamicObstacle snapshot to its neighbors.
std::vector<AgentResult> simulate_agents(const std::vector<VelocityObstaclePlanner*>& planners,
                                         const std::vector<AgentSpec>& specs,
                                         core::ObstacleQuery& space, const SimConfig& config,
                                         core::TraceRecorder* recorder = nullptr);

}  // namespace navigation::local_planning
