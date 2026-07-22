#pragma once

#include <vector>

#include "navigation/core/planner.hpp"
#include "navigation/core/trace.hpp"
#include "navigation/core/types.hpp"

// Closed-loop control-tick simulator shared by every local planner's demo,
// test suite, and bench matrix. Lives at the local_planning category root
// (not core: this is an execution harness, not a contract type; not demos:
// integration/termination/metric logic is not assembly; not tools: cpp demos
// need it too and tools stays Python-only) -- mirrors the `_sampling.py`
// precedent of category-shared machinery living in the category package.
namespace navigation::local_planning {

// Terminal outcome of one simulate() episode.
enum class SimStatus { REACHED, COLLISION, STALLED, TIMEOUT };

// Harness knobs: control cadence, termination tolerances, footprint, and
// stall detection. Declared separately from the planner's own ParamSet
// because these describe the execution loop, not the algorithm (they still
// arrive as ordinary entries in the same yaml -- param_schema.json has no
// separate "sim" section -- but the assembly layer splits them out here).
struct SimConfig {
  double control_dt = 0.0;
  int max_steps = 0;
  double goal_tolerance = 0.0;
  double footprint_radius = 0.0;
  int stall_window = 0;
  double stall_distance = 0.0;
};

struct SimResult {
  SimStatus status = SimStatus::TIMEOUT;
  bool success = false;  // success == (status == REACHED)
  int steps = 0;
  double time_to_goal = 0.0;      // steps * control_dt
  double distance_traveled = 0.0;
  double min_clearance = 0.0;
  std::vector<core::Pose> trajectory;  // start pose included
};

// Exact-arc unicycle integration over one control tick: the closed-form
// solution for constant (v, omega) held over dt, not a first-order Euler
// step -- so dt size never leaks discretization error into the executed
// path (that error would otherwise mix with an algorithm's own
// characteristic behavior, e.g. PF oscillation or PP tracking offset), and
// the fixed operation count keeps both language ports mirrorable tick for
// tick.
core::Pose integrate_unicycle(const core::Pose& pose, const core::VelocityCommand& cmd, double dt);

// Runs one episode: repeatedly asks `planner` for a velocity command and
// integrates it until the robot reaches task.goal, collides, stalls (net
// displacement over a sliding window below stall_distance -- catches both a
// force dropping to zero and in-place oscillation, which a velocity-based
// check would miss), or exhausts max_steps.
SimResult simulate(core::LocalPlanner<core::ObstacleQuery>& planner, core::ObstacleQuery& space,
                   const core::RobotState& start, const core::LocalTask& task,
                   const SimConfig& config, core::TraceRecorder* recorder);

}  // namespace navigation::local_planning
