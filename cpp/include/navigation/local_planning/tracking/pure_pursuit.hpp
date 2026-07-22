#pragma once

#include <set>
#include <string>
#include <utility>

#include "navigation/core/planner.hpp"

namespace navigation::local_planning {

// Pure Pursuit (Coulter 1992, CMU-RI-TR-92-01): chases the point where a
// lookahead circle centered on the robot meets the reference path, steering
// along the constant-curvature arc through it. Purely geometric -- see
// required_capabilities() below for why it still binds ObstacleQuery despite
// never querying obstacles.
class PurePursuitPlanner final : public core::ObstacleLocalPlanner {
 public:
  explicit PurePursuitPlanner(core::ParamSet params)
      : core::ObstacleLocalPlanner(std::move(params)) {}

  std::string name() const override { return "pure_pursuit"; }
  std::set<core::Capability> required_capabilities() const override {
    // The closed-loop execution contract (collision / clearance) is defined
    // over ObstacleQuery and the simulator always requires it, so this
    // planner binds the same view even though it never queries obstacles --
    // keeping the simulator single-path instead of branching per planner.
    return {core::Capability::OBSTACLE_QUERY};
  }
  bool requires_reference_path() const override { return true; }
  // Monotonic forward-only so a self-crossing path never snaps the pursuit
  // point backward to an earlier, geometrically-closer crossing.
  void reset() override { progress_index_ = 0; }

  core::VelocityCommand compute_command(core::ObstacleQuery& space, const core::RobotState& state,
                                        const core::LocalTask& task, double dt,
                                        core::TraceRecorder* recorder) override;

 private:
  int progress_index_ = 0;
};

}  // namespace navigation::local_planning
