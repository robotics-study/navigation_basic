#pragma once

#include <set>
#include <string>
#include <utility>

#include "navigation/core/planner.hpp"

namespace navigation::local_planning {

// Stanley (Thrun et al. 2006, DARPA Grand Challenge, sec. 9.2), with the
// k_soft low-speed softening term from Hoffmann et al. 2007. Unlike Pure
// Pursuit's single lookahead arc, Stanley steers on two errors measured at
// the front axle: heading misalignment with the path tangent and lateral
// (crosstrack) offset.
class StanleyPlanner final : public core::ObstacleLocalPlanner {
 public:
  explicit StanleyPlanner(core::ParamSet params)
      : core::ObstacleLocalPlanner(std::move(params)) {}

  std::string name() const override { return "stanley"; }
  std::set<core::Capability> required_capabilities() const override {
    // Same rationale as PurePursuitPlanner: the simulator's closed-loop
    // contract is defined over ObstacleQuery regardless of whether a given
    // tracker queries obstacles.
    return {core::Capability::OBSTACLE_QUERY};
  }
  bool requires_reference_path() const override { return true; }
  // Monotonic forward-only, same rationale as PurePursuitPlanner.
  void reset() override { progress_index_ = 0; }

  core::VelocityCommand compute_command(core::ObstacleQuery& space, const core::RobotState& state,
                                        const core::LocalTask& task, double dt,
                                        core::TraceRecorder* recorder) override;

 private:
  int progress_index_ = 0;
};

}  // namespace navigation::local_planning
