#pragma once

#include <set>
#include <string>
#include <utility>

#include "navigation/core/planner.hpp"

namespace navigation::local_planning {

// Regulated Pure Pursuit (Macenski, Singh, Martin & Gines 2023, "Regulated
// Pure Pursuit for Robot Path Tracking"; Nav2's default local controller).
// Builds on the plain lookahead-arc law (Coulter 1992) but adds three
// regulations Pure Pursuit never had: an adaptive lookahead distance
// (lineage Campbell 2007), a curvature-proportional speed cap, and a
// proximity-proportional speed cap, plus a predictive lookahead collision
// stop. Shares only the reference-path geometry (progress index + lookahead
// point) with PurePursuitPlanner via path.hpp; regulation and command
// assembly are independent by design (see regulated_pure_pursuit.cpp).
class RegulatedPurePursuitPlanner final : public core::ObstacleLocalPlanner {
 public:
  explicit RegulatedPurePursuitPlanner(core::ParamSet params)
      : core::ObstacleLocalPlanner(std::move(params)) {}

  std::string name() const override { return "regulated_pure_pursuit"; }
  std::set<core::Capability> required_capabilities() const override {
    // Unlike PurePursuitPlanner, this planner genuinely queries ObstacleQuery
    // every tick: proximity regulation and the lookahead collision check both
    // depend on it.
    return {core::Capability::OBSTACLE_QUERY};
  }
  bool requires_reference_path() const override { return true; }
  // Monotonic forward-only, same rationale as PurePursuitPlanner/StanleyPlanner.
  void reset() override { progress_index_ = 0; }

  core::VelocityCommand compute_command(core::ObstacleQuery& space, const core::RobotState& state,
                                        const core::LocalTask& task, double dt,
                                        core::TraceRecorder* recorder) override;

 private:
  int progress_index_ = 0;
};

}  // namespace navigation::local_planning
