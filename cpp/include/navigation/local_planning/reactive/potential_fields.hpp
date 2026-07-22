#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

// Potential Fields (Khatib 1986): sum an attractive pull toward the goal with
// a FIRAS repulsive push from every obstacle within range, and steer toward
// the resultant force every control tick. No search, no memory -- each tick
// is a fresh vector sum, so the planner is entirely stateless (reset() stays
// the base no-op).
namespace navigation::local_planning {

class PotentialFieldsPlanner final : public core::ObstacleLocalPlanner {
 public:
  explicit PotentialFieldsPlanner(core::ParamSet params);
  std::string name() const override { return "potential_fields"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::OBSTACLE_QUERY};
  }
  core::VelocityCommand compute_command(core::ObstacleQuery& space, const core::RobotState& state,
                                        const core::LocalTask& task, double dt,
                                        core::TraceRecorder* recorder) override;

 private:
  double k_att_;
  double k_rep_;
  double influence_radius_;
  double k_v_;
  double k_omega_;
  double max_speed_;
  double max_omega_;
  // Contact clamp for the repulsive 1/d term -- the footprint radius is the
  // closest the robot center can approach an obstacle center before
  // collision, so it doubles as the smallest physically meaningful d.
  double d_min_;
};

}  // namespace navigation::local_planning
