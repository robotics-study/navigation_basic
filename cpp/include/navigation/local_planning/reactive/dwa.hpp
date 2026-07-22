#pragma once

#include <set>
#include <string>
#include <utility>
#include <vector>

#include "navigation/core/planner.hpp"

// Dynamic Window Approach (Fox, Burgard & Thrun 1997, DOI 10.1109/100.580977):
// searches the reachable (v, omega) command space directly instead of a
// Cartesian path. Every tick samples a deterministic grid inside the box
// formed by the robot's kinematic limits intersected with what one
// accel-limited tick can reach from the current velocity, rolls each
// candidate forward as a constant-command arc, discards ones that collide or
// cannot stop before the nearest obstacle, and picks the highest-scoring
// survivor. No search, no memory across ticks -- reset() stays the base
// no-op.
namespace navigation::local_planning {

class DwaPlanner final : public core::ObstacleLocalPlanner {
 public:
  explicit DwaPlanner(core::ParamSet params);

  std::string name() const override { return "dwa"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::OBSTACLE_QUERY};
  }
  core::VelocityCommand compute_command(core::ObstacleQuery& space, const core::RobotState& state,
                                        const core::LocalTask& task, double dt,
                                        core::TraceRecorder* recorder) override;

 private:
  // Predicted trajectory for one candidate (v, omega) held constant over
  // sim_time, sampled at sim_steps equally spaced instants. Fills the
  // caller's buffer instead of returning a fresh vector so the candidate
  // loop reuses one allocation per tick (hot path).
  void rollout(const core::Pose& pose, double v, double omega, std::vector<core::Pose>& out) const;

  // Scores one candidate. Returns (cost, heading, clearance, velocity,
  // admissible); a colliding rollout is rejected outright (all four numeric
  // fields zeroed) rather than scored, per Fox 1997's hard obstacle
  // constraint.
  struct Score {
    double cost = 0.0;
    double heading = 0.0;
    double clearance = 0.0;
    double velocity = 0.0;
    bool admissible = false;
  };
  Score score(core::ObstacleQuery& space, const core::Footprint& footprint,
             const std::vector<core::Pose>& rollout, double v, double omega, double goal_x,
             double goal_y) const;

  // No admissible candidate survived this tick: brake at the kinematic
  // limits. Persistent triggering is a local minimum, honestly reported as
  // STALLED by the simulator's stall detector.
  std::pair<double, double> decelerate(double v_a, double omega_a, double dt) const;

  double max_speed_;
  double min_speed_;
  double max_omega_;
  double accel_;
  double accel_omega_;
  int v_samples_;
  int omega_samples_;
  double sim_time_;
  int sim_steps_;
  double heading_weight_;
  double clearance_weight_;
  double velocity_weight_;
  double clearance_limit_;
  double slow_radius_;
  double footprint_radius_;
};

}  // namespace navigation::local_planning
