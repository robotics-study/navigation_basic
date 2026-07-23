#pragma once

#include <set>
#include <string>
#include <vector>

#include "navigation/core/planner.hpp"
#include "navigation/local_planning/predictive/rollout.hpp"

// Model Predictive Control for a kinematic mobile robot (Klančar & Škrjanc,
// Robotics and Autonomous Systems 55(6):460-469, 2007, DOI
// 10.1016/j.robot.2007.01.002): each tick predicts the next H steps with the
// unicycle model, optimizes the control sequence U against the shared receding-
// horizon cost J(U), executes only the first control u_0, and re-optimizes next
// tick (receding horizon). The optimizer is fixed-iteration projected gradient
// descent with a central finite-difference gradient -- finite differences
// (rather than a hand-derived analytic Jacobian) so the gradient follows
// mechanically from the one shared scalar cost, keeping Python/C++/TS in step
// and the contrast with MPPI a clean "same J, sampling instead of gradient".
namespace navigation::local_planning {

class MpcPlanner final : public core::ObstacleLocalPlanner {
 public:
  explicit MpcPlanner(core::ParamSet params);

  std::string name() const override { return "mpc"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::OBSTACLE_QUERY};
  }
  void reset() override { controls_.clear(); }

  core::VelocityCommand compute_command(core::ObstacleQuery& space, const core::RobotState& state,
                                        const core::LocalTask& task, double dt,
                                        core::TraceRecorder* recorder) override;

 private:
  // Left-shift the previous U and duplicate its last control (the executed u_0
  // is dropped), so this tick starts where the last horizon left off. Cold
  // start (empty controls_) seeds H zero controls.
  std::vector<Control> warm_start() const;
  double cost(core::ObstacleQuery& space, const core::Pose& s0, const std::vector<Control>& controls,
              const core::Pose& goal, double h) const;
  // One projected gradient-descent iteration: a full 2H-component central
  // finite-difference gradient of J at the current U (component order k
  // ascending, v before omega -- part of the cross-language determinism
  // contract), then one U <- U - step_alpha*grad update with per-component step
  // clamp and box projection (v in [0, v_max], |omega| <= omega_max).
  void descent_step(core::ObstacleQuery& space, const core::Pose& s0,
                    std::vector<Control>& controls, const core::Pose& goal, double h) const;
  void emit_band(core::TraceRecorder& recorder, const core::Pose& s0,
                 const std::vector<core::Pose>& traj, double h, double total_cost) const;

  int horizon_;
  int iterations_;
  double step_alpha_;
  double grad_eps_;
  double max_step_v_;
  double max_step_omega_;
  double w_goal_;
  double w_obstacle_;
  double w_control_;
  double min_obstacle_dist_;
  double v_max_;
  double omega_max_;
  double a_max_;
  double footprint_radius_;

  // Control sequence U carried across ticks for warm-starting; empty = cold
  // start (first tick / after reset()), which seeds U with zeros.
  std::vector<Control> controls_;
};

}  // namespace navigation::local_planning
