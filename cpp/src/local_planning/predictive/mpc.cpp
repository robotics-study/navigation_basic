#include "navigation/local_planning/predictive/mpc.hpp"

#include <utility>
#include <vector>

namespace navigation::local_planning {

MpcPlanner::MpcPlanner(core::ParamSet params)
    : core::ObstacleLocalPlanner(std::move(params)),
      horizon_(params_.get_int("horizon")),
      iterations_(params_.get_int("iterations")),
      step_alpha_(params_.get_float("step_alpha")),
      grad_eps_(params_.get_float("grad_eps")),
      max_step_v_(params_.get_float("max_step_v")),
      max_step_omega_(params_.get_float("max_step_omega")),
      w_goal_(params_.get_float("w_goal")),
      w_obstacle_(params_.get_float("w_obstacle")),
      w_control_(params_.get_float("w_control")),
      min_obstacle_dist_(params_.get_float("min_obstacle_dist")),
      v_max_(params_.get_float("v_max")),
      omega_max_(params_.get_float("omega_max")),
      a_max_(params_.get_float("a_max")),
      footprint_radius_(params_.get_float("footprint_radius")) {}

std::vector<Control> MpcPlanner::warm_start() const {
  if (controls_.empty()) return std::vector<Control>(static_cast<size_t>(horizon_), Control{0.0, 0.0});
  std::vector<Control> shifted(controls_.begin() + 1, controls_.end());
  shifted.push_back(controls_.back());
  return shifted;
}

double MpcPlanner::cost(core::ObstacleQuery& space, const core::Pose& s0,
                        const std::vector<Control>& controls, const core::Pose& goal,
                        double h) const {
  std::vector<core::Pose> traj = rollout(s0, controls, h);
  return sequence_cost(space, traj, controls, goal, footprint_radius_, w_goal_, w_obstacle_,
                       min_obstacle_dist_, w_control_);
}

void MpcPlanner::descent_step(core::ObstacleQuery& space, const core::Pose& s0,
                              std::vector<Control>& controls, const core::Pose& goal,
                              double h) const {
  const size_t n = controls.size();
  const double eps = grad_eps_;
  std::vector<double> grad_v(n, 0.0), grad_omega(n, 0.0);
  for (size_t k = 0; k < n; ++k) {
    const double v = controls[k].first, omega = controls[k].second;
    controls[k] = Control{v + eps, omega};
    const double jvp = cost(space, s0, controls, goal, h);
    controls[k] = Control{v - eps, omega};
    const double jvm = cost(space, s0, controls, goal, h);
    grad_v[k] = (jvp - jvm) / (2.0 * eps);
    controls[k] = Control{v, omega + eps};
    const double jwp = cost(space, s0, controls, goal, h);
    controls[k] = Control{v, omega - eps};
    const double jwm = cost(space, s0, controls, goal, h);
    grad_omega[k] = (jwp - jwm) / (2.0 * eps);
    controls[k] = Control{v, omega};
  }

  for (size_t k = 0; k < n; ++k) {
    double v = controls[k].first, omega = controls[k].second;
    v -= clamp(step_alpha_ * grad_v[k], -max_step_v_, max_step_v_);
    omega -= clamp(step_alpha_ * grad_omega[k], -max_step_omega_, max_step_omega_);
    controls[k] = Control{clamp(v, 0.0, v_max_), clamp(omega, -omega_max_, omega_max_)};
  }
}

void MpcPlanner::emit_band(core::TraceRecorder& recorder, const core::Pose& s0,
                           const std::vector<core::Pose>& traj, double h, double total_cost) const {
  std::vector<std::vector<double>> band;
  band.reserve(traj.size() + 1);
  band.push_back({s0.x, s0.y, s0.theta, 0.0});
  for (const core::Pose& p : traj) band.push_back({p.x, p.y, p.theta, h});
  recorder.band_updated(band, {{"iterations", static_cast<double>(iterations_)},
                               {"horizon", static_cast<double>(horizon_)},
                               {"total_cost", total_cost}});
}

core::VelocityCommand MpcPlanner::compute_command(core::ObstacleQuery& space,
                                                  const core::RobotState& state,
                                                  const core::LocalTask& task, double dt,
                                                  core::TraceRecorder* recorder) {
  const core::Pose& s0 = state.pose;
  const core::Pose& goal = task.goal;
  // Prediction step equals the control period (dt): predicting and executing on
  // the same discretization keeps the executed u_0 consistent with the horizon
  // it was optimized in.
  const double h = dt;

  std::vector<Control> controls = warm_start();
  for (int i = 0; i < iterations_; ++i) descent_step(space, s0, controls, goal, h);
  controls_ = controls;

  if (recorder) {
    std::vector<core::Pose> traj = rollout(s0, controls, h);
    const double total_cost = sequence_cost(space, traj, controls, goal, footprint_radius_, w_goal_,
                                            w_obstacle_, min_obstacle_dist_, w_control_);
    emit_band(*recorder, s0, traj, h, total_cost);
  }

  // Executed command: accel-limit the linear speed against the velocity the
  // simulator reports (RobotState.v, like DWA -- no separate v_prev_ member),
  // then box-clamp so the executed command is always within limits.
  double v0 = controls[0].first, omega0 = controls[0].second;
  v0 = clamp(v0, state.v - a_max_ * h, state.v + a_max_ * h);
  v0 = clamp(v0, 0.0, v_max_);
  omega0 = clamp(omega0, -omega_max_, omega_max_);
  return core::VelocityCommand{v0, omega0};
}

}  // namespace navigation::local_planning
