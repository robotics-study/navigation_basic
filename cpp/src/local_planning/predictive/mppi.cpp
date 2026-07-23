#include "navigation/local_planning/predictive/mppi.hpp"

#include <cmath>
#include <utility>
#include <vector>

namespace navigation::local_planning {

MppiPlanner::MppiPlanner(core::ParamSet params)
    : core::ObstacleLocalPlanner(std::move(params)),
      horizon_(params_.get_int("horizon")),
      num_samples_(params_.get_int("num_samples")),
      temperature_(params_.get_float("temperature")),
      sigma_v_(params_.get_float("sigma_v")),
      sigma_omega_(params_.get_float("sigma_omega")),
      w_goal_(params_.get_float("w_goal")),
      w_obstacle_(params_.get_float("w_obstacle")),
      w_control_(params_.get_float("w_control")),
      min_obstacle_dist_(params_.get_float("min_obstacle_dist")),
      v_max_(params_.get_float("v_max")),
      omega_max_(params_.get_float("omega_max")),
      a_max_(params_.get_float("a_max")),
      footprint_radius_(params_.get_float("footprint_radius")),
      seed_(params_.get_int("seed")),
      rng_(static_cast<std::mt19937::result_type>(seed_)) {}

std::vector<Control> MppiPlanner::warm_start() const {
  if (controls_.empty()) return std::vector<Control>(static_cast<size_t>(horizon_), Control{0.0, 0.0});
  std::vector<Control> shifted(controls_.begin() + 1, controls_.end());
  shifted.push_back(controls_.back());
  return shifted;
}

double MppiPlanner::gaussian() {
  if (spare_.has_value()) {
    const double z = *spare_;
    spare_.reset();
    return z;
  }
  const double u1 = 1.0 - unit_(rng_);
  const double u2 = unit_(rng_);
  const double magnitude = std::sqrt(-2.0 * std::log(u1));
  spare_ = magnitude * std::sin(2.0 * M_PI * u2);
  return magnitude * std::cos(2.0 * M_PI * u2);
}

void MppiPlanner::emit_band(core::TraceRecorder& recorder, const core::Pose& s0,
                            const std::vector<core::Pose>& traj, double h, double min_cost) const {
  std::vector<std::vector<double>> band;
  band.reserve(traj.size() + 1);
  band.push_back({s0.x, s0.y, s0.theta, 0.0});
  for (const core::Pose& p : traj) band.push_back({p.x, p.y, p.theta, h});
  recorder.band_updated(band, {{"min_cost", min_cost},
                               {"num_samples", static_cast<double>(num_samples_)},
                               {"temperature", temperature_}});
}

core::VelocityCommand MppiPlanner::compute_command(core::ObstacleQuery& space,
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
  const size_t horizon = controls.size();

  // Sample K perturbed control sequences and score each. Every RNG draw is
  // unconditional (outside the recorder guard): the samples determine the
  // trajectory, so tracing on/off must not change what the robot does.
  std::vector<std::vector<Control>> eps_samples;
  eps_samples.reserve(static_cast<size_t>(num_samples_));
  std::vector<double> costs;
  costs.reserve(static_cast<size_t>(num_samples_));
  std::vector<core::Point> terminals;
  std::vector<std::vector<std::vector<double>>> rollouts_xy;
  if (recorder) {
    terminals.reserve(static_cast<size_t>(num_samples_));
    rollouts_xy.reserve(static_cast<size_t>(num_samples_));
  }
  for (int k = 0; k < num_samples_; ++k) {
    std::vector<Control> eps_seq(horizon);
    std::vector<Control> perturbed(horizon);
    for (size_t j = 0; j < horizon; ++j) {
      const double eps_v = gaussian() * sigma_v_;
      const double eps_omega = gaussian() * sigma_omega_;
      eps_seq[j] = Control{eps_v, eps_omega};
      perturbed[j] = Control{clamp(controls[j].first + eps_v, 0.0, v_max_),
                             clamp(controls[j].second + eps_omega, -omega_max_, omega_max_)};
    }
    std::vector<core::Pose> traj = rollout(s0, perturbed, h);
    const double cost = sequence_cost(space, traj, perturbed, goal, footprint_radius_, w_goal_,
                                      w_obstacle_, min_obstacle_dist_, w_control_);
    eps_samples.push_back(std::move(eps_seq));
    costs.push_back(cost);
    if (recorder) {
      const core::Pose& end = traj.back();
      terminals.push_back(core::Point{end.x, end.y});
      std::vector<std::vector<double>> xy;
      xy.reserve(traj.size());
      for (const core::Pose& p : traj) xy.push_back({p.x, p.y});
      rollouts_xy.push_back(std::move(xy));
    }
  }

  // Softmax importance weights. Subtracting beta = min_k S_k before exp keeps the
  // exponentials in range (the min-cost sample contributes exp(0) = 1, so the
  // normalizer is >= 1 and never underflows); the shift cancels in the
  // normalization (Williams et al. 2018).
  size_t best_index = 0;
  double beta = costs[0];
  for (size_t k = 1; k < costs.size(); ++k) {
    if (costs[k] < beta) {
      beta = costs[k];
      best_index = k;
    }
  }
  std::vector<double> weights(costs.size());
  double total = 0.0;
  for (size_t k = 0; k < costs.size(); ++k) {
    const double w = std::exp(-(costs[k] - beta) / temperature_);
    weights[k] = w;
    total += w;
  }
  const double inv_total = 1.0 / total;
  for (double& w : weights) w *= inv_total;

  // Update the nominal sequence by the weighted average of the raw noise, then
  // box-project. Accumulation order (j outer, k inner) is a cross-language
  // determinism contract.
  for (size_t j = 0; j < horizon; ++j) {
    double acc_v = 0.0;
    double acc_omega = 0.0;
    for (int k = 0; k < num_samples_; ++k) {
      const double w = weights[static_cast<size_t>(k)];
      acc_v += w * eps_samples[static_cast<size_t>(k)][j].first;
      acc_omega += w * eps_samples[static_cast<size_t>(k)][j].second;
    }
    controls[j] = Control{clamp(controls[j].first + acc_v, 0.0, v_max_),
                          clamp(controls[j].second + acc_omega, -omega_max_, omega_max_)};
  }
  controls_ = controls;

  if (recorder) {
    for (int k = 0; k < num_samples_; ++k) {
      const size_t ki = static_cast<size_t>(k);
      recorder->candidate_evaluated(
          terminals[ki], costs[ki],
          {{"weight", weights[ki]}, {"selected", ki == best_index ? 1.0 : 0.0}}, rollouts_xy[ki]);
    }
    std::vector<core::Pose> nominal_traj = rollout(s0, controls, h);
    emit_band(*recorder, s0, nominal_traj, h, beta);
  }

  // Executed command: accel-limit the linear speed against the velocity the
  // simulator reports (RobotState.v, like DWA/MPC -- no separate v_prev_ member),
  // then box-clamp so the executed command is always within limits.
  double v0 = controls[0].first, omega0 = controls[0].second;
  v0 = clamp(v0, state.v - a_max_ * h, state.v + a_max_ * h);
  v0 = clamp(v0, 0.0, v_max_);
  omega0 = clamp(omega0, -omega_max_, omega_max_);
  return core::VelocityCommand{v0, omega0};
}

}  // namespace navigation::local_planning
