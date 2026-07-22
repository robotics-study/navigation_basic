#include "navigation/local_planning/tracking/stanley.hpp"

#include <algorithm>
#include <cmath>

#include "navigation/local_planning/geometry.hpp"
#include "navigation/local_planning/tracking/path.hpp"

namespace navigation::local_planning {

namespace {

// Below this |tan(delta)| the clamp-recompute step (v = omega*L/tan(delta))
// would divide by a near-zero denominator. max_steer is declared < pi/2 (see
// stanley.yaml), so tan(delta) itself never diverges; this only guards the
// division defensively, mirroring pure_pursuit.cpp's kKappaEps.
constexpr double kTanEps = 1e-9;

}  // namespace

core::VelocityCommand StanleyPlanner::compute_command(core::ObstacleQuery& space,
                                                       const core::RobotState& state,
                                                       const core::LocalTask& task, double dt,
                                                       core::TraceRecorder* recorder) {
  (void)space;  // pure tracker -- see required_capabilities() for why the
                // capability is still declared.
  (void)dt;     // geometric law; no integration inside a single tick's command.
  const auto& path = task.reference_path;
  const double x = state.pose.x, y = state.pose.y, theta = state.pose.theta;

  // Speed profile first: the steering law below divides by (k_soft + v), so v
  // must be settled before delta is computed.
  const double max_speed = params_.get_float("max_speed");
  const double slow_radius = params_.get_float("slow_radius");
  const double remaining = std::hypot(task.goal.x - x, task.goal.y - y);
  double v = max_speed * std::min(1.0, remaining / slow_radius);

  // Front-axle point: the original paper defines both errors here (Thrun
  // 2006 sec. 9.2), not at the robot's rear-axle/center pose.
  const double wheelbase = params_.get_float("wheelbase");
  const core::Point front{x + wheelbase * std::cos(theta), y + wheelbase * std::sin(theta)};

  progress_index_ = advance_progress_index(path, front, progress_index_);
  const int i = progress_index_;
  const core::Point& a = path[static_cast<size_t>(i)];
  const core::Point& b = path[static_cast<size_t>(i) + 1];
  const double seg_len = std::hypot(b.x - a.x, b.y - a.y);
  const double tx = (b.x - a.x) / seg_len, ty = (b.y - a.y) / seg_len;
  const double theta_path = std::atan2(ty, tx);
  const double psi = wrap_to_pi(theta_path - theta);

  const core::Point foot = closest_point_on_segment(front, a, b);
  // Cross product of the tangent with (front - foot): positive when the
  // front axle sits to the path's left. This is a mirror-image sign
  // convention from the paper's e (right-positive) -- same steering law,
  // only the sign folds differently below.
  const double e = tx * (front.y - foot.y) - ty * (front.x - foot.x);

  const double k_gain = params_.get_float("k_gain");
  const double k_soft = params_.get_float("k_soft");
  const double delta_raw = psi - std::atan(k_gain * e / (k_soft + v));
  const double max_steer = params_.get_float("max_steer");
  const double delta = std::max(-max_steer, std::min(max_steer, delta_raw));

  // Rear-axle kinematic bicycle theta_dot = (v/L)*tan(delta) is exactly the
  // unicycle equation with omega = (v/L)*tan(delta) -- not an approximation,
  // just a different parametrization of the same motion (tire slip / vehicle
  // dynamics beyond this kinematic model are not captured).
  const double max_omega = params_.get_float("max_omega");
  const double omega_raw = v * std::tan(delta) / wheelbase;
  double omega = std::max(-max_omega, std::min(max_omega, omega_raw));
  if (omega != omega_raw && std::fabs(std::tan(delta)) > kTanEps) {
    // Clamp changed the turn rate -- recompute v so the executed (v, omega)
    // still traces the commanded curvature instead of silently understeering
    // relative to delta (PP's clamp-recompute pattern mirrored here).
    v = omega * wheelbase / std::tan(delta);
  }

  if (recorder) {
    recorder->candidate_evaluated(foot, delta, {{"e", e}, {"psi", psi}, {"v", v}});
  }
  return core::VelocityCommand{v, omega};
}

}  // namespace navigation::local_planning
