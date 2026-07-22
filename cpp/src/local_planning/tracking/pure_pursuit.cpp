#include "navigation/local_planning/tracking/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>

#include "navigation/local_planning/geometry.hpp"
#include "navigation/local_planning/tracking/path.hpp"

namespace navigation::local_planning {

namespace {

// Below this curvature the clamp-recompute step (v = omega / kappa) would divide
// by a near-zero denominator. At |kappa| this small, kappa*v is already far under
// any reasonable max_omega so the clamp branch never actually fires in practice --
// this guards the division defensively rather than tuning any real behavior.
constexpr double kKappaEps = 1e-9;

}  // namespace

core::VelocityCommand PurePursuitPlanner::compute_command(core::ObstacleQuery& space,
                                                           const core::RobotState& state,
                                                           const core::LocalTask& task, double dt,
                                                           core::TraceRecorder* recorder) {
  (void)space;  // pure tracker -- see required_capabilities() for why the
                // capability is still declared.
  (void)dt;     // geometric law; no integration inside a single tick's command.
  const auto& path = task.reference_path;
  const double x = state.pose.x, y = state.pose.y, theta = state.pose.theta;
  const core::Point robot_xy{x, y};

  progress_index_ = advance_progress_index(path, robot_xy, progress_index_);

  const double lookahead_distance = params_.get_float("lookahead_distance");
  const core::Point target = lookahead_point(path, progress_index_, robot_xy, lookahead_distance);

  const double alpha = wrap_to_pi(std::atan2(target.y - y, target.x - x) - theta);
  const double kappa = 2.0 * std::sin(alpha) / lookahead_distance;

  const double max_speed = params_.get_float("max_speed");
  const double slow_radius = params_.get_float("slow_radius");
  const double remaining = std::hypot(task.goal.x - x, task.goal.y - y);
  double v = max_speed * std::min(1.0, remaining / slow_radius);

  const double max_omega = params_.get_float("max_omega");
  const double omega_raw = kappa * v;
  double omega = std::max(-max_omega, std::min(max_omega, omega_raw));
  if (omega != omega_raw && std::fabs(kappa) > kKappaEps) {
    // Clamp changed the turn rate -- recompute v so the executed (v, omega)
    // still traces the commanded curvature kappa instead of silently
    // understeering relative to the geometric lookahead arc.
    v = omega / kappa;
  }

  if (recorder) {
    recorder->candidate_evaluated(target, kappa, {{"alpha", alpha}});
  }
  return core::VelocityCommand{v, omega};
}

}  // namespace navigation::local_planning
