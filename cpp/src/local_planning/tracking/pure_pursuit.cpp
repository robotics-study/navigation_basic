#include "navigation/local_planning/tracking/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include "navigation/local_planning/geometry.hpp"

namespace navigation::local_planning {

namespace {

// Below this curvature the clamp-recompute step (v = omega / kappa) would divide
// by a near-zero denominator. At |kappa| this small, kappa*v is already far under
// any reasonable max_omega so the clamp branch never actually fires in practice --
// this guards the division defensively rather than tuning any real behavior.
constexpr double kKappaEps = 1e-9;

core::Point closest_point_on_segment(const core::Point& p, const core::Point& a,
                                     const core::Point& b) {
  double dx = b.x - a.x, dy = b.y - a.y;
  double seg_len_sq = dx * dx + dy * dy;
  if (seg_len_sq < 1e-12) return a;
  double t = std::max(0.0, std::min(1.0, ((p.x - a.x) * dx + (p.y - a.y) * dy) / seg_len_sq));
  return core::Point{a.x + t * dx, a.y + t * dy};
}

double sq_dist(const core::Point& a, const core::Point& b) {
  double dx = a.x - b.x, dy = a.y - b.y;
  return dx * dx + dy * dy;
}

// Forward-most intersection of the robot-centered lookahead circle with segment
// a->b, as a parameter t in [0, 1], or nullopt if the segment stays entirely
// inside/outside the circle.
//
// Solves |a + t*(b-a) - p|^2 = radius^2 (Coulter 1992 sec. 3: circle-line
// intersection) and keeps the larger root in range -- the exit point, i.e.
// further along the path -- so the chosen point always leads the robot forward
// rather than back toward where it entered the circle.
std::optional<double> segment_circle_forward_t(const core::Point& p, const core::Point& a,
                                               const core::Point& b, double radius) {
  double dx = b.x - a.x, dy = b.y - a.y;
  double fx = a.x - p.x, fy = a.y - p.y;
  double aa = dx * dx + dy * dy;
  if (aa < 1e-12) return std::nullopt;
  double bb = 2.0 * (fx * dx + fy * dy);
  double cc = fx * fx + fy * fy - radius * radius;
  double disc = bb * bb - 4.0 * aa * cc;
  if (disc < 0.0) return std::nullopt;
  double sq = std::sqrt(disc);
  for (double t : {(-bb + sq) / (2.0 * aa), (-bb - sq) / (2.0 * aa)}) {
    if (t >= 0.0 && t <= 1.0) return t;
  }
  return std::nullopt;
}

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

  if (path.size() >= 2) {
    int best_index = progress_index_;
    double best_sq_dist = std::numeric_limits<double>::infinity();
    for (int i = progress_index_; i < static_cast<int>(path.size()) - 1; ++i) {
      core::Point closest = closest_point_on_segment(robot_xy, path[static_cast<size_t>(i)],
                                                      path[static_cast<size_t>(i) + 1]);
      double d = sq_dist(robot_xy, closest);
      // <=, not <: consecutive segments share their joint endpoint, so a robot
      // sitting exactly at a corner ties every segment ending/starting there.
      // Preferring the later (more forward) segment on a tie keeps progress
      // advancing through the corner instead of latching onto the segment just
      // traveled.
      if (d <= best_sq_dist) {
        best_sq_dist = d;
        best_index = i;
      }
    }
    progress_index_ = best_index;
  }

  const double lookahead_distance = params_.get_float("lookahead_distance");
  core::Point target = path.empty() ? robot_xy : path.back();
  for (int i = progress_index_; !path.empty() && i < static_cast<int>(path.size()) - 1; ++i) {
    auto t = segment_circle_forward_t(robot_xy, path[static_cast<size_t>(i)],
                                      path[static_cast<size_t>(i) + 1], lookahead_distance);
    if (t) {
      const core::Point& a = path[static_cast<size_t>(i)];
      const core::Point& b = path[static_cast<size_t>(i) + 1];
      target = core::Point{a.x + *t * (b.x - a.x), a.y + *t * (b.y - a.y)};
      break;
    }
  }

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
