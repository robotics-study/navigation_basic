#include "navigation/local_planning/tracking/regulated_pure_pursuit.hpp"

#include <algorithm>
#include <cmath>

#include "navigation/local_planning/geometry.hpp"
#include "navigation/local_planning/tracking/path.hpp"

namespace navigation::local_planning {

namespace {

// Below this curvature, both the "no curvature regulation" branch and the
// clamp-recompute step (v = omega / kappa) would be operating on a near-zero
// denominator. At |kappa| this small the arc is effectively straight, so
// skipping regulation and guarding the division are the same threshold.
// Identical value to pure_pursuit.cpp's kKappaEps -- same reasoning, defined
// independently per this file's own-computation design.
constexpr double kKappaEps = 1e-9;

// Guard band (radians) around alpha = 0 and alpha = +-pi for the lookahead
// collision-check arc length: both neighborhoods make L_d*alpha/sin(alpha)
// degenerate, so both fall back to L_d directly. Wide enough (~3 degrees) to
// cover real near-antipodal targets, not just exact floating-point equality
// -- see the WHY comment at the call site for why alpha actually reaches
// +-pi in practice.
constexpr double kArcAlphaMargin = 0.05;

// Pose reached after arc length s along the constant-curvature arc kappa
// starting at pose -- closed-form circular-arc propagation (same family as
// the simulator's integrate_unicycle, parametrized by arc length instead of
// time since the collision check walks distance, not ticks).
core::Pose propagate_arc(const core::Pose& pose, double kappa, double s) {
  if (std::fabs(kappa) < kKappaEps) {
    return core::Pose{pose.x + s * std::cos(pose.theta), pose.y + s * std::sin(pose.theta),
                      pose.theta};
  }
  double new_theta = pose.theta + kappa * s;
  double x2 = pose.x + (std::sin(new_theta) - std::sin(pose.theta)) / kappa;
  double y2 = pose.y - (std::cos(new_theta) - std::cos(pose.theta)) / kappa;
  return core::Pose{x2, y2, wrap_to_pi(new_theta)};
}

}  // namespace

core::VelocityCommand RegulatedPurePursuitPlanner::compute_command(
    core::ObstacleQuery& space, const core::RobotState& state, const core::LocalTask& task,
    double dt, core::TraceRecorder* recorder) {
  (void)dt;  // geometric + regulation law; no integration inside a single tick's command.
  const auto& path = task.reference_path;
  const double x = state.pose.x, y = state.pose.y, theta = state.pose.theta;
  const core::Point robot_xy{x, y};

  // 1. Adaptive lookahead (Macenski 2023 sec. 3.1; lineage Campbell 2007):
  // scales with current speed so a fast robot looks further ahead than a
  // slow one, instead of chasing a fixed-radius point regardless of pace.
  const double lookahead_time = params_.get_float("lookahead_time");
  const double min_lookahead = params_.get_float("min_lookahead");
  const double max_lookahead = params_.get_float("max_lookahead");
  // Clamp order is min(max(.), max_lookahead): if min_lookahead is declared
  // larger than max_lookahead, max_lookahead wins.
  const double lookahead_distance =
      std::min(std::max(lookahead_time * state.v, min_lookahead), max_lookahead);

  // 2. Target point: same geometry as Pure Pursuit, shared via path.hpp.
  progress_index_ = advance_progress_index(path, robot_xy, progress_index_);
  const core::Point target = lookahead_point(path, progress_index_, robot_xy, lookahead_distance);

  // 3. Commanded curvature (Coulter 1992 geometry, independently recomputed
  // here rather than imported -- see the header's class comment).
  const double alpha = wrap_to_pi(std::atan2(target.y - y, target.x - x) - theta);
  const double kappa = 2.0 * std::sin(alpha) / lookahead_distance;

  // 4. Speed regulation: v is the minimum of three independent caps.
  const double max_speed = params_.get_float("max_speed");
  const double slow_radius = params_.get_float("slow_radius");
  const double remaining = std::hypot(task.goal.x - x, task.goal.y - y);
  const double v_goal = max_speed * std::min(1.0, remaining / slow_radius);

  const double regulated_min_radius = params_.get_float("regulated_min_radius");
  double v_curv = max_speed;
  if (std::fabs(kappa) > kKappaEps) {
    const double radius = 1.0 / std::fabs(kappa);
    if (radius < regulated_min_radius) {
      v_curv = max_speed * (radius / regulated_min_radius);
    }
  }

  // Proximity heuristic (Macenski 2023 sec. 3.2): the paper scales by
  // costmap cost, this implementation scales by EDT distance to the nearest
  // obstacle instead -- a simplification (no costmap layer here).
  const double proximity_distance = params_.get_float("proximity_distance");
  const double footprint_radius = params_.get_float("footprint_radius");
  const double clearance = space.distance_to_nearest(core::Point{x, y}) - footprint_radius;
  double v_prox = max_speed;
  if (clearance < proximity_distance) {
    v_prox = max_speed * std::max(clearance, 0.0) / proximity_distance;
  }

  const double min_regulated_speed = params_.get_float("min_regulated_speed");
  // The min_regulated_speed floor only applies to the three caps above --
  // never to the collision stop in step 5, which must be able to reach 0.
  double v = std::max(std::min({v_goal, v_curv, v_prox}), min_regulated_speed);

  // 5. Lookahead collision check (Macenski 2023 sec. 3.3): walk the
  // commanded constant-curvature arc out to the target's arc length and stop
  // outright if it runs into an obstacle, instead of discovering the
  // collision only after already committing to the command.
  // sin(alpha) -> 0 both as alpha -> 0 and as alpha -> +-pi, so the arc
  // length L_d*alpha/sin(alpha) is degenerate at both ends: near alpha=0 it
  // correctly limits to L_d, but near alpha=+-pi the same formula blows up
  // (alpha stays ~pi while sin(alpha) shrinks toward 0), predicting an
  // absurdly long arc instead of a short one. The shared path.hpp
  // lookahead_point() can hand back a target *behind* the robot (alpha ~=
  // +-pi) whenever the robot is tracking a straight segment almost exactly
  // on-line and comes within lookahead_distance of that segment's end -- its
  // forward-circle intersection then falls just past the segment's t=1
  // endpoint, so the search returns the backward intersection instead of
  // continuing on to the next segment. That is a real, observable case (not
  // a hypothetical one), so both degenerate neighborhoods use the same L_d
  // fallback here -- a wide-enough guard band on alpha (not just a
  // division-by-zero guard) rather than only reacting exactly at alpha == 0.
  double arc_length;
  if (std::fabs(alpha) < kArcAlphaMargin || std::fabs(alpha) > M_PI - kArcAlphaMargin) {
    arc_length = lookahead_distance;
  } else {
    arc_length = lookahead_distance * alpha / std::sin(alpha);
  }

  const double collision_check_step = params_.get_float("collision_check_step");
  const core::Footprint footprint{footprint_radius};
  bool blocked = false;
  for (double s = collision_check_step; s <= arc_length; s += collision_check_step) {
    core::Pose pose = propagate_arc(state.pose, kappa, s);
    if (space.is_collision(footprint, pose)) {
      blocked = true;
      break;
    }
  }

  // Trace dict is only ever built inside this lambda, which itself only runs
  // when recorder is non-null -- keeps the hot path allocation-free when
  // tracing is off, same as every other planner's recorder guard.
  auto emit = [&](double blocked_value) {
    if (recorder) {
      recorder->candidate_evaluated(target, kappa,
                                    core::TraceRecorder::EventData{
                                        {"alpha", alpha},
                                        {"lookahead", lookahead_distance},
                                        {"curvature_scale", v_curv / max_speed},
                                        {"proximity_scale", v_prox / max_speed},
                                        {"blocked", blocked_value},
                                    });
    }
  };

  if (blocked) {
    emit(1.0);
    return core::VelocityCommand{0.0, 0.0};
  }

  // 6. Angular velocity, clamped with curvature-preserving recompute (same
  // pattern as pure_pursuit.cpp's kKappaEps clamp).
  const double max_omega = params_.get_float("max_omega");
  const double omega_raw = kappa * v;
  double omega = std::max(-max_omega, std::min(max_omega, omega_raw));
  if (omega != omega_raw && std::fabs(kappa) > kKappaEps) {
    v = omega / kappa;
  }

  emit(0.0);
  return core::VelocityCommand{v, omega};
}

}  // namespace navigation::local_planning
