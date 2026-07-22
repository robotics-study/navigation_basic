#include "navigation/local_planning/band/teb.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>

#include "navigation/local_planning/band/band.hpp"
#include "navigation/local_planning/geometry.hpp"

namespace navigation::local_planning {

namespace {

// Below this squared norm a unit-vector division (or segment-direction
// atan2) is unstable, so the term/direction is skipped instead -- same
// 1e-12 threshold as geometry.hpp's segment-degeneracy guard and
// elastic_bands.cpp's kEpsSq, applied uniformly to every division-by-length
// in this file.
constexpr double kEpsSq = 1e-12;

double clamp(double value, double bound) { return std::max(-bound, std::min(bound, value)); }

// Raw (unclamped) projection parameter of `probe` onto segment a->b. Unlike
// closest_point_on_segment (which clamps to [0, 1] and returns a point), warm
// start needs to know whether the robot has advanced *past* the band's own
// first edge -- t >= 1 -- so the parameter itself, not the clamped point, is
// what the caller needs.
double segment_t(const core::Point& probe, const core::Point& a, const core::Point& b) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double seg_len_sq = dx * dx + dy * dy;
  if (seg_len_sq < kEpsSq) return 0.0;
  return ((probe.x - a.x) * dx + (probe.y - a.y) * dy) / seg_len_sq;
}

// Nearest point to `p` over every segment of `points` (no monotonic
// constraint, unlike advance_progress_index -- this is a per-tick anchor
// lookup, not a forward-only progress cursor). Strict `<` keeps the first
// tie, mirroring nearest_occupied's determinism convention.
core::Point closest_point_on_polyline(const std::vector<core::Point>& points, const core::Point& p) {
  core::Point best = points.front();
  double best_sq = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    core::Point c = closest_point_on_segment(p, points[i], points[i + 1]);
    double d = sq_dist(p, c);
    if (d < best_sq) {
      best_sq = d;
      best = c;
    }
  }
  return best;
}

}  // namespace

TebPlanner::TebPlanner(core::ParamSet params)
    : core::ObstacleLocalPlanner(std::move(params)),
      w_path_(params_.get_float("w_path")),
      w_obstacle_(params_.get_float("w_obstacle")),
      w_velocity_(params_.get_float("w_velocity")),
      w_acceleration_(params_.get_float("w_acceleration")),
      w_time_(params_.get_float("w_time")),
      w_kinematics_(params_.get_float("w_kinematics")),
      v_max_(params_.get_float("v_max")),
      omega_max_(params_.get_float("omega_max")),
      a_max_(params_.get_float("a_max")),
      min_obstacle_dist_(params_.get_float("min_obstacle_dist")),
      dt_ref_(params_.get_float("dt_ref")),
      dt_min_(params_.get_float("dt_min")),
      horizon_(params_.get_float("horizon")),
      iterations_(params_.get_int("iterations")),
      step_alpha_(params_.get_float("step_alpha")),
      max_step_xy_(params_.get_float("max_step_xy")),
      max_step_theta_(params_.get_float("max_step_theta")),
      max_step_dt_(params_.get_float("max_step_dt")),
      max_poses_(params_.get_int("max_poses")),
      reinit_distance_(params_.get_float("reinit_distance")) {}

std::pair<core::Pose, std::vector<core::Point>> TebPlanner::clip(const std::vector<core::Point>& path,
                                                                  int start_index,
                                                                  const core::Point& origin,
                                                                  double goal_theta) const {
  std::vector<core::Point> points{origin};
  double remaining = horizon_;
  int idx = start_index;
  core::Point prev = origin;
  while (idx + 1 < static_cast<int>(path.size())) {
    const core::Point& nxt = path[static_cast<size_t>(idx) + 1];
    const double seg_len = std::sqrt(sq_dist(prev, nxt));
    if (remaining <= seg_len) {
      double gx, gy;
      if (seg_len < 1e-12) {
        gx = nxt.x;
        gy = nxt.y;
      } else {
        const double t = remaining / seg_len;
        gx = prev.x + t * (nxt.x - prev.x);
        gy = prev.y + t * (nxt.y - prev.y);
      }
      const double theta = std::atan2(nxt.y - prev.y, nxt.x - prev.x);
      points.push_back(core::Point{gx, gy});
      return {core::Pose{gx, gy, theta}, points};
    }
    remaining -= seg_len;
    points.push_back(nxt);
    prev = nxt;
    ++idx;
  }
  return {core::Pose{path.back().x, path.back().y, goal_theta}, points};
}

std::pair<std::vector<core::Pose>, std::vector<double>> TebPlanner::init_band(
    const std::vector<core::Point>& clip_points, double local_goal_theta) const {
  const double spacing = v_max_ * dt_ref_;
  std::vector<core::Point> pts = resample_polyline(clip_points, spacing);
  std::vector<core::Pose> poses;
  poses.reserve(pts.size());
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    const double dx = pts[i + 1].x - pts[i].x;
    const double dy = pts[i + 1].y - pts[i].y;
    const double theta = (dx * dx + dy * dy >= kEpsSq) ? std::atan2(dy, dx) : 0.0;
    poses.push_back(core::Pose{pts[i].x, pts[i].y, theta});
  }
  poses.push_back(core::Pose{pts.back().x, pts.back().y, local_goal_theta});

  std::vector<double> dts;
  dts.reserve(poses.size() - 1);
  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    const double ell = std::sqrt(sq_dist(core::Point{poses[i].x, poses[i].y},
                                         core::Point{poses[i + 1].x, poses[i + 1].y}));
    dts.push_back(ell / (0.5 * v_max_));
  }
  return {poses, dts};
}

void TebPlanner::resize(std::vector<core::Pose>& poses, std::vector<double>& dts) const {
  size_t i = 0;
  while (i < dts.size()) {
    if (dts[i] > 1.5 * dt_ref_ && static_cast<int>(poses.size()) < max_poses_) {
      const core::Pose& p0 = poses[i];
      const core::Pose& p1 = poses[i + 1];
      const double mx = 0.5 * (p0.x + p1.x);
      const double my = 0.5 * (p0.y + p1.y);
      const double mth = wrap_to_pi(p0.theta + 0.5 * wrap_to_pi(p1.theta - p0.theta));
      poses.insert(poses.begin() + static_cast<long>(i) + 1, core::Pose{mx, my, mth});
      const double half = 0.5 * dts[i];
      dts[i] = half;
      dts.insert(dts.begin() + static_cast<long>(i) + 1, half);
      continue;
    }
    if (dts[i] < 0.5 * dt_ref_ && poses.size() > 3 && i + 1 < dts.size()) {
      poses.erase(poses.begin() + static_cast<long>(i) + 1);
      dts[i] += dts[i + 1];
      dts.erase(dts.begin() + static_cast<long>(i) + 1);
      continue;
    }
    ++i;
  }
}

void TebPlanner::gradient_step(std::vector<core::Pose>& poses, std::vector<double>& dts,
                               const std::vector<core::Point>& anchors,
                               core::ObstacleQuery& space) const {
  const size_t n = poses.size();
  std::vector<double> gx(n, 0.0), gy(n, 0.0), gth(n, 0.0);
  std::vector<double> gdt(n - 1, 0.0);

  // Per-segment quantities shared by the velocity/acceleration/kinematics
  // terms, cached once per iteration in index order.
  std::vector<double> dxs(n - 1, 0.0), dys(n - 1, 0.0), ell(n - 1, 0.0);
  std::vector<bool> has_pos(n - 1, false);
  std::vector<double> v(n - 1, 0.0), omega(n - 1, 0.0);
  for (size_t i = 0; i + 1 < n; ++i) {
    const double dx = poses[i + 1].x - poses[i].x;
    const double dy = poses[i + 1].y - poses[i].y;
    dxs[i] = dx;
    dys[i] = dy;
    const double d_sq = dx * dx + dy * dy;
    if (d_sq < kEpsSq) {
      ell[i] = 0.0;
      v[i] = 0.0;
      has_pos[i] = false;
    } else {
      ell[i] = std::sqrt(d_sq);
      v[i] = ell[i] / dts[i];
      has_pos[i] = true;
    }
    omega[i] = wrap_to_pi(poses[i + 1].theta - poses[i].theta) / dts[i];
  }

  // (a) reference-path attraction.
  for (size_t i = 1; i + 1 < n; ++i) {
    const double c = 2.0 * w_path_;
    gx[i] += c * (poses[i].x - anchors[i].x);
    gy[i] += c * (poses[i].y - anchors[i].y);
  }

  // (b) obstacle clearance -- continuous distance to the nearest occupied
  // cell center (not the quantized distance_to_nearest EDT), recomputed
  // every iteration since p_i moves.
  for (size_t i = 1; i + 1 < n; ++i) {
    const core::Point p{poses[i].x, poses[i].y};
    auto [o, d_tilde] = nearest_occupied(space, p, min_obstacle_dist_);
    if (!o) continue;
    const double g_i = min_obstacle_dist_ - d_tilde;
    if (g_i <= 0.0) continue;
    if (d_tilde * d_tilde < kEpsSq) continue;
    const double c = -2.0 * w_obstacle_ * g_i / d_tilde;
    gx[i] += c * (p.x - o->x);
    gy[i] += c * (p.y - o->y);
  }

  // (c) velocity limits.
  for (size_t i = 0; i + 1 < n; ++i) {
    const double e_v = std::max(0.0, v[i] - v_max_);
    if (e_v > 0.0) {
      const double c = 2.0 * w_velocity_ * e_v;
      if (has_pos[i]) {
        const double coeff = c / (ell[i] * dts[i]);
        gx[i] -= coeff * dxs[i];
        gy[i] -= coeff * dys[i];
        gx[i + 1] += coeff * dxs[i];
        gy[i + 1] += coeff * dys[i];
      }
      gdt[i] += c * (-v[i] / dts[i]);
    }
    const double e_w = std::max(0.0, std::fabs(omega[i]) - omega_max_);
    if (e_w > 0.0) {
      const double sign = omega[i] > 0.0 ? 1.0 : (omega[i] < 0.0 ? -1.0 : 0.0);
      const double c = 2.0 * w_velocity_ * e_w * sign;
      gth[i] -= c / dts[i];
      gth[i + 1] += c / dts[i];
      gdt[i] += c * (-omega[i] / dts[i]);
    }
  }

  // (d) translational acceleration limits.
  for (size_t i = 0; i + 2 < n; ++i) {
    const double denom = 0.5 * (dts[i] + dts[i + 1]);
    const double a_i = (v[i + 1] - v[i]) / denom;
    const double e_a = std::max(0.0, std::fabs(a_i) - a_max_);
    if (e_a <= 0.0) continue;
    const double sign_a = a_i > 0.0 ? 1.0 : (a_i < 0.0 ? -1.0 : 0.0);
    const double c = 2.0 * w_acceleration_ * e_a * sign_a;
    const double dv1 = c / denom;
    const double dv0 = -c / denom;
    if (has_pos[i]) {
      const double coeff0 = dv0 / (ell[i] * dts[i]);
      gx[i] -= coeff0 * dxs[i];
      gy[i] -= coeff0 * dys[i];
      gx[i + 1] += coeff0 * dxs[i];
      gy[i + 1] += coeff0 * dys[i];
    }
    gdt[i] += dv0 * (-v[i] / dts[i]);
    if (has_pos[i + 1]) {
      const double coeff1 = dv1 / (ell[i + 1] * dts[i + 1]);
      gx[i + 1] -= coeff1 * dxs[i + 1];
      gy[i + 1] -= coeff1 * dys[i + 1];
      gx[i + 2] += coeff1 * dxs[i + 1];
      gy[i + 2] += coeff1 * dys[i + 1];
    }
    gdt[i + 1] += dv1 * (-v[i + 1] / dts[i + 1]);
    gdt[i] += c * (-a_i * 0.5 / denom);
    gdt[i + 1] += c * (-a_i * 0.5 / denom);
  }

  // (f) nonholonomic two-pose-arc kinematics (Rösmann 2012).
  for (size_t i = 0; i + 1 < n; ++i) {
    const double th_i = poses[i].theta, th_i1 = poses[i + 1].theta;
    const double cos_sum = std::cos(th_i) + std::cos(th_i1);
    const double sin_sum = std::sin(th_i) + std::sin(th_i1);
    const double h_i = cos_sum * dys[i] - sin_sum * dxs[i];
    const double c = 2.0 * w_kinematics_ * h_i;
    gx[i] += c * sin_sum;
    gx[i + 1] -= c * sin_sum;
    gy[i] -= c * cos_sum;
    gy[i + 1] += c * cos_sum;
    gth[i] += c * (-std::sin(th_i) * dys[i] - std::cos(th_i) * dxs[i]);
    gth[i + 1] += c * (-std::sin(th_i1) * dys[i] - std::cos(th_i1) * dxs[i]);
  }

  // (e) time optimality.
  for (size_t i = 0; i + 1 < n; ++i) gdt[i] += w_time_;

  for (size_t i = 1; i + 1 < n; ++i) {
    double x_i = poses[i].x - clamp(step_alpha_ * gx[i], max_step_xy_);
    double y_i = poses[i].y - clamp(step_alpha_ * gy[i], max_step_xy_);
    double th_i = wrap_to_pi(poses[i].theta - clamp(step_alpha_ * gth[i], max_step_theta_));
    poses[i] = core::Pose{x_i, y_i, th_i};
  }
  for (size_t i = 0; i + 1 < n; ++i) {
    const double step = clamp(step_alpha_ * gdt[i], max_step_dt_);
    dts[i] = std::max(dt_min_, dts[i] - step);
  }
}

double TebPlanner::total_cost(const std::vector<core::Pose>& poses, const std::vector<double>& dts,
                              const std::vector<core::Point>& anchors,
                              core::ObstacleQuery& space) const {
  const size_t n = poses.size();
  double total = 0.0;
  for (size_t i = 1; i + 1 < n; ++i) {
    const double dx = poses[i].x - anchors[i].x, dy = poses[i].y - anchors[i].y;
    total += w_path_ * (dx * dx + dy * dy);
  }
  for (size_t i = 1; i + 1 < n; ++i) {
    const core::Point p{poses[i].x, poses[i].y};
    auto [o, d_tilde] = nearest_occupied(space, p, min_obstacle_dist_);
    if (o) {
      const double g_i = std::max(0.0, min_obstacle_dist_ - d_tilde);
      total += w_obstacle_ * g_i * g_i;
    }
  }
  std::vector<double> v(n - 1, 0.0);
  for (size_t i = 0; i + 1 < n; ++i) {
    const double dx = poses[i + 1].x - poses[i].x, dy = poses[i + 1].y - poses[i].y;
    const double d_sq = dx * dx + dy * dy;
    v[i] = (d_sq >= kEpsSq) ? std::sqrt(d_sq) / dts[i] : 0.0;
    const double omega_i = wrap_to_pi(poses[i + 1].theta - poses[i].theta) / dts[i];
    const double e_v = std::max(0.0, v[i] - v_max_);
    const double e_w = std::max(0.0, std::fabs(omega_i) - omega_max_);
    total += w_velocity_ * (e_v * e_v + e_w * e_w);
  }
  for (size_t i = 0; i + 2 < n; ++i) {
    const double denom = 0.5 * (dts[i] + dts[i + 1]);
    const double a_i = (v[i + 1] - v[i]) / denom;
    const double e_a = std::max(0.0, std::fabs(a_i) - a_max_);
    total += w_acceleration_ * e_a * e_a;
  }
  for (size_t i = 0; i + 1 < n; ++i) {
    const double th_i = poses[i].theta, th_i1 = poses[i + 1].theta;
    const double dx = poses[i + 1].x - poses[i].x, dy = poses[i + 1].y - poses[i].y;
    const double h_i = (std::cos(th_i) + std::cos(th_i1)) * dy - (std::sin(th_i) + std::sin(th_i1)) * dx;
    total += w_kinematics_ * h_i * h_i;
  }
  total += w_time_ * std::accumulate(dts.begin(), dts.end(), 0.0);
  return total;
}

void TebPlanner::emit_band(core::TraceRecorder& recorder, const std::vector<core::Pose>& poses,
                           const std::vector<double>& dts, int iterations, double total_cost) const {
  std::vector<std::vector<double>> band;
  band.reserve(poses.size());
  band.push_back({poses[0].x, poses[0].y, poses[0].theta, 0.0});
  for (size_t i = 1; i < poses.size(); ++i) {
    band.push_back({poses[i].x, poses[i].y, poses[i].theta, dts[i - 1]});
  }
  recorder.band_updated(band, {{"iterations", static_cast<double>(iterations)},
                               {"poses", static_cast<double>(poses.size())},
                               {"total_cost", total_cost},
                               {"horizon_time", std::accumulate(dts.begin(), dts.end(), 0.0)}});
}

core::VelocityCommand TebPlanner::compute_command(core::ObstacleQuery& space,
                                                   const core::RobotState& state,
                                                   const core::LocalTask& task, double dt,
                                                   core::TraceRecorder* recorder) {
  (void)dt;  // command extraction reads only the optimized band; dt is the caller's tick length.
  const double x = state.pose.x, y = state.pose.y, theta = state.pose.theta;
  const core::Point robot_xy{x, y};
  const std::vector<core::Point>& path = task.reference_path;

  progress_index_ = advance_progress_index(path, robot_xy, progress_index_);
  const core::Point origin = closest_point_on_segment(
      robot_xy, path[static_cast<size_t>(progress_index_)], path[static_cast<size_t>(progress_index_) + 1]);
  auto [local_goal, clip_points] = clip(path, progress_index_, origin, task.goal.theta);

  const bool need_reinit =
      poses_.empty() ||
      std::sqrt(sq_dist(core::Point{poses_.back().x, poses_.back().y},
                        core::Point{local_goal.x, local_goal.y})) > reinit_distance_;

  std::vector<core::Pose> poses;
  std::vector<double> dts;
  if (need_reinit) {
    std::tie(poses, dts) = init_band(clip_points, local_goal.theta);
  } else {
    poses = std::move(poses_);
    dts = std::move(dts_);
    // Warm start: the band's own first edge, not the reference path's
    // progress index above -- the band trails the robot at whatever pace
    // optimization left it.
    while (poses.size() > 2 &&
          segment_t(robot_xy, core::Point{poses[0].x, poses[0].y}, core::Point{poses[1].x, poses[1].y}) >=
              1.0) {
      poses.erase(poses.begin());
      dts.erase(dts.begin());
    }
  }

  poses[0] = core::Pose{x, y, theta};
  poses.back() = local_goal;

  if (poses.size() < 3) {
    // Degenerate: the robot is essentially at the local goal already, with no
    // internal pose to optimize -- skip the solver and steer straight at the
    // goal with proportional heading control (same structure as Elastic
    // Bands' command extraction). The gain is omega_max/pi rather than a
    // dedicated heading_gain parameter (TEB declares none): alpha never
    // exceeds pi in magnitude, so this keeps the clamp a structural no-op
    // instead of inventing an unvalidated constant.
    poses_ = std::move(poses);
    dts_ = std::move(dts);
    if (recorder) emit_band(*recorder, poses_, dts_, 0, 0.0);
    const double alpha = wrap_to_pi(std::atan2(local_goal.y - y, local_goal.x - x) - theta);
    const double v_cmd = v_max_ * std::max(std::cos(alpha), 0.0);
    const double omega_cmd = clamp((omega_max_ / M_PI) * alpha, omega_max_);
    return core::VelocityCommand{v_cmd, omega_cmd};
  }

  resize(poses, dts);

  // Anchors fixed for the whole tick: the nearest point on the clipped
  // reference path to each internal pose's *initial* position (before this
  // tick's optimization moves it) -- a moving target would make the
  // path-attraction term chase the pose it's supposed to pull.
  std::vector<core::Point> anchors(poses.size(), core::Point{0.0, 0.0});
  for (size_t i = 1; i + 1 < poses.size(); ++i) {
    anchors[i] = closest_point_on_polyline(clip_points, core::Point{poses[i].x, poses[i].y});
  }

  for (int k = 0; k < iterations_; ++k) gradient_step(poses, dts, anchors, space);

  poses_ = poses;
  dts_ = dts;

  if (recorder) {
    const double cost = total_cost(poses, dts, anchors, space);
    emit_band(*recorder, poses, dts, iterations_, cost);
  }

  const double dx0 = poses[1].x - poses[0].x;
  const double dy0 = poses[1].y - poses[0].y;
  const double ell0 = std::sqrt(dx0 * dx0 + dy0 * dy0);
  const double sigma = (dx0 * std::cos(theta) + dy0 * std::sin(theta) >= 0.0) ? 1.0 : -1.0;
  const double v_cmd = clamp(sigma * ell0 / dts[0], v_max_);
  const double omega_cmd = clamp(wrap_to_pi(poses[1].theta - poses[0].theta) / dts[0], omega_max_);
  return core::VelocityCommand{v_cmd, omega_cmd};
}

}  // namespace navigation::local_planning
