#include "navigation/local_planning/band/elastic_bands.hpp"

#include <algorithm>
#include <cmath>

#include "navigation/local_planning/band/band.hpp"
#include "navigation/local_planning/geometry.hpp"

namespace navigation::local_planning {

namespace {

// Below this squared norm a unit-vector division is unstable, so the term is
// skipped instead -- same 1e-12 threshold as geometry.hpp's segment-degeneracy
// guard, applied uniformly to every division-by-distance in this file
// (contraction neighbor terms, repulsion terms, tangent removal).
constexpr double kEpsSq = 1e-12;

}  // namespace

ElasticBandsPlanner::ElasticBandsPlanner(core::ParamSet params)
    : core::ObstacleLocalPlanner(std::move(params)),
      k_contraction_(params_.get_float("k_contraction")),
      k_repulsion_(params_.get_float("k_repulsion")),
      rho_max_(params_.get_float("rho_max")),
      rho_influence_(params_.get_float("rho_influence")),
      rho_min_(params_.get_float("rho_min")),
      step_size_(params_.get_float("step_size")),
      deform_iterations_(params_.get_int("deform_iterations")),
      repair_iterations_(params_.get_int("repair_iterations")),
      repair_step_(params_.get_float("repair_step")),
      overlap_factor_(params_.get_float("overlap_factor")),
      max_bubbles_(params_.get_int("max_bubbles")),
      bubble_spacing_(params_.get_float("bubble_spacing")),
      lookahead_distance_(params_.get_float("lookahead_distance")),
      heading_gain_(params_.get_float("heading_gain")),
      v_max_(params_.get_float("v_max")),
      omega_max_(params_.get_float("omega_max")) {}

double ElasticBandsPlanner::clearance(core::ObstacleQuery& space, const core::Point& p) const {
  return std::min(space.distance_to_nearest(p), rho_max_);
}

void ElasticBandsPlanner::initialize(core::ObstacleQuery& space, const core::LocalTask& task,
                                     const core::Point& robot_xy) {
  std::vector<core::Point> resampled =
      resample_polyline(task.reference_path, bubble_spacing_);
  core::Point goal_xy{task.goal.x, task.goal.y};

  centers_.clear();
  centers_.reserve(resampled.size() + 2);
  centers_.push_back(robot_xy);
  centers_.insert(centers_.end(), resampled.begin(), resampled.end());
  centers_.push_back(goal_xy);

  radii_.clear();
  radii_.reserve(centers_.size());
  for (const core::Point& c : centers_) radii_.push_back(clearance(space, c));

  for (int i = 0; i < repair_iterations_; ++i) deform_once(space);
}

void ElasticBandsPlanner::deform_once(core::ObstacleQuery& space) {
  const size_t n = centers_.size();
  if (n < 3) return;
  std::vector<core::Point> deltas(n, core::Point{0.0, 0.0});

  for (size_t i = 1; i + 1 < n; ++i) {
    const double cx = centers_[i].x, cy = centers_[i].y;

    // Internal contraction: pulls bubble i toward both neighbors (keeps the
    // band taut). Tangential component kept -- it is what equalizes bubble
    // spacing along the band.
    double fcx = 0.0, fcy = 0.0;
    for (size_t j : {i - 1, i + 1}) {
      const double dx = centers_[j].x - cx, dy = centers_[j].y - cy;
      const double d_sq = dx * dx + dy * dy;
      if (d_sq < kEpsSq) continue;
      const double d = std::sqrt(d_sq);
      fcx += dx / d;
      fcy += dy / d;
    }
    fcx *= k_contraction_;
    fcy *= k_contraction_;

    // External repulsion: summed over every occupied cell within
    // rho_influence, in occupied_within's row/col-ascending order --
    // summation (not just the nearest cell) so a bubble embedded inside a
    // multi-cell obstacle still feels a net push toward its nearest edge
    // rather than one that might point inward from a single sampled cell.
    double frx = 0.0, fry = 0.0;
    for (const core::Point& o : space.occupied_within(core::Point{cx, cy}, rho_influence_)) {
      const double dx = cx - o.x, dy = cy - o.y;
      const double d_sq = dx * dx + dy * dy;
      if (d_sq < kEpsSq) continue;
      const double d = std::sqrt(d_sq);
      frx += (rho_influence_ - d) * dx / d;
      fry += (rho_influence_ - d) * dy / d;
    }
    frx *= k_repulsion_;
    fry *= k_repulsion_;

    // Tangent-component removal applies to the repulsive force only (Quinlan
    // & Khatib 1993): its tangential part would slide bubbles along the band
    // and bunch them together, whereas the contraction force's tangential
    // part is exactly what equalizes spacing above.
    double tx = centers_[i + 1].x - centers_[i - 1].x;
    double ty = centers_[i + 1].y - centers_[i - 1].y;
    const double t_sq = tx * tx + ty * ty;
    if (t_sq >= kEpsSq) {
      const double t_norm = std::sqrt(t_sq);
      tx /= t_norm;
      ty /= t_norm;
      const double proj = frx * tx + fry * ty;
      frx -= proj * tx;
      fry -= proj * ty;
    }

    double dx_step = step_size_ * (fcx + frx);
    double dy_step = step_size_ * (fcy + fry);
    // Displacement cap: floored at repair_step (not just 0.5*rho) so a bubble
    // that starts with rho=0 (inside an obstacle) still moves instead of
    // being permanently clamped to zero step.
    const double limit = std::max(0.5 * radii_[i], repair_step_);
    const double mag_sq = dx_step * dx_step + dy_step * dy_step;
    if (mag_sq > limit * limit) {
      const double scale = limit / std::sqrt(mag_sq);
      dx_step *= scale;
      dy_step *= scale;
    }
    deltas[i] = core::Point{dx_step, dy_step};
  }

  for (size_t i = 1; i + 1 < n; ++i) {
    const core::Point new_center{centers_[i].x + deltas[i].x, centers_[i].y + deltas[i].y};
    centers_[i] = new_center;
    radii_[i] = clearance(space, new_center);
  }
}

bool ElasticBandsPlanner::maintain(core::ObstacleQuery& space) {
  size_t i = 0;
  while (i + 1 < centers_.size()) {
    if (i + 2 < centers_.size()) {
      const double gap = std::sqrt(sq_dist(centers_[i], centers_[i + 2]));
      if (gap <= overlap_factor_ * (radii_[i] + radii_[i + 2])) {
        centers_.erase(centers_.begin() + static_cast<long>(i) + 1);
        radii_.erase(radii_.begin() + static_cast<long>(i) + 1);
        continue;
      }
    }
    const double gap = std::sqrt(sq_dist(centers_[i], centers_[i + 1]));
    if (gap > overlap_factor_ * (radii_[i] + radii_[i + 1])) {
      if (static_cast<int>(centers_.size()) >= max_bubbles_) return false;
      const core::Point mid{(centers_[i].x + centers_[i + 1].x) / 2.0,
                            (centers_[i].y + centers_[i + 1].y) / 2.0};
      const double rho_mid = clearance(space, mid);
      if (rho_mid < rho_min_) return false;
      centers_.insert(centers_.begin() + static_cast<long>(i) + 1, mid);
      radii_.insert(radii_.begin() + static_cast<long>(i) + 1, rho_mid);
      continue;
    }
    ++i;
  }
  for (size_t k = 1; k + 1 < radii_.size(); ++k) {
    if (radii_[k] < rho_min_) return false;
  }
  return true;
}

void ElasticBandsPlanner::emit_band(core::TraceRecorder& recorder, double broken) const {
  std::vector<std::vector<double>> band;
  band.reserve(centers_.size());
  for (size_t i = 0; i < centers_.size(); ++i) {
    band.push_back({centers_[i].x, centers_[i].y, radii_[i]});
  }
  recorder.band_updated(band, {{"iterations", static_cast<double>(deform_iterations_)},
                               {"bubbles", static_cast<double>(centers_.size())},
                               {"broken", broken}});
}

core::VelocityCommand ElasticBandsPlanner::on_broken(core::TraceRecorder* recorder) {
  // The band is serialized exactly as it stood at the moment maintenance
  // broke it (not rolled back to the last valid band) -- a single fixed rule
  // shared with the byte-identical parity fixture. The internal band is then
  // discarded so the next tick re-initializes and repairs from scratch.
  if (recorder) emit_band(*recorder, 1.0);
  centers_.clear();
  radii_.clear();
  return core::VelocityCommand{0.0, 0.0};
}

core::VelocityCommand ElasticBandsPlanner::compute_command(core::ObstacleQuery& space,
                                                            const core::RobotState& state,
                                                            const core::LocalTask& task, double dt,
                                                            core::TraceRecorder* recorder) {
  (void)dt;  // command extraction is a geometric law; no integration inside a single tick.
  const double x = state.pose.x, y = state.pose.y, theta = state.pose.theta;
  const core::Point robot_xy{x, y};

  if (centers_.empty()) {
    initialize(space, task, robot_xy);
    if (!maintain(space)) return on_broken(recorder);
  }

  // Front pruning: drop bubbles the robot has already advanced past (its
  // position sits within the next bubble's clearance disc), then anchor c_0
  // to the executed pose -- the two endpoints are never moved by
  // deformation, only re-pinned here and at initialization.
  while (centers_.size() > 2 && std::sqrt(sq_dist(robot_xy, centers_[1])) <= radii_[1]) {
    centers_.erase(centers_.begin());
    radii_.erase(radii_.begin());
  }
  centers_[0] = robot_xy;
  radii_[0] = clearance(space, robot_xy);

  for (int i = 0; i < deform_iterations_; ++i) deform_once(space);
  if (!maintain(space)) return on_broken(recorder);

  if (recorder) emit_band(*recorder, 0.0);

  // Command extraction: a lookahead point on the deformed band's own
  // polyline, tracked with plain proportional heading control. The band is
  // re-deformed whole every tick (not a progress-indexed path), so the
  // tracking family's lookahead-circle intersection doesn't apply here.
  const core::Point target = point_at_arclength(centers_, lookahead_distance_);
  const double alpha = wrap_to_pi(std::atan2(target.y - y, target.x - x) - theta);
  const double v = v_max_ * std::max(std::cos(alpha), 0.0);
  const double omega = std::max(-omega_max_, std::min(omega_max_, heading_gain_ * alpha));
  return core::VelocityCommand{v, omega};
}

}  // namespace navigation::local_planning
