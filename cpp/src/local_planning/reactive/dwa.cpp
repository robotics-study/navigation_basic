#include "navigation/local_planning/reactive/dwa.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "navigation/local_planning/geometry.hpp"

namespace navigation::local_planning {

namespace {

// Below this |omega| the closed-form arc (division by omega) is numerically
// unstable, so the rollout falls back to the straight-line limit instead --
// the same threshold and reasoning as simulation.cpp's inline 1e-9 guard.
// Duplicated rather than shared: algorithm modules depend on core only, never
// on the simulator.
constexpr double kOmegaEps = 1e-9;

}  // namespace

DwaPlanner::DwaPlanner(core::ParamSet params)
    : core::ObstacleLocalPlanner(std::move(params)),
      max_speed_(params_.get_float("max_speed")),
      min_speed_(params_.get_float("min_speed")),
      max_omega_(params_.get_float("max_omega")),
      accel_(params_.get_float("accel")),
      accel_omega_(params_.get_float("accel_omega")),
      v_samples_(params_.get_int("v_samples")),
      omega_samples_(params_.get_int("omega_samples")),
      sim_time_(params_.get_float("sim_time")),
      sim_steps_(params_.get_int("sim_steps")),
      heading_weight_(params_.get_float("heading_weight")),
      clearance_weight_(params_.get_float("clearance_weight")),
      velocity_weight_(params_.get_float("velocity_weight")),
      clearance_limit_(params_.get_float("clearance_limit")),
      slow_radius_(params_.get_float("slow_radius")),
      footprint_radius_(params_.get_float("footprint_radius")) {}

void DwaPlanner::rollout(const core::Pose& pose, double v, double omega,
                         std::vector<core::Pose>& out) const {
  // Each candidate is scored by holding (v, omega) constant for sim_time,
  // sampled at sim_steps equally spaced instants (Fox 1997's circular-arc
  // trajectory prediction) -- every point is computed directly from the start
  // pose (not chained step-to-step), which is exact for a constant command
  // and matches simulation.cpp's per-tick integrate_unicycle.
  const double x = pose.x, y = pose.y, theta = pose.theta;
  out.clear();  // keeps capacity: no per-candidate allocation in the hot loop
  for (int k = 1; k <= sim_steps_; ++k) {
    const double t = sim_time_ * static_cast<double>(k) / static_cast<double>(sim_steps_);
    if (std::fabs(omega) < kOmegaEps) {
      out.push_back(core::Pose{x + v * t * std::cos(theta), y + v * t * std::sin(theta), theta});
      continue;
    }
    const double new_theta = theta + omega * t;
    const double px = x + (v / omega) * (std::sin(new_theta) - std::sin(theta));
    const double py = y - (v / omega) * (std::cos(new_theta) - std::cos(theta));
    out.push_back(core::Pose{px, py, wrap_to_pi(new_theta)});
  }
}

DwaPlanner::Score DwaPlanner::score(core::ObstacleQuery& space, const core::Footprint& footprint,
                                    const std::vector<core::Pose>& rollout, double v, double omega,
                                    double goal_x, double goal_y) const {
  for (const core::Pose& pose : rollout) {
    if (space.is_collision(footprint, pose)) return Score{};
  }

  // Fox 1997 eq. 14 uses the curvature's true distance to the nearest
  // obstacle; this implementation approximates it conservatively with the
  // minimum clearance sampled along the finite rollout.
  double nearest = std::numeric_limits<double>::infinity();
  for (const core::Pose& pose : rollout) {
    nearest = std::min(nearest, space.distance_to_nearest(core::Point{pose.x, pose.y}));
  }
  const double clearance = std::max(0.0, nearest - footprint_radius_);
  const core::Pose& end = rollout.back();
  // Fox 1997 eq. 15's target-heading term, evaluated at this implementation's
  // rollout endpoint rather than the paper's maximum-deceleration stopping
  // position (a deliberate simplification).
  const double goal_bearing = std::atan2(goal_y - end.y, goal_x - end.x);
  const double heading = 1.0 - std::fabs(wrap_to_pi(goal_bearing - end.theta)) / M_PI;
  const double velocity = v / max_speed_;
  const double clearance_term = std::min(clearance, clearance_limit_) / clearance_limit_;

  Score result;
  result.heading = heading;
  result.clearance = clearance;
  result.velocity = velocity;
  // Fixed (non-batch) normalization: scoring one candidate never depends on
  // the rest of the candidate set, which keeps py/cpp/TS scoring
  // bit-identical and reproducible test-to-test, unlike Fox 1997's per-tick
  // min-max smoothing over the whole candidate batch.
  result.cost = heading_weight_ * heading + clearance_weight_ * clearance_term + velocity_weight_ * velocity;
  // Fox 1997 eq. 14: admissible iff the robot could stop (at accel/
  // accel_omega) before covering `clearance` -- a stopping-distance bound,
  // approximated here with the finite-rollout clearance above.
  result.admissible = v <= std::sqrt(2.0 * clearance * accel_) &&
                      std::fabs(omega) <= std::sqrt(2.0 * clearance * accel_omega_);
  return result;
}

std::pair<double, double> DwaPlanner::decelerate(double v_a, double omega_a, double dt) const {
  const double v_cmd = std::max(0.0, v_a - accel_ * dt);
  const double sign = omega_a > 0.0 ? 1.0 : (omega_a < 0.0 ? -1.0 : 0.0);
  const double omega_cmd = omega_a - sign * std::min(std::fabs(omega_a), accel_omega_ * dt);
  return {v_cmd, omega_cmd};
}

core::VelocityCommand DwaPlanner::compute_command(core::ObstacleQuery& space,
                                                   const core::RobotState& state,
                                                   const core::LocalTask& task, double dt,
                                                   core::TraceRecorder* recorder) {
  const double x = state.pose.x, y = state.pose.y, theta = state.pose.theta;
  const double v_a = state.v, omega_a = state.omega;
  const double goal_x = task.goal.x, goal_y = task.goal.y;

  const double remaining = std::hypot(goal_x - x, goal_y - y);
  // Goal-proximity speed cap: not part of Fox 1997, a practical extension so
  // an episode ends in REACHED instead of orbiting/overshooting the goal at
  // max_speed.
  const double v_max_eff = max_speed_ * std::min(1.0, remaining / slow_radius_);

  const double v_lo = std::max(min_speed_, v_a - accel_ * dt);
  const double v_hi = std::min(v_max_eff, v_a + accel_ * dt);
  const double omega_lo = std::max(-max_omega_, omega_a - accel_omega_ * dt);
  const double omega_hi = std::min(max_omega_, omega_a + accel_omega_ * dt);

  bool have_best = false;
  double best_cost = 0.0, best_v = 0.0, best_omega = 0.0;
  int best_index = -1;

  // Trace buffering: only populated when a recorder is attached (zero cost
  // otherwise) because `selected` cannot be known until every candidate in
  // the tick has been scored.
  struct Buffered {
    core::Point state_xy;
    double cost;
    std::map<std::string, double> data;
    std::vector<std::vector<double>> rollout_xy;
  };
  std::vector<Buffered> buffered;

  if (v_lo <= v_hi && omega_lo <= omega_hi) {
    const core::Footprint footprint{footprint_radius_};
    const double v_step = v_samples_ > 1 ? (v_hi - v_lo) / (v_samples_ - 1) : 0.0;
    const double omega_step = omega_samples_ > 1 ? (omega_hi - omega_lo) / (omega_samples_ - 1) : 0.0;
    int candidate_index = 0;
    // One rollout buffer reused across every candidate in the tick (clear()
    // keeps capacity), so the hot loop performs no per-candidate allocation.
    std::vector<core::Pose> candidate_rollout;
    candidate_rollout.reserve(static_cast<size_t>(sim_steps_));
    // Deterministic uniform grid (never random sampling): fixed traversal
    // order (v outer, omega inner) keeps py/cpp/TS scoring and tie-breaking
    // bit-identical.
    for (int i = 0; i < v_samples_; ++i) {
      const double v = v_lo + v_step * static_cast<double>(i);
      for (int j = 0; j < omega_samples_; ++j) {
        const double omega = omega_lo + omega_step * static_cast<double>(j);
        rollout(core::Pose{x, y, theta}, v, omega, candidate_rollout);
        Score s = score(space, footprint, candidate_rollout, v, omega, goal_x, goal_y);
        if (s.admissible && (!have_best || s.cost > best_cost)) {
          have_best = true;
          best_index = candidate_index;
          best_cost = s.cost;
          best_v = v;
          best_omega = omega;
        }
        if (recorder) {
          const core::Pose& end = candidate_rollout.back();
          std::vector<std::vector<double>> rollout_xy;
          rollout_xy.reserve(candidate_rollout.size());
          for (const core::Pose& pose : candidate_rollout) rollout_xy.push_back({pose.x, pose.y});
          buffered.push_back(Buffered{
              core::Point{end.x, end.y},
              s.cost,
              {{"v", v},
               {"omega", omega},
               {"heading", s.heading},
               {"clearance", s.clearance},
               {"velocity", s.velocity},
               {"admissible", s.admissible ? 1.0 : 0.0},
               {"selected", 0.0}},  // finalized below once selection is known
              std::move(rollout_xy)});
        }
        ++candidate_index;
      }
    }
  }

  double v_cmd, omega_cmd;
  if (have_best) {
    v_cmd = best_v;
    omega_cmd = best_omega;
  } else {
    std::tie(v_cmd, omega_cmd) = decelerate(v_a, omega_a, dt);
  }

  if (recorder) {
    for (int idx = 0; idx < static_cast<int>(buffered.size()); ++idx) {
      Buffered& b = buffered[static_cast<size_t>(idx)];
      b.data["selected"] = idx == best_index ? 1.0 : 0.0;
      recorder->candidate_evaluated(b.state_xy, b.cost, b.data, b.rollout_xy);
    }
  }

  return core::VelocityCommand{v_cmd, omega_cmd};
}

}  // namespace navigation::local_planning
