#include "navigation/local_planning/reactive/potential_fields.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "navigation/local_planning/geometry.hpp"
#include "navigation/local_planning/reactive/steering.hpp"

namespace navigation::local_planning {

PotentialFieldsPlanner::PotentialFieldsPlanner(core::ParamSet params)
    : core::ObstacleLocalPlanner(std::move(params)),
      k_att_(params_.get_float("k_att")),
      k_rep_(params_.get_float("k_rep")),
      influence_radius_(params_.get_float("influence_radius")),
      k_v_(params_.get_float("k_v")),
      k_omega_(params_.get_float("k_omega")),
      max_speed_(params_.get_float("max_speed")),
      max_omega_(params_.get_float("max_omega")),
      d_min_(params_.get_float("footprint_radius")) {}

core::VelocityCommand PotentialFieldsPlanner::compute_command(core::ObstacleQuery& space,
                                                        const core::RobotState& state,
                                                        const core::LocalTask& task,
                                                        double /*dt*/,
                                                        core::TraceRecorder* recorder) {
  const double x = state.pose.x, y = state.pose.y, theta = state.pose.theta;
  const double fx_att = k_att_ * (task.goal.x - x);
  const double fy_att = k_att_ * (task.goal.y - y);

  double fx_rep = 0.0, fy_rep = 0.0;
  for (const core::Point& o : space.occupied_within(core::Point{x, y}, influence_radius_)) {
    const double dx = x - o.x, dy = y - o.y;
    const double d = std::max(std::hypot(dx, dy), d_min_);
    if (d >= influence_radius_) continue;
    const double magnitude = k_rep_ * (1.0 / d - 1.0 / influence_radius_) * (1.0 / (d * d));
    fx_rep += magnitude * dx / d;
    fy_rep += magnitude * dy / d;
  }

  const double fx = fx_att + fx_rep;
  const double fy = fy_att + fy_rep;

  if (recorder) {
    recorder->force_computed(
        state.pose, core::TraceRecorder::EventData{{"fx_att", fx_att},
                                                    {"fy_att", fy_att},
                                                    {"fx_rep", fx_rep},
                                                    {"fy_rep", fy_rep},
                                                    {"fx", fx},
                                                    {"fy", fy}});
  }

  const double theta_d = std::atan2(fy, fx);
  const double v_eff = std::min(max_speed_, k_v_ * std::hypot(fx, fy));
  return heading_command(wrap_to_pi(theta_d - theta), k_omega_, v_eff, max_omega_);
}

}  // namespace navigation::local_planning
