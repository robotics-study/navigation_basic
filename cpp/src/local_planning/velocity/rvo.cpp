#include "navigation/local_planning/velocity/rvo.hpp"

#include <cmath>
#include <utility>

namespace navigation::local_planning {

Rvo::Rvo(core::ParamSet params)
    : VelocityObstaclePlanner(std::move(params)),
      speed_samples_(params_.get_int("speed_samples")),
      angle_samples_(params_.get_int("angle_samples")),
      reciprocity_(params_.get_float("reciprocity")) {}

VelocitySelection Rvo::select_velocity(const core::Point& v_pref,
                                       const std::vector<DynamicObstacle>& neighbors,
                                       const std::vector<DynamicObstacle>& statics,
                                       const core::RobotState& state, double /*dt*/) {
  double theta = state.pose.theta;
  core::Point pos{state.pose.x, state.pose.y};
  core::Point v_self{state.v * std::cos(theta), state.v * std::sin(theta)};
  double reciprocity = reciprocity_;
  std::vector<DynamicObstacle> obstacles = neighbors;
  obstacles.insert(obstacles.end(), statics.begin(), statics.end());
  return select_sampled_velocity(
      v_pref, obstacles, pos, agent_radius_, neighbor_dist_, time_horizon_, max_speed_,
      speed_samples_, angle_samples_, [v_self, reciprocity](const DynamicObstacle& o) {
        return rvo_apex(v_self, o.velocity, reciprocity);
      });
}

}  // namespace navigation::local_planning
