#include "navigation/local_planning/velocity/vo.hpp"

#include <utility>

namespace navigation::local_planning {

Vo::Vo(core::ParamSet params)
    : VelocityObstaclePlanner(std::move(params)),
      speed_samples_(params_.get_int("speed_samples")),
      angle_samples_(params_.get_int("angle_samples")) {}

VelocitySelection Vo::select_velocity(const core::Point& v_pref,
                                      const std::vector<DynamicObstacle>& neighbors,
                                      const std::vector<DynamicObstacle>& statics,
                                      const core::RobotState& state, double /*dt*/) {
  core::Point pos{state.pose.x, state.pose.y};
  std::vector<DynamicObstacle> obstacles = neighbors;
  obstacles.insert(obstacles.end(), statics.begin(), statics.end());
  return select_sampled_velocity(v_pref, obstacles, pos, agent_radius_, neighbor_dist_,
                                 time_horizon_, max_speed_, speed_samples_, angle_samples_,
                                 [](const DynamicObstacle& o) { return o.velocity; });
}

}  // namespace navigation::local_planning
