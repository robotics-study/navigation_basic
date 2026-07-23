#include "navigation/local_planning/velocity/orca.hpp"

#include <cmath>
#include <utility>

namespace navigation::local_planning {

Orca::Orca(core::ParamSet params)
    : VelocityObstaclePlanner(std::move(params)),
      time_horizon_obst_(params_.get_float("time_horizon_obst")) {}

std::vector<HalfPlane> Orca::half_planes_for(const std::vector<DynamicObstacle>& obstacles,
                                             const core::Point& pos, const core::Point& v_self,
                                             double dt, double tau) const {
  std::vector<HalfPlane> planes;
  for (const DynamicObstacle& o : obstacles) {
    core::Point rel_pos{o.position.x - pos.x, o.position.y - pos.y};
    if (std::hypot(rel_pos.x, rel_pos.y) >= neighbor_dist_ + o.radius) continue;
    core::Point rel_vel{v_self.x - o.velocity.x, v_self.y - o.velocity.y};
    planes.push_back(orca_half_plane(rel_pos, rel_vel, v_self, agent_radius_ + o.radius, tau, dt));
  }
  return planes;
}

VelocitySelection Orca::select_velocity(const core::Point& v_pref,
                                        const std::vector<DynamicObstacle>& neighbors,
                                        const std::vector<DynamicObstacle>& statics,
                                        const core::RobotState& state, double dt) {
  double theta = state.pose.theta;
  core::Point pos{state.pose.x, state.pose.y};
  core::Point v_self{state.v * std::cos(theta), state.v * std::sin(theta)};
  std::vector<HalfPlane> planes = half_planes_for(neighbors, pos, v_self, dt, time_horizon_);
  std::vector<HalfPlane> static_planes =
      half_planes_for(statics, pos, v_self, dt, time_horizon_obst_);
  planes.insert(planes.end(), static_planes.begin(), static_planes.end());
  LinearProgram2DResult lp = linear_program_2d(planes, v_pref, max_speed_);
  core::Point v_new = lp.velocity;
  if (!lp.ok) {
    v_new = linear_program_3d(planes, lp.fail_index, v_pref, max_speed_);
  }
  VelocitySelection result;
  result.velocity = v_new;
  result.constraints.reserve(planes.size());
  for (const HalfPlane& p : planes) result.constraints.push_back(halfplane_to_constraint(p));
  return result;
}

}  // namespace navigation::local_planning
