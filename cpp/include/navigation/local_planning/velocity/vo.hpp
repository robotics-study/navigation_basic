#pragma once

#include <string>
#include <vector>

#include "navigation/core/params.hpp"
#include "navigation/core/types.hpp"
#include "navigation/local_planning/velocity/velocity_obstacle.hpp"

// Velocity Obstacle (Fiorini & Shiller 1998, DOI 10.1177/027836499801700706):
// every tick, build one truncated cone per nearby obstacle (apex = the
// obstacle's own velocity) and pick the candidate velocity closest to the
// goal-seeking preferred velocity that lies outside every cone.
namespace navigation::local_planning {

class Vo final : public VelocityObstaclePlanner {
 public:
  explicit Vo(core::ParamSet params);

  std::string name() const override { return "vo"; }

 protected:
  VelocitySelection select_velocity(const core::Point& v_pref,
                                    const std::vector<DynamicObstacle>& neighbors,
                                    const std::vector<DynamicObstacle>& statics,
                                    const core::RobotState& state, double dt) override;

 private:
  int speed_samples_;
  int angle_samples_;
};

}  // namespace navigation::local_planning
