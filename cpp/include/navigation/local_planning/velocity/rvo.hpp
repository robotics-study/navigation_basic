#pragma once

#include <string>
#include <vector>

#include "navigation/core/params.hpp"
#include "navigation/core/types.hpp"
#include "navigation/local_planning/velocity/velocity_obstacle.hpp"

// Reciprocal Velocity Obstacle (van den Berg, Lin & Manocha 2008,
// DOI 10.1109/ROBOT.2008.4543489): identical to VO except each cone's apex is
// shifted from the obstacle's velocity toward the midpoint of both agents'
// velocities (`reciprocity`, 0.5 by default), so each side of a symmetric
// encounter absorbs half the avoidance effort instead of both assuming the
// other holds course -- the fix for VO's reciprocal-dance oscillation.
namespace navigation::local_planning {

class Rvo final : public VelocityObstaclePlanner {
 public:
  explicit Rvo(core::ParamSet params);

  std::string name() const override { return "rvo"; }

 protected:
  VelocitySelection select_velocity(const core::Point& v_pref,
                                    const std::vector<DynamicObstacle>& neighbors,
                                    const std::vector<DynamicObstacle>& statics,
                                    const core::RobotState& state, double dt) override;

 private:
  int speed_samples_;
  int angle_samples_;
  double reciprocity_;
};

}  // namespace navigation::local_planning
