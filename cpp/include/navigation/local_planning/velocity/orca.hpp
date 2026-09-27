#pragma once

#include <string>
#include <vector>

#include "navigation/core/params.hpp"
#include "navigation/core/types.hpp"
#include "navigation/local_planning/velocity/velocity_obstacle.hpp"

// Optimal Reciprocal Collision Avoidance (van den Berg, Guy, Lin & Manocha
// 2011, DOI 10.1007/978-3-642-19457-3_1): replaces VO/RVO's sampled candidate
// grid with an exact half-plane per obstacle plus a deterministic 2D linear
// program (RVO2's linearProgram1/2), falling back to a penetration-minimizing
// 3D solve when the constraints are jointly infeasible. Static obstacles use
// a shorter, separately-configured time horizon than moving neighbors, since
// a wall's "collision" urgency isn't governed by the same lookahead as
// another agent's.
namespace navigation::local_planning {

class Orca final : public VelocityObstaclePlanner {
 public:
  explicit Orca(core::ParamSet params);

  std::string name() const override { return "orca"; }

 protected:
  VelocitySelection select_velocity(const core::Point& v_pref,
                                    const std::vector<DynamicObstacle>& neighbors,
                                    const std::vector<DynamicObstacle>& statics,
                                    const core::RobotState& state, double dt) override;

 private:
  std::vector<HalfPlane> half_planes_for(const std::vector<DynamicObstacle>& obstacles,
                                         const core::Point& pos, const core::Point& v_self,
                                         double dt, double tau) const;

  double time_horizon_obst_;
};

}  // namespace navigation::local_planning
