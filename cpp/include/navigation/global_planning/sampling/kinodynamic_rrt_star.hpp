#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// Kinodynamic RRT* (Webb & van den Berg 2013): RRT* for systems with differential
// constraints. Edges are the trajectories of a fixed-final-state, free-final-time
// optimal controller for a linear system x' = Ax + Bu with cost J = integral(1 +
// u^T R u) dt; that optimal-arrival cost is used as BOTH the nearest-neighbour
// metric and the choose-parent / rewire cost, exactly as geometric RRT* uses the
// Euclidean distance. The planner OWNS its dynamics (a 2D double integrator with
// state (x, y, vx, vy)) and depends only on the SamplingSpace capability, which it
// queries on the (x, y) projection for collision checking. Start and goal are
// lifted to rest states; the returned path is the projected optimal trajectory.
class KinodynamicRrtStarPlanner final : public core::SamplingPlanner {
 public:
  explicit KinodynamicRrtStarPlanner(core::ParamSet params)
      : core::SamplingPlanner(std::move(params)) {}
  std::string name() const override { return "kinodynamic_rrt_star"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::SAMPLING_SPACE};
  }
  core::PlanResult<core::Point> plan(core::SamplingSpace<core::Point>& space,
                                     const core::Point& start, const core::Point& goal,
                                     core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
