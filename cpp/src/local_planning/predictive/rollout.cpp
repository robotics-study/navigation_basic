#include "navigation/local_planning/predictive/rollout.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "navigation/local_planning/geometry.hpp"

namespace navigation::local_planning {

namespace {

// Extra query band beyond the obstacle-penalty activation distance. The hinge
// term is nonzero only where clearance c_k < min_obstacle_dist, i.e. where the
// nearest occupied cell is within min_obstacle_dist + footprint_radius; querying
// one half-cell further lets the finite-difference gradient see an obstacle just
// as it enters the active band instead of only once c_k has already crossed it.
constexpr double kQueryMargin = 0.5;

}  // namespace

double clamp(double value, double lo, double hi) { return std::max(lo, std::min(hi, value)); }

core::Pose unicycle_step(const core::Pose& s, const Control& u, double h) {
  const double x = s.x, y = s.y, theta = s.theta;
  const double v = u.first, omega = u.second;
  if (std::fabs(omega) < kOmegaEps) {
    return core::Pose{x + v * h * std::cos(theta), y + v * h * std::sin(theta), theta};
  }
  const double new_theta = theta + omega * h;
  const double x2 = x + (v / omega) * (std::sin(new_theta) - std::sin(theta));
  const double y2 = y - (v / omega) * (std::cos(new_theta) - std::cos(theta));
  return core::Pose{x2, y2, wrap_to_pi(new_theta)};
}

std::vector<core::Pose> rollout(const core::Pose& s0, const std::vector<Control>& controls,
                                double h) {
  std::vector<core::Pose> states;
  states.reserve(controls.size());
  core::Pose s = s0;
  for (const Control& u : controls) {
    s = unicycle_step(s, u, h);
    states.push_back(s);
  }
  return states;
}

double sequence_cost(const core::ObstacleQuery& space, const std::vector<core::Pose>& traj,
                     const std::vector<Control>& controls, const core::Pose& goal,
                     double footprint_radius, double w_goal, double w_obstacle,
                     double min_obstacle_dist, double w_control) {
  const double gx = goal.x, gy = goal.y;
  const double r_query = min_obstacle_dist + footprint_radius + kQueryMargin;
  double total = 0.0;
  for (size_t k = 0; k < traj.size(); ++k) {
    const double dx = traj[k].x - gx;
    const double dy = traj[k].y - gy;
    total += w_goal * (dx * dx + dy * dy);
    auto [o, d_tilde] = nearest_occupied(space, core::Point{traj[k].x, traj[k].y}, r_query);
    if (o) {
      const double c_k = d_tilde - footprint_radius;
      const double hinge = min_obstacle_dist - c_k;
      if (hinge > 0.0) total += w_obstacle * hinge * hinge;
    }
    const double v = controls[k].first, omega = controls[k].second;
    total += w_control * (v * v + omega * omega);
  }
  return total;
}

}  // namespace navigation::local_planning
