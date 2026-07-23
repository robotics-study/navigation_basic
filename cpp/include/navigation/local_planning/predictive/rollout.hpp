#pragma once

#include <utility>
#include <vector>

#include "navigation/core/capabilities.hpp"
#include "navigation/core/types.hpp"

// Shared receding-horizon rollout and cost for the predictive family (MPC,
// MPPI). Both planners optimize the same finite-horizon control sequence
// U = [u_0, ..., u_{H-1}] (u_k = (v_k, omega_k)) against the same cost J(U);
// only the optimizer differs (finite-difference gradient descent vs.
// path-integral sampling). Keeping the closed-form arc integrator and the
// scalar cost here as free functions both planners call makes that "same J,
// different optimizer" contrast the only thing separating them. Mirrors the
// Python `local_planning/predictive/_rollout.py`.
namespace navigation::local_planning {

// A control u_k = (v, omega): first = linear speed, second = angular rate.
using Control = std::pair<double, double>;

// Below this |omega| the closed-form arc (division by omega) is numerically
// unstable, so integration falls back to the straight-line limit instead.
// Re-implemented here rather than shared from the simulator because algorithm
// modules depend on core only, never on the simulator -- the same reason DWA
// keeps its own kOmegaEps. Sharing the exact closed-form arc (identical to the
// simulator's integrate_unicycle) keeps prediction and executed motion on one
// discretization, so observed behavior is the algorithm's, not an integrator
// mismatch.
inline constexpr double kOmegaEps = 1e-9;

double clamp(double value, double lo, double hi);

// Exact constant-(v, omega) unicycle arc over one step h (closed form).
core::Pose unicycle_step(const core::Pose& s, const Control& u, double h);

// States s_1..s_H from applying each control in `controls` from s0 (length
// H = controls.size(); s0 is NOT included -- callers prepend it themselves).
std::vector<core::Pose> rollout(const core::Pose& s0, const std::vector<Control>& controls, double h);

// Receding-horizon cost J(U) shared by MPC and MPPI. traj[i] is s_{i+1} and
// pairs with controls[i] = u_i. The obstacle term uses the continuous
// nearest-occupied clearance (not the cell-quantized distance_to_nearest, which
// is constant inside a cell and would give a zero finite-difference gradient).
// Explicit scalar accumulation (no vectorization) keeps the fold order
// identical across Python/C++/TS.
double sequence_cost(const core::ObstacleQuery& space, const std::vector<core::Pose>& traj,
                     const std::vector<Control>& controls, const core::Pose& goal,
                     double footprint_radius, double w_goal, double w_obstacle,
                     double min_obstacle_dist, double w_control);

}  // namespace navigation::local_planning
