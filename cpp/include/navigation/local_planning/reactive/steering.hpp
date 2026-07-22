#pragma once

#include <algorithm>
#include <cmath>

#include "navigation/core/types.hpp"

// Heading-command law shared by the reactive family (Potential Fields, VFH).
// Both planners reduce their tick to "steer toward this heading at this
// speed": PF from the resultant force direction, VFH from the selected
// valley direction. Each computes its own effective speed first (PF:
// min(max_speed, k_v*|F|); VFH: density-proportional slowdown) and passes it
// in as `max_speed` -- this helper owns only the steering law (turn-rate
// clamp + cos-gated forward speed), not speed selection, so it stays
// reusable across both.
namespace navigation::local_planning {

// Turn-in-place-then-drive command toward a heading error `theta_err` (rad).
//
// omega = clamp(gain * theta_err, +/- max_omega). v is gated by
// cos(theta_err) so a target behind the robot (|theta_err| > pi/2) produces
// v <= 0 clamped to 0 -- the robot rotates in place instead of driving
// backward or arcing wide.
inline core::VelocityCommand heading_command(double theta_err, double gain, double max_speed,
                                             double max_omega) {
  double omega = std::max(-max_omega, std::min(max_omega, gain * theta_err));
  double v = max_speed * std::max(0.0, std::cos(theta_err));
  return core::VelocityCommand{v, omega};
}

}  // namespace navigation::local_planning
