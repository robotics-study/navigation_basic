#pragma once

#include <cmath>

// Angle utility shared across the local_planning category. Header-only, small
// enough that a .cpp would only add a translation unit for no benefit — the
// exact-arc unicycle integrator and the reactive steering law both need a
// canonical heading range so pose theta stays comparable tick to tick.
namespace navigation::local_planning {

// Normalizes `angle` (radians) to (-pi, pi].
inline double wrap_to_pi(double angle) {
  double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (wrapped <= 0.0) wrapped += 2.0 * M_PI;
  return wrapped - M_PI;
}

}  // namespace navigation::local_planning
