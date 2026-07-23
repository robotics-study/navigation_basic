#include "navigation/local_planning/geometry.hpp"

#include <cmath>
#include <limits>

namespace navigation::local_planning {

std::pair<std::optional<core::Point>, double> nearest_occupied(const core::ObstacleQuery& space,
                                                               const core::Point& p, double radius) {
  std::optional<core::Point> best;
  double best_sq = std::numeric_limits<double>::infinity();
  for (const core::Point& o : space.occupied_within(p, radius)) {
    double d = sq_dist(p, o);
    if (d < best_sq) {
      best_sq = d;
      best = o;
    }
  }
  if (!best) return {std::nullopt, std::numeric_limits<double>::infinity()};
  return {best, std::sqrt(best_sq)};
}

}  // namespace navigation::local_planning
