#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "navigation/core/capabilities.hpp"
#include "navigation/core/types.hpp"

// Geometry shared across the local_planning category. The angle utility and the
// polyline primitives (nearest point on a segment, squared distance, monotonic
// progress-index advance) stay inline in this header — each needed by both the
// tracking family (path.hpp's lookahead circle) and the band family (arc-length
// progress projection), promoted out of tracking/path.{hpp,cpp} so a new family
// can reuse them without an include across families. The nearest-occupied
// lookup, once band-only, is declared here and defined in geometry.cpp since the
// predictive family (MPC/MPPI obstacle cost) reuses the exact same helper the
// band family uses for bubble/pose clearance.
namespace navigation::local_planning {

// Normalizes `angle` (radians) to (-pi, pi].
inline double wrap_to_pi(double angle) {
  double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (wrapped <= 0.0) wrapped += 2.0 * M_PI;
  return wrapped - M_PI;
}

inline core::Point closest_point_on_segment(const core::Point& p, const core::Point& a,
                                            const core::Point& b) {
  double dx = b.x - a.x, dy = b.y - a.y;
  double seg_len_sq = dx * dx + dy * dy;
  if (seg_len_sq < 1e-12) return a;
  double t = std::max(0.0, std::min(1.0, ((p.x - a.x) * dx + (p.y - a.y) * dy) / seg_len_sq));
  return core::Point{a.x + t * dx, a.y + t * dy};
}

inline double sq_dist(const core::Point& a, const core::Point& b) {
  double dx = a.x - b.x, dy = a.y - b.y;
  return dx * dx + dy * dy;
}

// Nearest-segment index at or after start_index -- monotonic forward-only so a
// self-crossing path never snaps tracking backward to an earlier,
// geometrically-closer crossing.
inline int advance_progress_index(const std::vector<core::Point>& path, const core::Point& probe,
                                  int start_index) {
  if (path.size() < 2) return start_index;
  int best_index = start_index;
  double best_sq_dist = std::numeric_limits<double>::infinity();
  for (int i = start_index; i < static_cast<int>(path.size()) - 1; ++i) {
    core::Point closest = closest_point_on_segment(probe, path[static_cast<size_t>(i)],
                                                   path[static_cast<size_t>(i) + 1]);
    double d = sq_dist(probe, closest);
    // <=, not <: consecutive segments share their joint endpoint, so a probe
    // sitting exactly at a corner ties every segment ending/starting there.
    // Preferring the later (more forward) segment on a tie keeps progress
    // advancing through the corner instead of latching onto the segment just
    // traveled.
    if (d <= best_sq_dist) {
      best_sq_dist = d;
      best_index = i;
    }
  }
  return best_index;
}

// Closest occupied cell center to p within radius, and the continuous
// (non-quantized) distance to it -- nullopt (with distance +inf) if none.
// Strict `<` keeps the first tie in occupied_within's row/col-ascending list,
// so a symmetric cluster of equidistant cells resolves the same way in every
// language rather than depending on iteration/comparison order. Defined in
// geometry.cpp (not inline) since it loops over an obstacle query.
std::pair<std::optional<core::Point>, double> nearest_occupied(const core::ObstacleQuery& space,
                                                               const core::Point& p, double radius);

}  // namespace navigation::local_planning
