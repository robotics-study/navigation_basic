#include "navigation/local_planning/band/band.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "navigation/local_planning/geometry.hpp"

namespace navigation::local_planning {

std::pair<std::optional<core::Point>, double> nearest_occupied(const core::ObstacleQuery& space,
                                                                const core::Point& p,
                                                                double radius) {
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

core::Point point_at_arclength(const std::vector<core::Point>& points, double s) {
  if (points.size() == 1 || s <= 0.0) return points[0];
  double remaining = s;
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    const core::Point& a = points[i];
    const core::Point& b = points[i + 1];
    double seg_len = std::sqrt(sq_dist(a, b));
    if (remaining <= seg_len) {
      if (seg_len < 1e-12) return a;
      double t = remaining / seg_len;
      return core::Point{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
    }
    remaining -= seg_len;
  }
  return points.back();
}

std::vector<core::Point> resample_polyline(const std::vector<core::Point>& points,
                                           double spacing) {
  if (points.size() < 2) return points;
  double total = 0.0;
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    total += std::sqrt(sq_dist(points[i], points[i + 1]));
  }
  if (total < 1e-12) return {points.front(), points.back()};
  // Same max(1, round(total/spacing)) contract as _band.py. Python's round() is
  // half-to-even, and std::lround is half-away-from-zero -- they disagree exactly at
  // .5 ties, so use nearbyint (FE_TONEAREST default = ties-to-even) to keep the
  // segment count bit-identical with Python even on that measure-zero boundary.
  long n_segments = std::max(1L, static_cast<long>(std::nearbyint(total / spacing)));
  double step = total / static_cast<double>(n_segments);
  std::vector<core::Point> out;
  out.reserve(static_cast<size_t>(n_segments) + 1);
  out.push_back(points.front());
  for (long k = 1; k < n_segments; ++k) {
    out.push_back(point_at_arclength(points, static_cast<double>(k) * step));
  }
  out.push_back(points.back());
  return out;
}

}  // namespace navigation::local_planning
