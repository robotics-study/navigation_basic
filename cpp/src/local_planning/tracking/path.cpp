#include "navigation/local_planning/tracking/path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation::local_planning {

core::Point closest_point_on_segment(const core::Point& p, const core::Point& a,
                                     const core::Point& b) {
  double dx = b.x - a.x, dy = b.y - a.y;
  double seg_len_sq = dx * dx + dy * dy;
  if (seg_len_sq < 1e-12) return a;
  double t = std::max(0.0, std::min(1.0, ((p.x - a.x) * dx + (p.y - a.y) * dy) / seg_len_sq));
  return core::Point{a.x + t * dx, a.y + t * dy};
}

double sq_dist(const core::Point& a, const core::Point& b) {
  double dx = a.x - b.x, dy = a.y - b.y;
  return dx * dx + dy * dy;
}

// Solves |a + t*(b-a) - p|^2 = radius^2 (Coulter 1992 sec. 3: circle-line
// intersection) and keeps the larger root in range -- the exit point, i.e.
// further along the path -- so the chosen point always leads the robot forward
// rather than back toward where it entered the circle.
std::optional<double> segment_circle_forward_t(const core::Point& p, const core::Point& a,
                                               const core::Point& b, double radius) {
  double dx = b.x - a.x, dy = b.y - a.y;
  double fx = a.x - p.x, fy = a.y - p.y;
  double aa = dx * dx + dy * dy;
  if (aa < 1e-12) return std::nullopt;
  double bb = 2.0 * (fx * dx + fy * dy);
  double cc = fx * fx + fy * fy - radius * radius;
  double disc = bb * bb - 4.0 * aa * cc;
  if (disc < 0.0) return std::nullopt;
  double sq = std::sqrt(disc);
  for (double t : {(-bb + sq) / (2.0 * aa), (-bb - sq) / (2.0 * aa)}) {
    if (t >= 0.0 && t <= 1.0) return t;
  }
  return std::nullopt;
}

int advance_progress_index(const std::vector<core::Point>& path, const core::Point& probe,
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

core::Point lookahead_point(const std::vector<core::Point>& path, int start_index,
                            const core::Point& robot_xy, double lookahead_distance) {
  // No segment crossing the lookahead circle means the remaining path is
  // shorter than L_d, so aim at the path's end (the goal).
  core::Point target = path.empty() ? robot_xy : path.back();
  for (int i = start_index; !path.empty() && i < static_cast<int>(path.size()) - 1; ++i) {
    auto t = segment_circle_forward_t(robot_xy, path[static_cast<size_t>(i)],
                                      path[static_cast<size_t>(i) + 1], lookahead_distance);
    if (t) {
      const core::Point& a = path[static_cast<size_t>(i)];
      const core::Point& b = path[static_cast<size_t>(i) + 1];
      target = core::Point{a.x + *t * (b.x - a.x), a.y + *t * (b.y - a.y)};
      break;
    }
  }
  return target;
}

}  // namespace navigation::local_planning
