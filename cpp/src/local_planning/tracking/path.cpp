#include "navigation/local_planning/tracking/path.hpp"

#include <cmath>

namespace navigation::local_planning {

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
