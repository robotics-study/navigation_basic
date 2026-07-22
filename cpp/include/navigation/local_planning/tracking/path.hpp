#pragma once

#include <optional>
#include <vector>

#include "navigation/core/types.hpp"

namespace navigation::local_planning {

// Shared reference-path geometry for tracking planners. Path trackers (Pure
// Pursuit and its descendants) all need the same primitives: project a probe
// point onto the path, advance a monotonic progress index along it, and
// intersect a lookahead circle with the polyline. Kept as free functions so
// each planner owns its progress state while sharing the geometry. Mirrors the
// Python `local_planning/tracking/_path.py`.

core::Point closest_point_on_segment(const core::Point& p, const core::Point& a,
                                     const core::Point& b);

double sq_dist(const core::Point& a, const core::Point& b);

// Forward-most intersection of the probe-centered lookahead circle with segment
// a->b, as a parameter t in [0, 1], or nullopt if the segment stays entirely
// inside/outside the circle.
std::optional<double> segment_circle_forward_t(const core::Point& p, const core::Point& a,
                                               const core::Point& b, double radius);

// Nearest-segment index at or after start_index -- monotonic forward-only so a
// self-crossing path never snaps tracking backward to an earlier,
// geometrically-closer crossing.
int advance_progress_index(const std::vector<core::Point>& path, const core::Point& probe,
                           int start_index);

// First forward intersection of the lookahead circle with the path at or after
// start_index; falls back to the path end when the remaining path is shorter
// than the lookahead distance (and to the probe itself on an empty path).
core::Point lookahead_point(const std::vector<core::Point>& path, int start_index,
                            const core::Point& robot_xy, double lookahead_distance);

}  // namespace navigation::local_planning
