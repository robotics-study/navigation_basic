#pragma once

#include <optional>
#include <vector>

#include "navigation/core/types.hpp"
#include "navigation/local_planning/geometry.hpp"

namespace navigation::local_planning {

// Reference-path lookahead geometry for tracking planners. Path trackers
// (Pure Pursuit and its descendants) additionally need to intersect a
// lookahead circle with the polyline -- kept here as a tracking-only free
// function. The point-projection/progress-index primitives this family
// shares with the band family live in geometry.hpp (included above) so
// callers of this header keep getting them unqualified. Mirrors the Python
// `local_planning/tracking/_path.py`.

// Forward-most intersection of the probe-centered lookahead circle with segment
// a->b, as a parameter t in [0, 1], or nullopt if the segment stays entirely
// inside/outside the circle.
std::optional<double> segment_circle_forward_t(const core::Point& p, const core::Point& a,
                                               const core::Point& b, double radius);

// First forward intersection of the lookahead circle with the path at or after
// start_index; falls back to the path end when the remaining path is shorter
// than the lookahead distance (and to the probe itself on an empty path).
core::Point lookahead_point(const std::vector<core::Point>& path, int start_index,
                            const core::Point& robot_xy, double lookahead_distance);

}  // namespace navigation::local_planning
