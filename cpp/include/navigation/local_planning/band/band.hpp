#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "navigation/core/capabilities.hpp"
#include "navigation/core/types.hpp"

namespace navigation::local_planning {

// Shared geometry for the band family (Elastic Bands, TEB). Both members
// represent their state as a polyline (bubble centers / poses) deformed
// against the obstacle field each tick, so both need the same two
// primitives: the closest occupied cell to a point (bubble clearance /
// obstacle gradient) and arc-length resampling of a polyline (band
// initialization / TEB's progress projection). Mirrors the Python
// `local_planning/band/_band.py`.

// Closest occupied cell center to p within radius, and the continuous
// (non-quantized) distance to it -- nullopt (with distance +inf) if none.
// Strict `<` keeps the first tie in occupied_within's row/col-ascending
// list, so a symmetric cluster of equidistant cells resolves the same way
// in every language rather than depending on iteration/comparison order.
std::pair<std::optional<core::Point>, double> nearest_occupied(const core::ObstacleQuery& space,
                                                                const core::Point& p,
                                                                double radius);

// Point on polyline `points` at arc-length s from its start; clamped to the
// last point once s reaches or exceeds the total length.
core::Point point_at_arclength(const std::vector<core::Point>& points, double s);

// Re-samples `points` at even `spacing` arc-length intervals, exactly
// preserving the start and end points. A degenerate (near-zero-length) input
// collapses to just those two endpoints.
std::vector<core::Point> resample_polyline(const std::vector<core::Point>& points,
                                           double spacing);

}  // namespace navigation::local_planning
