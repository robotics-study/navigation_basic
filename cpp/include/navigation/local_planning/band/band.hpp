#pragma once

#include <vector>

#include "navigation/core/types.hpp"

namespace navigation::local_planning {

// Arc-length polyline geometry for the band family (Elastic Bands, TEB). Both
// members represent their state as a polyline (bubble centers / poses) deformed
// against the obstacle field each tick, so both need arc-length
// sampling/resampling of a polyline (band initialization / TEB's progress
// projection). Mirrors the Python `local_planning/band/_band.py`.
// (Nearest-occupied lookup, once band-only, now lives in the category-shared
// geometry.hpp since the predictive family reuses it too.)

// Point on polyline `points` at arc-length s from its start; clamped to the
// last point once s reaches or exceeds the total length.
core::Point point_at_arclength(const std::vector<core::Point>& points, double s);

// Re-samples `points` at even `spacing` arc-length intervals, exactly
// preserving the start and end points. A degenerate (near-zero-length) input
// collapses to just those two endpoints.
std::vector<core::Point> resample_polyline(const std::vector<core::Point>& points,
                                           double spacing);

}  // namespace navigation::local_planning
