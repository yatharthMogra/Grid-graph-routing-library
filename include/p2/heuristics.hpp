#pragma once

#include "grid.hpp"

#include <algorithm>
#include <cmath>

namespace p2 {

/** Manhattan: admissible for 4-connected grids with unit edge cost. */
inline double heuristic_manhattan(GridCoord a, GridCoord b) noexcept {
    return static_cast<double>(std::abs(a.x - b.x) + std::abs(a.y - b.y));
}

/** Euclidean straight-line distance. */
inline double heuristic_euclidean(GridCoord a, GridCoord b) noexcept {
    double dx = static_cast<double>(a.x - b.x);
    double dy = static_cast<double>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}

/** Chebyshev: admissible when diagonal and cardinal moves both cost 1. */
inline double heuristic_chebyshev(GridCoord a, GridCoord b) noexcept {
    return static_cast<double>(std::max(std::abs(a.x - b.x), std::abs(a.y - b.y)));
}

/**
 * Octile / diagonal distance: D1 per cardinal step, D2 per diagonal.
 * Admissible for 8-connected grid with those edge weights (default D1=1, D2=sqrt(2)).
 */
inline double heuristic_octile(GridCoord a, GridCoord b, double d1 = 1.0,
                               double d2 = std::sqrt(2.0)) noexcept {
    int dx = std::abs(a.x - b.x);
    int dy = std::abs(a.y - b.y);
    int dmin = std::min(dx, dy);
    int dmax = std::max(dx, dy);
    return d1 * static_cast<double>(dmax - dmin) + d2 * static_cast<double>(dmin);
}

}  // namespace p2
