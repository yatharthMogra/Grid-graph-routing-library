#pragma once

#include "grid.hpp"

#include <limits>
#include <queue>
#include <vector>

namespace p2 {

/**
 * Single-source shortest paths on a grid using std::priority_queue (binary heap).
 * Lazy relaxation: duplicate entries may be pushed; stale pops are skipped.
 * std::priority_queue often beats Fibonacci heaps in practice on small/medium grids.
 */
inline void dijkstra_pq(const Grid& grid, GridCoord start, Connectivity conn,
                        std::vector<double>& dist_out, std::vector<int>& parent_out) {
    const int V = grid.vertex_count();
    const double inf = std::numeric_limits<double>::infinity();
    dist_out.assign(static_cast<std::size_t>(V), inf);
    parent_out.assign(static_cast<std::size_t>(V), -1);

    if (grid.blocked(start))
        return;

    const int s = grid.to_index(start);
    dist_out[static_cast<std::size_t>(s)] = 0.0;

    using Elem = std::pair<double, int>;  // dist, vertex
    std::priority_queue<Elem, std::vector<Elem>, std::greater<Elem>> pq;
    pq.push({0.0, s});

    std::vector<GridCoord> nbrs;

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > dist_out[static_cast<std::size_t>(u)])
            continue;
        GridCoord u_coord = grid.from_index(u);
        grid.neighbors(u_coord, conn, nbrs);
        for (const GridCoord& v_coord : nbrs) {
            const int v = grid.to_index(v_coord);
            const double w = (conn == Connectivity::Four) ? 1.0
                                                          : Grid::step_cost(u_coord, v_coord);
            const double nd = d + w;
            if (nd < dist_out[static_cast<std::size_t>(v)]) {
                dist_out[static_cast<std::size_t>(v)] = nd;
                parent_out[static_cast<std::size_t>(v)] = u;
                pq.push({nd, v});
            }
        }
    }
}

}  // namespace p2
