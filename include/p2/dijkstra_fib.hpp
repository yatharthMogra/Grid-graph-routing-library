#pragma once

#include "fibonacci_heap.hpp"
#include "grid.hpp"

#include <limits>
#include <vector>

namespace p2 {

/**
 * Single-source shortest paths on a grid using a Fibonacci heap (decrease_key).
 * Same semantics as dijkstra_pq; asymptotically O(E + V log V) with amortized bounds.
 * Constant factors are often worse than std::priority_queue on typical grids.
 */
inline void dijkstra_fib(const Grid& grid, GridCoord start, Connectivity conn,
                         std::vector<double>& dist_out, std::vector<int>& parent_out) {
    const int V = grid.vertex_count();
    const double inf = std::numeric_limits<double>::infinity();
    dist_out.assign(static_cast<std::size_t>(V), inf);
    parent_out.assign(static_cast<std::size_t>(V), -1);

    if (grid.blocked(start))
        return;

    FibonacciHeap<double, int> heap;
    std::vector<typename FibonacciHeap<double, int>::Handle> handle(
        static_cast<std::size_t>(V));

    const int s = grid.to_index(start);
    dist_out[static_cast<std::size_t>(s)] = 0.0;
    handle[static_cast<std::size_t>(s)] = heap.insert(0.0, s);

    std::vector<GridCoord> nbrs;
    std::vector<char> done(static_cast<std::size_t>(V), 0);

    while (!heap.empty()) {
        auto [d, u] = heap.extract_min();
        handle[static_cast<std::size_t>(u)] = {};
        if (done[static_cast<std::size_t>(u)])
            continue;
        if (d > dist_out[static_cast<std::size_t>(u)])
            continue;
        done[static_cast<std::size_t>(u)] = 1;

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
                auto& hv = handle[static_cast<std::size_t>(v)];
                if (!hv.valid())
                    hv = heap.insert(nd, v);
                else
                    heap.decrease_key(hv, nd);
            }
        }
    }
}

}  // namespace p2
