#pragma once

#include "grid.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <vector>

namespace p2 {

/**
 * A* on an implicit grid. Heuristic should be admissible for the chosen connectivity
 * (e.g. Manhattan + 4-way unit cost; octile/Euclidean + 8-way with Grid::step_cost).
 */
class AStarPathfinder {
public:
    using HeuristicFn = std::function<double(GridCoord, GridCoord)>;

    explicit AStarPathfinder(const Grid& grid) : grid_(grid) {}

    std::vector<GridCoord> find_path(GridCoord start, GridCoord goal, HeuristicFn h,
                                     Connectivity conn) {
        std::vector<GridCoord> path;
        if (grid_.blocked(start) || grid_.blocked(goal))
            return path;
        if (start == goal) {
            path.push_back(start);
            return path;
        }

        const int V = grid_.vertex_count();
        const double inf = std::numeric_limits<double>::infinity();
        std::vector<double> g_score(static_cast<std::size_t>(V), inf);
        std::vector<int> parent(static_cast<std::size_t>(V), -1);
        std::vector<char> closed(static_cast<std::size_t>(V), 0);

        const int start_i = grid_.to_index(start);
        const int goal_i = grid_.to_index(goal);
        g_score[static_cast<std::size_t>(start_i)] = 0.0;

        using OpenElem = std::pair<double, int>;  // f, vertex_index
        std::priority_queue<OpenElem, std::vector<OpenElem>, std::greater<OpenElem>> open;
        const double h0 = h(start, goal);
        open.push({h0, start_i});

        std::vector<GridCoord> nbrs;
        constexpr double eps = 1e-12;

        while (!open.empty()) {
            auto [f_pop, u] = open.top();
            open.pop();
            if (closed[static_cast<std::size_t>(u)])
                continue;
            GridCoord u_coord = grid_.from_index(u);
            const double g_u = g_score[static_cast<std::size_t>(u)];
            const double f_u = g_u + h(u_coord, goal);
            if (f_pop > f_u + eps)
                continue;
            closed[static_cast<std::size_t>(u)] = 1;

            if (u == goal_i)
                break;

            grid_.neighbors(u_coord, conn, nbrs);
            for (const GridCoord& v_coord : nbrs) {
                const int v = grid_.to_index(v_coord);
                const double step = (conn == Connectivity::Four)
                    ? 1.0
                    : Grid::step_cost(u_coord, v_coord);
                const double tentative = g_u + step;
                if (tentative + eps < g_score[static_cast<std::size_t>(v)]) {
                    g_score[static_cast<std::size_t>(v)] = tentative;
                    parent[static_cast<std::size_t>(v)] = u;
                    const double f = tentative + h(v_coord, goal);
                    open.push({f, v});
                }
            }
        }

        if (g_score[static_cast<std::size_t>(goal_i)] >= inf)
            return path;

        for (int cur = goal_i; cur >= 0; cur = parent[static_cast<std::size_t>(cur)])
            path.push_back(grid_.from_index(cur));
        std::reverse(path.begin(), path.end());
        return path;
    }

private:
    const Grid& grid_;
};

}  // namespace p2
