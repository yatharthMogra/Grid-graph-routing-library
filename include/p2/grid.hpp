#pragma once

#include <cmath>
#include <cstddef>
#include <functional>
#include <vector>

namespace p2 {

struct GridCoord {
    int x = 0;
    int y = 0;

    bool operator==(const GridCoord& o) const noexcept { return x == o.x && y == o.y; }
    bool operator!=(const GridCoord& o) const noexcept { return !(*this == o); }
};

struct GridCoordHash {
    std::size_t operator()(const GridCoord& c) const noexcept {
        std::size_t hx = static_cast<std::size_t>(c.x);
        std::size_t hy = static_cast<std::size_t>(c.y);
        return hx ^ (hy << 16) ^ (hy >> 16);
    }
};

enum class Connectivity { Four, Eight };

/** Implicit grid: passable cells vs blocked; neighbors generated on demand. */
class Grid {
public:
    Grid(int width, int height, std::function<bool(int, int)> blocked)
        : width_(width), height_(height), blocked_(std::move(blocked)) {}

    int width() const noexcept { return width_; }
    int height() const noexcept { return height_; }

    bool in_bounds(int x, int y) const noexcept {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }

    bool in_bounds(GridCoord c) const noexcept { return in_bounds(c.x, c.y); }

    bool blocked(int x, int y) const {
        return !in_bounds(x, y) || blocked_(x, y);
    }

    bool blocked(GridCoord c) const { return blocked(c.x, c.y); }

    int to_index(GridCoord c) const noexcept { return c.y * width_ + c.x; }

    GridCoord from_index(int i) const noexcept {
        return {i % width_, i / width_};
    }

    int vertex_count() const noexcept { return width_ * height_; }

    /** Cardinal neighbors only; skips blocked and out of bounds. */
    void neighbors4(GridCoord c, std::vector<GridCoord>& out) const {
        out.clear();
        static const int dx[] = {1, -1, 0, 0};
        static const int dy[] = {0, 0, 1, -1};
        for (int k = 0; k < 4; ++k) {
            int nx = c.x + dx[k];
            int ny = c.y + dy[k];
            if (!blocked(nx, ny))
                out.push_back({nx, ny});
        }
    }

    /** 8-connected; diagonal only if both adjacent cardinals are passable (no corner-cutting). */
    void neighbors8(GridCoord c, std::vector<GridCoord>& out) const {
        out.clear();
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0)
                    continue;
                int nx = c.x + dx;
                int ny = c.y + dy;
                if (blocked(nx, ny))
                    continue;
                if (dx != 0 && dy != 0) {
                    if (blocked(c.x + dx, c.y) || blocked(c.x, c.y + dy))
                        continue;
                }
                out.push_back({nx, ny});
            }
        }
    }

    void neighbors(GridCoord c, Connectivity conn, std::vector<GridCoord>& out) const {
        if (conn == Connectivity::Four)
            neighbors4(c, out);
        else
            neighbors8(c, out);
    }

    /** Unit cardinal, sqrt(2) diagonal — matches typical 8-connected costs. */
    static double step_cost(GridCoord from, GridCoord to) {
        int dx = std::abs(to.x - from.x);
        int dy = std::abs(to.y - from.y);
        if (dx + dy == 1)
            return 1.0;
        if (dx == 1 && dy == 1)
            return std::sqrt(2.0);
        return std::hypot(static_cast<double>(dx), static_cast<double>(dy));
    }

private:
    int width_;
    int height_;
    std::function<bool(int, int)> blocked_;
};

}  // namespace p2
