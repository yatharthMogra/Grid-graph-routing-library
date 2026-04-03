# p2_router — Grid graph routing (C++)

## What we built

This project is a small **header-only C++17 library** for shortest-path routing on **implicit grid graphs** (no explicit adjacency list per cell). It includes:

- **A\*** — `AStarPathfinder` in [`include/p2/astar.hpp`](include/p2/astar.hpp), with a pluggable heuristic (`std::function`) and either **4-** or **8-connected** neighborhoods.
- **Heuristics** — Manhattan, Euclidean, Chebyshev, and octile distances in [`include/p2/heuristics.hpp`](include/p2/heuristics.hpp), aligned with the movement model (e.g. Manhattan with 4-way unit steps; octile / Euclidean with 8-way costs using cardinal and diagonal step weights).
- **Dijkstra (binary heap)** — [`include/p2/dijkstra.hpp`](include/p2/dijkstra.hpp) uses `std::priority_queue` with **lazy relaxation** (re-push on improve; skip stale pops).
- **Fibonacci heap** — [`include/p2/fibonacci_heap.hpp`](include/p2/fibonacci_heap.hpp): generic min-heap with `insert`, `extract_min`, and **`decrease_key`** via stable handles.
- **Dijkstra (Fibonacci heap)** — [`include/p2/dijkstra_fib.hpp`](include/p2/dijkstra_fib.hpp) wires that heap into the same grid API as the binary-heap version.
- **Grid layer** — [`include/p2/grid.hpp`](include/p2/grid.hpp): coordinates, obstacle predicate, neighbor generation (8-neighbor diagonals avoid **corner-cutting** through blocked cells).

Build with CMake: the **`p2_router`** target is an **INTERFACE** library; add `include/` to your include path via `target_link_libraries(... p2_router)`.

## Why it matters

Grid routing is the same **combinatorial core** that shows up in **VLSI / physical design**: global and detailed **routing**, layer-aware connectivity, and **netlist-style** reachability questions often reduce to shortest paths (and more generally flows) on graphs derived from a discretized layout. Even when production tools use richer graphs and constraints, the ideas here — **consistent heuristics**, **single-source distances**, and **priority-queue behavior** — carry over directly.

Separately, this project highlights a classic **algorithms engineering** tension: **Fibonacci heaps** improve **asymptotic** bounds for Dijkstra with `decrease_key`, but **binary heaps** (especially with lazy decrease) often win on **real grids** because of **cache locality** and **lower constant factors**. Having both implementations makes that tradeoff concrete rather than theoretical.

## What we learned

1. **Heuristics must match the graph** — An admissible A\* heuristic depends on **connectivity** and **edge costs** (e.g. Manhattan pairs naturally with 4-way unit cost; octile pairs with diagonal costs).
2. **Two correct styles of Dijkstra on a heap** — **Lazy** binary-heap Dijkstra avoids `decrease_key` entirely; **Fibonacci-heap** Dijkstra uses **`decrease_key`** and one node per vertex (until extracted). Both are standard; they differ in API and analysis.
3. **Fibonacci heaps are subtle** — Correctness hinges on **cut**, **cascading cut**, **consolidate**, and **careful pointer surgery** on circular sibling lists; handles must be **invalidated** after `extract_min` if the node is freed.
4. **Implicit grids stay simple and fast** — Linear indexing `id = y * width + x` gives dense `dist` / `parent` arrays without building `adj[u]` for every cell.

## Quick usage

```cmake
target_link_libraries(your_target PRIVATE p2_router)
```

```cpp
#include <p2/grid.hpp>
#include <p2/heuristics.hpp>
#include <p2/astar.hpp>
#include <p2/dijkstra.hpp>
#include <p2/dijkstra_fib.hpp>

// Grid: width, height, blocked(x,y)
p2::Grid grid(100, 100, [](int x, int y) { return /* obstacle */ false; });

p2::AStarPathfinder astar(grid);
auto path = astar.find_path(
    {0, 0}, {99, 99},
    p2::heuristic_manhattan,
    p2::Connectivity::Four);

std::vector<double> dist;
std::vector<int> parent;
p2::dijkstra_pq(grid, {0, 0}, p2::Connectivity::Four, dist, parent);
p2::dijkstra_fib(grid, {0, 0}, p2::Connectivity::Four, dist, parent);
```

## Requirements

- **C++17**
- **CMake** 3.16+
