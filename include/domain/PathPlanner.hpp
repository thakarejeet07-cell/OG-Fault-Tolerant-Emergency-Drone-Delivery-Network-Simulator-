#ifndef DRONE_MESH_PATHPLANNER_HPP
#define DRONE_MESH_PATHPLANNER_HPP

#include <vector>
#include <optional>
#include <cstdint>
#include "Coordinate.hpp"
#include "Grid.hpp"
#include "PlanarPoint.hpp"
#include "Units.hpp"

namespace simulator {
namespace mesh {
namespace domain {

struct Path {
    std::vector<Coordinate> waypoints;
    uint64_t gridVersion{0};
    double totalDistance{0.0};
};

class PathPlanner {
public:
    /**
     * @brief A* path planning on an 8-connected grid.
     * 
     * Heuristic: Octile Distance Heuristic
     * For 8-connected grids with orthogonal cost D = 1.0 * cellSize and diagonal cost D2 = sqrt(2) * cellSize:
     *   dx = |x1 - x2|, dy = |y1 - y2|
     *   h(x, y) = D * (max(dx, dy) - min(dx, dy)) + D2 * min(dx, dy)
     * This heuristic is both admissible (never overestimates the true cost) and consistent (monotonic),
     * guaranteeing an optimal path on the grid.
     * 
     * Concurrency: Operates exclusively on an immutable Grid snapshot (by value/const reference).
     * Planning never holds the live grid lock.
     * 
     * @param snapshot An immutable copy of the Grid tagged with its version
     * @param start The starting coordinate
     * @param goal The destination coordinate
     * @return std::optional<Path> containing waypoints and version if found, or std::nullopt
     */
    static std::optional<Path> plan(const Grid& snapshot, const Coordinate& start, const Coordinate& goal);

    static std::optional<Path> plan(const Grid& snapshot, int startX, int startY, int goalX, int goalY);
};

} // namespace domain
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_PATHPLANNER_HPP
