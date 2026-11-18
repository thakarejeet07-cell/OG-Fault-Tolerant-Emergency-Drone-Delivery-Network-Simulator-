#include "../../include/domain/PathPlanner.hpp"
#include "../../include/domain/PlanarPoint.hpp"
#include "../../include/domain/Units.hpp"
#include <cmath>
#include <queue>
#include <vector>
#include <limits>
#include <algorithm>

namespace simulator {
namespace mesh {
namespace domain {

namespace {

struct Node {
    int x{0};
    int y{0};
    double f{0.0};
    double g{0.0};

    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

double heuristic(int x1, int y1, int x2, int y2, double cellSize) {
    int dx = std::abs(x1 - x2);
    int dy = std::abs(y1 - y2);
    constexpr double SQRT2_MINUS_2 = 1.4142135623730950488 - 2.0;
    return cellSize * (static_cast<double>(dx + dy) + SQRT2_MINUS_2 * static_cast<double>(std::min(dx, dy)));
}

bool isMoveValid(const Grid& snapshot, int cx, int cy, int dx, int dy) {
    int nx = cx + dx;
    int ny = cy + dy;
    if (!snapshot.isValidCell(nx, ny) || !snapshot.isPassable(nx, ny)) {
        return false;
    }
    // Diagonal corner-cutting prevention: adjacent orthogonal cells must be passable
    if (dx != 0 && dy != 0) {
        if (!snapshot.isPassable(cx + dx, cy) || !snapshot.isPassable(cx, cy + dy)) {
            return false;
        }
    }
    return true;
}

Path reconstructPath(const Grid& snapshot, const std::vector<int>& cameFrom,
                    int goalX, int goalY, double totalDistance, uint64_t gridVersion) {
    int width = snapshot.getWidth();
    Path path;
    path.gridVersion = gridVersion;
    Meters pathMeters(totalDistance); // IMPLICIT converting constructor: double -> Meters
    path.totalDistance = pathMeters;  // IMPLICIT conversion operator: Meters -> double

    std::vector<Coordinate> coords;
    int curr = goalY * width + goalX;
    while (curr != -1) {
        int cx = curr % width;
        int cy = curr / width;
        // CLASS-TO-CLASS: Coordinate -> PlanarPoint (constructor), PlanarPoint -> Coordinate (operator)
        PlanarPoint localPt(snapshot.cellToCoordinate(cx, cy));
        coords.push_back(static_cast<Coordinate>(localPt));
        curr = cameFrom[static_cast<size_t>(curr)];
    }

    std::reverse(coords.begin(), coords.end());
    path.waypoints = std::move(coords);
    return path;
}

} // anonymous namespace

std::optional<Path> PathPlanner::plan(const Grid& snapshot, const Coordinate& start, const Coordinate& goal) {
    auto [startX, startY] = snapshot.coordinateToCell(start);
    auto [goalX, goalY] = snapshot.coordinateToCell(goal);
    return plan(snapshot, startX, startY, goalX, goalY);
}

std::optional<Path> PathPlanner::plan(const Grid& snapshot, int startX, int startY, int goalX, int goalY) {
    int width = snapshot.getWidth();
    int height = snapshot.getHeight();
    double cellSize = snapshot.getCellSizeMeters();
    uint64_t gridVersion = snapshot.getVersion();

    if (!snapshot.isValidCell(startX, startY) || !snapshot.isValidCell(goalX, goalY) ||
        !snapshot.isPassable(startX, startY) || !snapshot.isPassable(goalX, goalY)) {
        return std::nullopt;
    }

    if (startX == goalX && startY == goalY) {
        Path p;
        p.waypoints.push_back(snapshot.cellToCoordinate(startX, startY));
        p.gridVersion = gridVersion;
        p.totalDistance = 0.0;
        return p;
    }

    size_t totalCells = static_cast<size_t>(width * height);
    auto cellIndex = [width](int x, int y) -> size_t { return static_cast<size_t>(y * width + x); };

    std::vector<double> gScore(totalCells, std::numeric_limits<double>::infinity());
    std::vector<int> cameFrom(totalCells, -1);
    std::vector<bool> closed(totalCells, false);
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;

    gScore[cellIndex(startX, startY)] = 0.0;
    openSet.push({startX, startY, heuristic(startX, startY, goalX, goalY, cellSize), 0.0});

    constexpr int dirX[8] = { 1, -1,  0,  0,  1,  1, -1, -1 };
    constexpr int dirY[8] = { 0,  0,  1, -1,  1, -1,  1, -1 };
    constexpr double SQRT2 = 1.4142135623730950488;

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        if (current.x == goalX && current.y == goalY) {
            return reconstructPath(snapshot, cameFrom, goalX, goalY, gScore[cellIndex(goalX, goalY)], gridVersion);
        }

        size_t currIdx = cellIndex(current.x, current.y);
        if (closed[currIdx]) continue;
        closed[currIdx] = true;

        for (int i = 0; i < 8; ++i) {
            if (!isMoveValid(snapshot, current.x, current.y, dirX[i], dirY[i])) continue;

            int nx = current.x + dirX[i];
            int ny = current.y + dirY[i];
            size_t nIdx = cellIndex(nx, ny);
            if (closed[nIdx]) continue;

            double stepCost = (dirX[i] == 0 || dirY[i] == 0) ? cellSize : (cellSize * SQRT2);
            double tentativeG = current.g + stepCost;

            if (tentativeG < gScore[nIdx]) {
                gScore[nIdx] = tentativeG;
                cameFrom[nIdx] = static_cast<int>(currIdx);
                openSet.push({nx, ny, tentativeG + heuristic(nx, ny, goalX, goalY, cellSize), tentativeG});
            }
        }
    }

    return std::nullopt;
}

} // namespace domain
} // namespace mesh
} // namespace simulator
