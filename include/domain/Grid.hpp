#ifndef DRONE_MESH_GRID_HPP
#define DRONE_MESH_GRID_HPP

#include <vector>
#include <shared_mutex>
#include <cstdint>
#include <utility>
#include "Coordinate.hpp"

namespace simulator {
namespace mesh {
namespace domain {

enum class CellState : uint8_t {
    FREE = 0,
    STATIC_OBSTACLE = 1,
    NO_FLY_ZONE = 2
};

class Grid {
private:
    int width{0};
    int height{0};
    double cellSizeMeters{10.0};
    uint64_t version{0};

    std::vector<CellState> cells;
    std::vector<int> noFlyRefCounts;

    mutable std::shared_mutex gridMutex;

    inline size_t index(int x, int y) const {
        return static_cast<size_t>(y * width + x);
    }

public:
    Grid() = default;
    Grid(int width, int height, double cellSizeMeters = 10.0);

    // Copy constructor and assignment operator for immutable snapshotting
    Grid(const Grid& other);
    Grid& operator=(const Grid& other);

    // Move operations
    Grid(Grid&& other) noexcept;
    Grid& operator=(Grid&& other) noexcept;

    ~Grid() = default;

    int getWidth() const;
    int getHeight() const;
    double getCellSizeMeters() const;
    uint64_t getVersion() const;

    bool isValidCell(int x, int y) const;
    CellState getCellState(int x, int y) const;
    bool isPassable(int x, int y) const;

    // Mutation methods (exclusive lock)
    void setStaticObstacle(int x, int y);
    bool addNoFlyZoneCell(int x, int y);
    bool removeNoFlyZoneCell(int x, int y);

    // Coordinate conversions
    // Center of cell (x, y)
    Coordinate cellToCoordinate(int x, int y) const;
    // Nearest cell (x, y) for coordinate
    std::pair<int, int> coordinateToCell(const Coordinate& coord) const;

    // Snapshot creation: copies grid under shared_lock
    Grid snapshot() const;
};

} // namespace domain
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_GRID_HPP
