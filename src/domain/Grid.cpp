#include "../../include/domain/Grid.hpp"
#include <cmath>
#include <mutex>

namespace simulator {
namespace mesh {
namespace domain {

Grid::Grid(int width, int height, double cellSizeMeters)
    : width(width),
      height(height),
      cellSizeMeters(cellSizeMeters),
      version(0),
      cells(static_cast<size_t>(width * height), CellState::FREE),
      noFlyRefCounts(static_cast<size_t>(width * height), 0) {}

Grid::Grid(const Grid& other) {
    std::shared_lock<std::shared_mutex> lock(other.gridMutex);
    width = other.width;
    height = other.height;
    cellSizeMeters = other.cellSizeMeters;
    version = other.version;
    cells = other.cells;
    noFlyRefCounts = other.noFlyRefCounts;
}

Grid& Grid::operator=(const Grid& other) {
    if (this == &other) return *this;
    // Acquire locks in a deterministic order or uniquely
    std::unique_lock<std::shared_mutex> thisLock(gridMutex, std::defer_lock);
    std::shared_lock<std::shared_mutex> otherLock(other.gridMutex, std::defer_lock);
    std::lock(thisLock, otherLock);

    width = other.width;
    height = other.height;
    cellSizeMeters = other.cellSizeMeters;
    version = other.version;
    cells = other.cells;
    noFlyRefCounts = other.noFlyRefCounts;
    return *this;
}

Grid::Grid(Grid&& other) noexcept {
    std::unique_lock<std::shared_mutex> lock(other.gridMutex);
    width = other.width;
    height = other.height;
    cellSizeMeters = other.cellSizeMeters;
    version = other.version;
    cells = std::move(other.cells);
    noFlyRefCounts = std::move(other.noFlyRefCounts);
}

Grid& Grid::operator=(Grid&& other) noexcept {
    if (this == &other) return *this;
    std::unique_lock<std::shared_mutex> thisLock(gridMutex, std::defer_lock);
    std::unique_lock<std::shared_mutex> otherLock(other.gridMutex, std::defer_lock);
    std::lock(thisLock, otherLock);

    width = other.width;
    height = other.height;
    cellSizeMeters = other.cellSizeMeters;
    version = other.version;
    cells = std::move(other.cells);
    noFlyRefCounts = std::move(other.noFlyRefCounts);
    return *this;
}

int Grid::getWidth() const {
    std::shared_lock<std::shared_mutex> lock(gridMutex);
    return width;
}

int Grid::getHeight() const {
    std::shared_lock<std::shared_mutex> lock(gridMutex);
    return height;
}

double Grid::getCellSizeMeters() const {
    std::shared_lock<std::shared_mutex> lock(gridMutex);
    return cellSizeMeters;
}

uint64_t Grid::getVersion() const {
    std::shared_lock<std::shared_mutex> lock(gridMutex);
    return version;
}

bool Grid::isValidCell(int x, int y) const {
    return x >= 0 && x < width && y >= 0 && y < height;
}

CellState Grid::getCellState(int x, int y) const {
    std::shared_lock<std::shared_mutex> lock(gridMutex);
    if (!isValidCell(x, y)) {
        return CellState::STATIC_OBSTACLE;
    }
    return cells[index(x, y)];
}

bool Grid::isPassable(int x, int y) const {
    std::shared_lock<std::shared_mutex> lock(gridMutex);
    if (!isValidCell(x, y)) {
        return false;
    }
    return cells[index(x, y)] == CellState::FREE;
}

void Grid::setStaticObstacle(int x, int y) {
    std::unique_lock<std::shared_mutex> lock(gridMutex);
    if (isValidCell(x, y)) {
        cells[index(x, y)] = CellState::STATIC_OBSTACLE;
        ++version;
    }
}

bool Grid::addNoFlyZoneCell(int x, int y) {
    std::unique_lock<std::shared_mutex> lock(gridMutex);
    if (!isValidCell(x, y)) return false;

    size_t idx = index(x, y);
    noFlyRefCounts[idx]++;
    if (cells[idx] != CellState::STATIC_OBSTACLE) {
        cells[idx] = CellState::NO_FLY_ZONE;
    }
    ++version;
    return true;
}

bool Grid::removeNoFlyZoneCell(int x, int y) {
    std::unique_lock<std::shared_mutex> lock(gridMutex);
    if (!isValidCell(x, y)) return false;

    size_t idx = index(x, y);
    if (noFlyRefCounts[idx] > 0) {
        noFlyRefCounts[idx]--;
        if (noFlyRefCounts[idx] == 0 && cells[idx] == CellState::NO_FLY_ZONE) {
            cells[idx] = CellState::FREE;
        }
        ++version;
        return true;
    }
    return false;
}

Coordinate Grid::cellToCoordinate(int x, int y) const {
    return Coordinate(static_cast<double>(x) * cellSizeMeters,
                      static_cast<double>(y) * cellSizeMeters);
}

std::pair<int, int> Grid::coordinateToCell(const Coordinate& coord) const {
    int x = static_cast<int>(std::round(coord.getLatitude() / cellSizeMeters));
    int y = static_cast<int>(std::round(coord.getLongitude() / cellSizeMeters));
    return {x, y};
}

Grid Grid::snapshot() const {
    return *this; // Invokes copy constructor with shared_lock
}

} // namespace domain
} // namespace mesh
} // namespace simulator
