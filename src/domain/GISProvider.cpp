#include "../../include/domain/GISProvider.hpp"
#include <random>

namespace simulator {
namespace mesh {
namespace domain {

Grid GISProvider::loadGrid(const engine::SimulationConfig& config) {
    Grid grid(config.gridWidth, config.gridHeight, config.cellSizeMeters);
    std::mt19937_64 rng(config.seed);

    int totalCells = config.gridWidth * config.gridHeight;
    int numObstacles = totalCells / 20; // ~5% obstacles

    std::uniform_int_distribution<int> distX(0, config.gridWidth - 1);
    std::uniform_int_distribution<int> distY(0, config.gridHeight - 1);

    for (int i = 0; i < numObstacles; ++i) {
        int x = distX(rng);
        int y = distY(rng);

        // Keep the starting area (e.g. x < 5, y < 5) free for drone deployment
        if (x < 5 && y < 5) {
            continue;
        }

        grid.setStaticObstacle(x, y);
    }

    return grid;
}

} // namespace domain
} // namespace mesh
} // namespace simulator
