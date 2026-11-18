#include "../../include/hazards/HazardGenerator.hpp"
#include "../../include/hazards/CommLossEvent.hpp"
#include "../../include/hazards/BatteryDrainEvent.hpp"
#include "../../include/hazards/NoFlyZoneEvent.hpp"
#include "../../include/engine/Simulation.hpp"
#include <random>
#include <iostream>

namespace simulator {
namespace mesh {
namespace hazards {

namespace {

std::string formatDroneId(int id) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "DRN-%03d", id);
    return std::string(buf);
}

} // anonymous namespace

void HazardGenerator::generateHazards(const engine::SimulationConfig& config, engine::Simulation* simulation) {
    if (!simulation || config.simulationDurationMs <= 1000) return;

    std::cout << "[HazardGenerator] Initializing hazard schedule with seed: " << config.seed << std::endl;
    std::mt19937_64 rng(config.seed);

    std::uniform_int_distribution<long long> timeDist(500, config.simulationDurationMs - 1000);
    std::uniform_int_distribution<int> droneDist(1, config.numDrones);
    std::uniform_real_distribution<double> probDist(0.0, 1.0);
    std::uniform_real_distribution<double> xDist(0.0, config.gridWidth * config.cellSizeMeters);
    std::uniform_real_distribution<double> yDist(0.0, config.gridHeight * config.cellSizeMeters);

    int opportunities = static_cast<int>(std::max(1LL, config.simulationDurationMs / 5000));

    for (int i = 0; i < opportunities; ++i) {
        if (probDist(rng) < config.commLossProb) {
            long long duration = 2000 + (rng() % 3000);
            simulation->scheduleSimulationEvent(std::make_unique<CommLossEvent>(
                timeDist(rng), 7, formatDroneId(droneDist(rng)), duration, simulation->getDispatcher(), simulation));
        }
        if (probDist(rng) < config.batteryDrainProb) {
            double drain = 15.0 + (rng() % 25);
            simulation->scheduleSimulationEvent(std::make_unique<BatteryDrainEvent>(
                timeDist(rng), 6, formatDroneId(droneDist(rng)), drain, simulation->getDispatcher(), simulation));
        }
        if (probDist(rng) < config.noFlyZoneProb) {
            double radius = config.cellSizeMeters * (2.0 + (rng() % 4));
            long long duration = 3000 + (rng() % 7000);
            domain::Coordinate center(xDist(rng), yDist(rng));
            simulation->scheduleSimulationEvent(std::make_unique<NoFlyZoneEvent>(
                timeDist(rng), 7, center, radius, duration, simulation->getGrid(), simulation->getDispatcher(), simulation));
        }
    }
}

} // namespace hazards
} // namespace mesh
} // namespace simulator
