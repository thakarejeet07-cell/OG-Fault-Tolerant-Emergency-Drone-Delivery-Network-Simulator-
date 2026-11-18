#ifndef DRONE_MESH_SIMULATIONCONFIG_HPP
#define DRONE_MESH_SIMULATIONCONFIG_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace simulator {
namespace mesh {
namespace engine {

struct SimulationConfig {
    int numDrones{5};
    int gridWidth{50};
    int gridHeight{50};
    double cellSizeMeters{10.0};
    double batteryDrainRate{0.05}; // % per meter traveled
    double droneSpeed{15.0};        // meters per second
    
    // Hazard probabilities / rates
    double commLossProb{0.1};
    double batteryDrainProb{0.1};
    double noFlyZoneProb{0.1};

    uint64_t seed{42};
    int workerThreadCount{4};
    long long simulationDurationMs{60000}; // 60s simulated time default

    std::vector<std::string> validate() const {
        std::vector<std::string> errors;
        if (numDrones <= 0) {
            errors.push_back("numDrones must be positive.");
        }
        if (gridWidth <= 0 || gridHeight <= 0) {
            errors.push_back("grid dimensions (gridWidth, gridHeight) must be positive.");
        }
        if (cellSizeMeters <= 0.0) {
            errors.push_back("cellSizeMeters must be positive.");
        }
        if (batteryDrainRate < 0.0) {
            errors.push_back("batteryDrainRate cannot be negative.");
        }
        if (droneSpeed <= 0.0) {
            errors.push_back("droneSpeed must be positive.");
        }
        if (commLossProb < 0.0 || commLossProb > 1.0) {
            errors.push_back("commLossProb must be between 0.0 and 1.0.");
        }
        if (batteryDrainProb < 0.0 || batteryDrainProb > 1.0) {
            errors.push_back("batteryDrainProb must be between 0.0 and 1.0.");
        }
        if (noFlyZoneProb < 0.0 || noFlyZoneProb > 1.0) {
            errors.push_back("noFlyZoneProb must be between 0.0 and 1.0.");
        }
        if (workerThreadCount <= 0) {
            errors.push_back("workerThreadCount must be positive.");
        }
        if (simulationDurationMs <= 0) {
            errors.push_back("simulationDurationMs must be positive.");
        }
        return errors;
    }

    bool isValid() const {
        return validate().empty();
    }
};

} // namespace engine
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_SIMULATIONCONFIG_HPP
