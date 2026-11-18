#ifndef DRONE_MESH_HAZARDGENERATOR_HPP
#define DRONE_MESH_HAZARDGENERATOR_HPP

#include "../engine/SimulationConfig.hpp"
#include <vector>

namespace simulator {
namespace mesh {

namespace engine {
class Simulation;
}

namespace hazards {

class HazardGenerator {
public:
    static void generateHazards(const engine::SimulationConfig& config, engine::Simulation* simulation);
};

} // namespace hazards
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_HAZARDGENERATOR_HPP
