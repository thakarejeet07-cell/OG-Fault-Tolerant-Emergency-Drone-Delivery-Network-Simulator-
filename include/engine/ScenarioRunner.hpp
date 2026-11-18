#ifndef DRONE_MESH_SCENARIORUNNER_HPP
#define DRONE_MESH_SCENARIORUNNER_HPP

#include "Simulation.hpp"
#include "SimulationConfig.hpp"
#include <vector>

namespace simulator {
namespace mesh {
namespace engine {

struct InvariantResult {
    bool passed{true};
    std::vector<std::string> violations;
};

class ScenarioRunner {
public:
    static InvariantResult verifyInvariants(Simulation& sim,
                                            const std::vector<domain::Mission*>& allMissions,
                                            bool isDemoScenario);

    static bool runDemo(SimulationConfig config = SimulationConfig(), bool printMetrics = true);
    static bool runStress(SimulationConfig config = SimulationConfig(), bool printMetrics = true);
};

} // namespace engine
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_SCENARIORUNNER_HPP
