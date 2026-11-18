#ifndef DRONE_MESH_GISPROVIDER_HPP
#define DRONE_MESH_GISPROVIDER_HPP

#include "Grid.hpp"
#include "../engine/SimulationConfig.hpp"

namespace simulator {
namespace mesh {
namespace domain {

class GISProvider {
public:
    Grid loadGrid(const engine::SimulationConfig& config);
};

} // namespace domain
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_GISPROVIDER_HPP
