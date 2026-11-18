#include "../../include/events/HeartbeatTimeoutEvent.hpp"
#include "../../include/domain/Dispatcher.hpp"
#include "../../include/domain/Drone.hpp"
#include "../../include/engine/Simulation.hpp"

namespace simulator {
namespace mesh {
namespace events {

HeartbeatTimeoutEvent::HeartbeatTimeoutEvent(long long timestamp, int priority,
                                             const std::string& droneId,
                                             uint64_t generation,
                                             domain::Dispatcher* dispatcher,
                                             engine::Simulation* simulation)
    : timestamp(timestamp),
      priority(priority),
      droneId(droneId),
      generation(generation),
      dispatcher(dispatcher),
      simulation(simulation) {}

void HeartbeatTimeoutEvent::execute() {
    if (!dispatcher) return;

    domain::Drone* drone = dispatcher->findDroneById(droneId);
    if (!drone) return;

    // Check if generation matches; stale timeouts are no-ops
    if (drone->getHeartbeatGeneration() != generation) {
        return;
    }

    // Heartbeat timed out while drone is EN_ROUTE
    if (drone->getState() == domain::Drone::DroneState::EN_ROUTE) {
        if (drone->isCommsLost()) {
            if (simulation) {
                simulation->getMetrics()->commLossFaultsCount.fetch_add(1, std::memory_order_relaxed);
            }
            // Drop drone and reallocate mission via handleFault() path
            drone->handleFault();
        }
    }
}

} // namespace events
} // namespace mesh
} // namespace simulator
