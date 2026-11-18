#include "../../include/events/FaultRecoveryEvent.hpp"
#include "../../include/domain/Dispatcher.hpp"
#include "../../include/domain/Drone.hpp"
#include "../../include/engine/Simulation.hpp"

namespace simulator {
namespace mesh {
namespace events {

FaultRecoveryEvent::FaultRecoveryEvent(long long timestamp, int priority,
                                       const std::string& droneId,
                                       domain::Dispatcher* dispatcher,
                                       engine::Simulation* simulation)
    : timestamp(timestamp),
      priority(priority),
      droneId(droneId),
      dispatcher(dispatcher),
      simulation(simulation) {}

void FaultRecoveryEvent::execute() {
    if (!dispatcher) return;

    domain::Drone* drone = dispatcher->findDroneById(droneId);
    if (!drone) return;

    // Only recover if not currently in comm loss
    if (drone->getState() == domain::Drone::DroneState::FAULT_RECOVERY && !drone->isCommsLost()) {
        drone->updateState(domain::Drone::DroneState::IDLE);
        drone->setBatteryLevel(100.0);
        dispatcher->checkAndDispatchQueuedMissions();
    }
}

} // namespace events
} // namespace mesh
} // namespace simulator
