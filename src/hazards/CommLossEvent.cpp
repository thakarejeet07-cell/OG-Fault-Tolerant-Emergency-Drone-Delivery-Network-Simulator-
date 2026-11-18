#include "../../include/hazards/CommLossEvent.hpp"
#include "../../include/domain/Dispatcher.hpp"
#include "../../include/domain/Drone.hpp"
#include "../../include/engine/Simulation.hpp"
#include "../../include/concurrency/Logger.hpp"

namespace simulator {
namespace mesh {
namespace hazards {

// --- CommRestoreEvent ---
CommRestoreEvent::CommRestoreEvent(long long timestamp, int priority,
                                   const std::string& droneId,
                                   domain::Dispatcher* dispatcher,
                                   engine::Simulation* simulation)
    : timestamp(timestamp),
      priority(priority),
      droneId(droneId),
      dispatcher(dispatcher),
      simulation(simulation) {}

void CommRestoreEvent::execute() {
    if (!dispatcher) return;

    domain::Drone* drone = dispatcher->findDroneById(droneId);
    if (!drone) return;

    util::log("[Hazards] CommRestoreEvent at t=", timestamp, " for drone ", droneId);
    drone->setCommsLost(false);

    if (drone->getState() == domain::Drone::DroneState::FAULT_RECOVERY) {
        drone->updateState(domain::Drone::DroneState::IDLE);
        drone->setBatteryLevel(100.0); // Recovered and recharged
        dispatcher->checkAndDispatchQueuedMissions();
    }
}

// --- CommLossEvent ---
CommLossEvent::CommLossEvent(long long timestamp, int priority,
                             const std::string& droneId,
                             long long duration,
                             domain::Dispatcher* dispatcher,
                             engine::Simulation* simulation)
    : timestamp(timestamp),
      priority(priority),
      droneId(droneId),
      duration(duration),
      dispatcher(dispatcher),
      simulation(simulation) {}

void CommLossEvent::execute() {
    if (!dispatcher) return;

    domain::Drone* drone = dispatcher->findDroneById(droneId);
    if (!drone) return;

    util::log("[Hazards] CommLossEvent at t=", timestamp, " for drone ", droneId,
              " duration=", duration, "ms");
    drone->setCommsLost(true);

    if (simulation) {
        simulation->scheduleSimulationEvent(std::make_unique<CommRestoreEvent>(
            timestamp + duration, priority, droneId, dispatcher, simulation));
    }
}

} // namespace hazards
} // namespace mesh
} // namespace simulator
