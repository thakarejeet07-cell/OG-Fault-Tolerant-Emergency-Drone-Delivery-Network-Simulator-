#include "../../include/hazards/BatteryDrainEvent.hpp"
#include "../../include/domain/Dispatcher.hpp"
#include "../../include/domain/Drone.hpp"
#include "../../include/events/FaultEvent.hpp"
#include "../../include/events/FaultRecoveryEvent.hpp"
#include "../../include/engine/Simulation.hpp"
#include "../../include/concurrency/Logger.hpp"

namespace simulator {
namespace mesh {
namespace hazards {

BatteryDrainEvent::BatteryDrainEvent(long long timestamp, int priority,
                                     const std::string& droneId,
                                     double amount,
                                     domain::Dispatcher* dispatcher,
                                     engine::Simulation* simulation)
    : timestamp(timestamp),
      priority(priority),
      droneId(droneId),
      amount(amount),
      dispatcher(dispatcher),
      simulation(simulation) {}

void BatteryDrainEvent::execute() {
    if (!dispatcher) return;

    domain::Drone* drone = dispatcher->findDroneById(droneId);
    if (!drone) return;

    util::log("[Hazards] BatteryDrainEvent at t=", timestamp, " for drone ", droneId,
              " drain=", amount, "%");
    drone->drainBattery(amount);

    if (drone->getBatteryLevel() < BATTERY_CRITICAL_THRESHOLD) {
        if (simulation) {
            simulation->getMetrics()->batteryFaultsCount.fetch_add(1, std::memory_order_relaxed);
            simulation->scheduleSimulationEvent(std::make_unique<events::FaultEvent>(
                timestamp, 10, events::FaultEvent::BATTERY_CRITICAL, drone));
            simulation->scheduleSimulationEvent(std::make_unique<events::FaultRecoveryEvent>(
                timestamp + 4000, 5, droneId, dispatcher, simulation));
        } else {
            drone->handleFault();
        }
    }
}

} // namespace hazards
} // namespace mesh
} // namespace simulator
