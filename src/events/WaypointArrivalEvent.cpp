#include "../../include/events/WaypointArrivalEvent.hpp"
#include "../../include/events/MissionCompleteEvent.hpp"
#include "../../include/events/HeartbeatTimeoutEvent.hpp"
#include "../../include/events/FaultEvent.hpp"
#include "../../include/events/FaultRecoveryEvent.hpp"
#include "../../include/domain/Dispatcher.hpp"
#include "../../include/domain/Drone.hpp"
#include "../../include/domain/Mission.hpp"
#include "../../include/engine/Simulation.hpp"
#include <cmath>
#include <algorithm>

namespace simulator {
namespace mesh {
namespace events {

WaypointArrivalEvent::WaypointArrivalEvent(long long timestamp, int priority,
                                           const std::string& droneId,
                                           uint64_t routeGeneration,
                                           size_t waypointIndex,
                                           domain::Dispatcher* dispatcher,
                                           engine::Simulation* simulation)
    : timestamp(timestamp), priority(priority), droneId(droneId),
      routeGeneration(routeGeneration), waypointIndex(waypointIndex),
      dispatcher(dispatcher), simulation(simulation) {}

void WaypointArrivalEvent::execute() {
    if (!dispatcher || !simulation) return;

    domain::Drone* drone = dispatcher->findDroneById(droneId);
    if (!drone) return;

    // Guard: drone must be EN_ROUTE with matching route generation
    if (drone->getState() != domain::Drone::DroneState::EN_ROUTE) return;
    if (drone->getRouteGeneration() != routeGeneration) return;

    domain::Path path = drone->getActivePath();
    if (waypointIndex >= path.waypoints.size()) return;

    // Drain battery along segment
    if (waypointIndex > 0) {
        double dist = path.waypoints[waypointIndex - 1].distanceTo(path.waypoints[waypointIndex]);
        drone->drainBattery(dist * simulation->getConfig().batteryDrainRate);

        if (drone->getBatteryLevel() < 10.0) { // Battery critical fault threshold
            simulation->getMetrics()->batteryFaultsCount.fetch_add(1, std::memory_order_relaxed);
            simulation->scheduleSimulationEvent(std::make_unique<FaultEvent>(
                timestamp, 10, FaultEvent::BATTERY_CRITICAL, drone));
            simulation->scheduleSimulationEvent(std::make_unique<FaultRecoveryEvent>(
                timestamp + 4000, 5, droneId, dispatcher, simulation));
            return;
        }
    }

    drone->setCurrentLocation(path.waypoints[waypointIndex]);
    drone->setCurrentWaypointIndex(waypointIndex);

    // Refresh heartbeat timeout if communications are healthy
    if (!drone->isCommsLost()) {
        uint64_t gen = drone->nextHeartbeatGeneration();
        simulation->scheduleSimulationEvent(std::make_unique<HeartbeatTimeoutEvent>(
            timestamp + 5000, 8, droneId, gen, dispatcher, simulation));
    }

    // Advance to next waypoint or complete mission
    if (waypointIndex + 1 < path.waypoints.size()) {
        double speed = simulation->getConfig().droneSpeed > 0 ? simulation->getConfig().droneSpeed : 15.0;
        double nextDist = path.waypoints[waypointIndex].distanceTo(path.waypoints[waypointIndex + 1]);
        long long dt = std::max(1LL, static_cast<long long>(std::round((nextDist / speed) * 1000.0)));
        simulation->scheduleSimulationEvent(std::make_unique<WaypointArrivalEvent>(
            timestamp + dt, priority, droneId, routeGeneration, waypointIndex + 1, dispatcher, simulation));
    } else {
        simulation->scheduleSimulationEvent(std::make_unique<MissionCompleteEvent>(
            timestamp, 9, droneId, drone->getCurrentMission(), dispatcher, simulation));
    }
}

} // namespace events
} // namespace mesh
} // namespace simulator
