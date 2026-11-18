#include "../../include/events/ReplanEvent.hpp"
#include "../../include/events/WaypointArrivalEvent.hpp"
#include "../../include/events/FaultRecoveryEvent.hpp"
#include "../../include/domain/Dispatcher.hpp"
#include "../../include/domain/Drone.hpp"
#include "../../include/domain/Mission.hpp"
#include "../../include/domain/Grid.hpp"
#include "../../include/domain/PathPlanner.hpp"
#include "../../include/engine/Simulation.hpp"
#include <cmath>
#include <algorithm>

namespace simulator {
namespace mesh {
namespace events {

namespace {

void handleReplanFailure(domain::Drone* drone, const std::string& droneId,
                         long long timestamp, domain::Dispatcher* dispatcher,
                         engine::Simulation* simulation) {
    simulation->getMetrics()->replanFailuresCount.fetch_add(1, std::memory_order_relaxed);
    drone->handleFault();
    simulation->scheduleSimulationEvent(std::make_unique<FaultRecoveryEvent>(
        timestamp + 3000, 5, droneId, dispatcher, simulation));
}

} // anonymous namespace

ReplanEvent::ReplanEvent(long long timestamp, int priority,
                         const std::string& droneId,
                         uint64_t gridVersionAtTrigger,
                         int retries,
                         domain::Dispatcher* dispatcher,
                         engine::Simulation* simulation)
    : timestamp(timestamp), priority(priority), droneId(droneId),
      gridVersionAtTrigger(gridVersionAtTrigger), retries(retries),
      dispatcher(dispatcher), simulation(simulation) {}

void ReplanEvent::execute() {
    if (!dispatcher || !simulation) return;

    domain::Drone* drone = dispatcher->findDroneById(droneId);
    if (!drone || drone->getState() != domain::Drone::DroneState::REROUTING) return;

    domain::Mission* mission = drone->getCurrentMission();
    domain::Grid* grid = simulation->getGrid();
    if (!mission || !grid) return;

    domain::Grid snap = grid->snapshot();
    auto optPath = domain::PathPlanner::plan(snap, drone->getCurrentLocation(), mission->getDropoffLocation());

    if (!optPath.has_value()) {
        handleReplanFailure(drone, droneId, timestamp, dispatcher, simulation);
        return;
    }

    // Stale-plan protection: if live grid advanced during planning, retry up to 3 times
    if (grid->getVersion() != snap.getVersion()) {
        if (retries < 3) {
            simulation->scheduleSimulationEvent(std::make_unique<ReplanEvent>(
                timestamp, priority, droneId, grid->getVersion(), retries + 1, dispatcher, simulation));
        } else {
            handleReplanFailure(drone, droneId, timestamp, dispatcher, simulation);
        }
        return;
    }

    // Apply replanned route to drone
    drone->setActivePath(optPath.value());
    drone->setCurrentWaypointIndex(0);
    uint64_t newRouteGen = drone->incrementRouteGeneration();
    drone->updateState(domain::Drone::DroneState::EN_ROUTE);

    if (optPath->waypoints.size() > 1) {
        double speed = simulation->getConfig().droneSpeed > 0 ? simulation->getConfig().droneSpeed : 15.0;
        double dist = optPath->waypoints[0].distanceTo(optPath->waypoints[1]);
        long long dt = std::max(1LL, static_cast<long long>(std::round((dist / speed) * 1000.0)));
        simulation->scheduleSimulationEvent(std::make_unique<WaypointArrivalEvent>(
            timestamp + dt, 5, droneId, newRouteGen, 1, dispatcher, simulation));
    }
}

} // namespace events
} // namespace mesh
} // namespace simulator
