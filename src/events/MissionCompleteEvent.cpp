#include "../../include/events/MissionCompleteEvent.hpp"
#include "../../include/domain/Dispatcher.hpp"
#include "../../include/domain/Drone.hpp"
#include "../../include/domain/Mission.hpp"
#include "../../include/engine/Simulation.hpp"

namespace simulator {
namespace mesh {
namespace events {

MissionCompleteEvent::MissionCompleteEvent(long long timestamp, int priority,
                                           const std::string& droneId,
                                           domain::Mission* mission,
                                           domain::Dispatcher* dispatcher,
                                           engine::Simulation* simulation)
    : timestamp(timestamp),
      priority(priority),
      droneId(droneId),
      mission(mission),
      dispatcher(dispatcher),
      simulation(simulation) {}

void MissionCompleteEvent::execute() {
    if (!dispatcher || !mission) return;

    domain::Drone* drone = dispatcher->findDroneById(droneId);
    if (!drone) return;

    if (drone->getCurrentMission() == mission &&
        drone->getState() == domain::Drone::DroneState::EN_ROUTE) {
        dispatcher->completeMission(mission, drone);
    }
}

} // namespace events
} // namespace mesh
} // namespace simulator
