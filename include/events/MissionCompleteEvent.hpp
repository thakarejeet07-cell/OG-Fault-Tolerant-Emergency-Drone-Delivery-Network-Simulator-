#ifndef DRONE_MESH_MISSIONCOMPLETEEVENT_HPP
#define DRONE_MESH_MISSIONCOMPLETEEVENT_HPP

#include "Event.hpp"
#include <string>

namespace simulator {
namespace mesh {

namespace domain {
class Mission;
class Dispatcher;
}

namespace engine {
class Simulation;
}

namespace events {

class MissionCompleteEvent : public Event {
private:
    long long timestamp;
    int priority;
    std::string droneId;
    domain::Mission* mission;
    domain::Dispatcher* dispatcher;
    engine::Simulation* simulation;

public:
    MissionCompleteEvent(long long timestamp, int priority, const std::string& droneId,
                         domain::Mission* mission, domain::Dispatcher* dispatcher,
                         engine::Simulation* simulation);

    void execute() override;
    long long getTimestamp() const override { return timestamp; }
    int getPriority() const override { return priority; }
    const char* getEventType() const override { return "MissionCompleteEvent"; }

    const std::string& getDroneId() const { return droneId; }
    domain::Mission* getMission() const { return mission; }
    engine::Simulation* getSimulation() const { return simulation; }
};

} // namespace events
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_MISSIONCOMPLETEEVENT_HPP
