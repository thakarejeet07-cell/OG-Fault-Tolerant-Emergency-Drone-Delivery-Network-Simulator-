#ifndef DRONE_MESH_FAULTRECOVERYEVENT_HPP
#define DRONE_MESH_FAULTRECOVERYEVENT_HPP

#include "Event.hpp"
#include <string>

namespace simulator {
namespace mesh {

namespace domain {
class Dispatcher;
}

namespace engine {
class Simulation;
}

namespace events {

class FaultRecoveryEvent : public Event {
private:
    long long timestamp;
    int priority;
    std::string droneId;
    domain::Dispatcher* dispatcher;
    engine::Simulation* simulation;

public:
    FaultRecoveryEvent(long long timestamp, int priority, const std::string& droneId,
                       domain::Dispatcher* dispatcher, engine::Simulation* simulation);

    void execute() override;
    long long getTimestamp() const override { return timestamp; }
    int getPriority() const override { return priority; }
    const char* getEventType() const override { return "FaultRecoveryEvent"; }

    const std::string& getDroneId() const { return droneId; }
    engine::Simulation* getSimulation() const { return simulation; }
};

} // namespace events
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_FAULTRECOVERYEVENT_HPP
