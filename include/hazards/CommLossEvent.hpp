#ifndef DRONE_MESH_COMMLOSSEVENT_HPP
#define DRONE_MESH_COMMLOSSEVENT_HPP

#include "../events/Event.hpp"
#include <string>

namespace simulator {
namespace mesh {

namespace domain {
class Dispatcher;
}

namespace engine {
class Simulation;
}

namespace hazards {

class CommRestoreEvent : public events::Event {
private:
    long long timestamp;
    int priority;
    std::string droneId;
    domain::Dispatcher* dispatcher;
    engine::Simulation* simulation;

public:
    CommRestoreEvent(long long timestamp, int priority, const std::string& droneId,
                     domain::Dispatcher* dispatcher, engine::Simulation* simulation);

    void execute() override;
    long long getTimestamp() const override { return timestamp; }
    int getPriority() const override { return priority; }
    const char* getEventType() const override { return "CommRestoreEvent"; }

    const std::string& getDroneId() const { return droneId; }
    engine::Simulation* getSimulation() const { return simulation; }
};

class CommLossEvent : public events::Event {
private:
    long long timestamp;
    int priority;
    std::string droneId;
    long long duration;
    domain::Dispatcher* dispatcher;
    engine::Simulation* simulation;

public:
    CommLossEvent(long long timestamp, int priority, const std::string& droneId,
                  long long duration, domain::Dispatcher* dispatcher,
                  engine::Simulation* simulation);

    void execute() override;
    long long getTimestamp() const override { return timestamp; }
    int getPriority() const override { return priority; }
    const char* getEventType() const override { return "CommLossEvent"; }

    const std::string& getDroneId() const { return droneId; }
    long long getDuration() const { return duration; }
};

} // namespace hazards
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_COMMLOSSEVENT_HPP
