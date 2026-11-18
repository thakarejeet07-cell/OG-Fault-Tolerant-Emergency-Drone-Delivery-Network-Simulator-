#ifndef DRONE_MESH_HEARTBEATTIMEOUTEVENT_HPP
#define DRONE_MESH_HEARTBEATTIMEOUTEVENT_HPP

#include "Event.hpp"
#include <string>
#include <cstdint>

namespace simulator {
namespace mesh {

namespace domain {
class Dispatcher;
}

namespace engine {
class Simulation;
}

namespace events {

class HeartbeatTimeoutEvent : public Event {
private:
    long long timestamp;
    int priority;
    std::string droneId;
    uint64_t generation;
    domain::Dispatcher* dispatcher;
    engine::Simulation* simulation;

public:
    HeartbeatTimeoutEvent(long long timestamp, int priority, const std::string& droneId,
                          uint64_t generation, domain::Dispatcher* dispatcher,
                          engine::Simulation* simulation);

    void execute() override;
    long long getTimestamp() const override { return timestamp; }
    int getPriority() const override { return priority; }
    const char* getEventType() const override { return "HeartbeatTimeoutEvent"; }

    const std::string& getDroneId() const { return droneId; }
    uint64_t getGeneration() const { return generation; }
};

} // namespace events
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_HEARTBEATTIMEOUTEVENT_HPP
