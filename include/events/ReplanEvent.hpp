#ifndef DRONE_MESH_REPLANEVENT_HPP
#define DRONE_MESH_REPLANEVENT_HPP

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

class ReplanEvent : public Event {
private:
    long long timestamp;
    int priority;
    std::string droneId;
    uint64_t gridVersionAtTrigger;
    int retries{0};
    domain::Dispatcher* dispatcher;
    engine::Simulation* simulation;

public:
    ReplanEvent(long long timestamp, int priority, const std::string& droneId,
                uint64_t gridVersionAtTrigger, int retries,
                domain::Dispatcher* dispatcher, engine::Simulation* simulation);

    void execute() override;
    long long getTimestamp() const override { return timestamp; }
    int getPriority() const override { return priority; }
    const char* getEventType() const override { return "ReplanEvent"; }

    const std::string& getDroneId() const { return droneId; }
    uint64_t getGridVersionAtTrigger() const { return gridVersionAtTrigger; }
    int getRetries() const { return retries; }
};

} // namespace events
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_REPLANEVENT_HPP
