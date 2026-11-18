#ifndef DRONE_MESH_WAYPOINTARRIVALEVENT_HPP
#define DRONE_MESH_WAYPOINTARRIVALEVENT_HPP

#include "Event.hpp"
#include <string>
#include <cstddef>

namespace simulator {
namespace mesh {

namespace domain {
class Dispatcher;
}

namespace engine {
class Simulation;
}

namespace events {

class WaypointArrivalEvent : public Event {
private:
    long long timestamp;
    int priority;
    std::string droneId;
    uint64_t routeGeneration;
    size_t waypointIndex;
    domain::Dispatcher* dispatcher;
    engine::Simulation* simulation;

public:
    WaypointArrivalEvent(long long timestamp, int priority, const std::string& droneId,
                         uint64_t routeGeneration, size_t waypointIndex,
                         domain::Dispatcher* dispatcher, engine::Simulation* simulation);

    void execute() override;
    long long getTimestamp() const override { return timestamp; }
    int getPriority() const override { return priority; }
    const char* getEventType() const override { return "WaypointArrivalEvent"; }

    const std::string& getDroneId() const { return droneId; }
    uint64_t getRouteGeneration() const { return routeGeneration; }
    size_t getWaypointIndex() const { return waypointIndex; }
};

} // namespace events
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_WAYPOINTARRIVALEVENT_HPP
