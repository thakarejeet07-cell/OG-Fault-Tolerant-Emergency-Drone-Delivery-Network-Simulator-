#ifndef DRONE_MESH_NOFLYZONEEVENT_HPP
#define DRONE_MESH_NOFLYZONEEVENT_HPP

#include "../events/Event.hpp"
#include "../domain/Coordinate.hpp"

namespace simulator {
namespace mesh {

namespace domain {
class Grid;
class Dispatcher;
}

namespace engine {
class Simulation;
}

namespace hazards {

class ZoneClearEvent : public events::Event {
private:
    long long timestamp;
    int priority;
    domain::Coordinate center;
    double radiusMeters;
    domain::Grid* grid;
    domain::Dispatcher* dispatcher;
    engine::Simulation* simulation;

public:
    ZoneClearEvent(long long timestamp, int priority, const domain::Coordinate& center,
                   double radiusMeters, domain::Grid* grid,
                   domain::Dispatcher* dispatcher, engine::Simulation* simulation);

    void execute() override;
    long long getTimestamp() const override { return timestamp; }
    int getPriority() const override { return priority; }
    const char* getEventType() const override { return "ZoneClearEvent"; }

    const domain::Coordinate& getCenter() const { return center; }
    double getRadiusMeters() const { return radiusMeters; }
    engine::Simulation* getSimulation() const { return simulation; }
};

class NoFlyZoneEvent : public events::Event {
private:
    long long timestamp;
    int priority;
    domain::Coordinate center;
    double radiusMeters;
    long long duration;
    domain::Grid* grid;
    domain::Dispatcher* dispatcher;
    engine::Simulation* simulation;

public:
    NoFlyZoneEvent(long long timestamp, int priority, const domain::Coordinate& center,
                   double radiusMeters, long long duration, domain::Grid* grid,
                   domain::Dispatcher* dispatcher, engine::Simulation* simulation);

    void execute() override;
    long long getTimestamp() const override { return timestamp; }
    int getPriority() const override { return priority; }
    const char* getEventType() const override { return "NoFlyZoneEvent"; }

    const domain::Coordinate& getCenter() const { return center; }
    double getRadiusMeters() const { return radiusMeters; }
    long long getDuration() const { return duration; }
};

} // namespace hazards
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_NOFLYZONEEVENT_HPP
