#ifndef DRONE_MESH_FAULTEVENT_HPP
#define DRONE_MESH_FAULTEVENT_HPP

#include "Event.hpp"
#include "../domain/Drone.hpp"

namespace simulator {
namespace mesh {
namespace events {

class FaultEvent : public Event {
private:
    long long timestamp;
    int priority;
    int errorCode;
    
    // We store a pointer to the Drone to mutate its state directly in execute().
    // A pointer is chosen for minimal dependency; an ID would require a registry lookup.
    domain::Drone* targetDrone;

public:
    static constexpr int BATTERY_CRITICAL = 1001;

    FaultEvent(long long timestamp, int priority, int errorCode, domain::Drone* targetDrone);
    
    void execute() override;
    long long getTimestamp() const override;
    int getPriority() const override;
    int getErrorCode() const;
    const char* getEventType() const override { return "FaultEvent"; }
};

} // namespace events
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_FAULTEVENT_HPP
