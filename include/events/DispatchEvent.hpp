#ifndef DRONE_MESH_DISPATCHEVENT_HPP
#define DRONE_MESH_DISPATCHEVENT_HPP

#include "Event.hpp"
#include "../domain/Mission.hpp"

namespace simulator {
namespace mesh {
namespace domain {
class Mission;
class Dispatcher;
}

namespace events {

class DispatchEvent : public Event {
private:
    long long timestamp;
    int priority;
    
    // We store a pointer to the Mission to mutate its state directly in execute().
    // A pointer is chosen for minimal dependency.
    domain::Mission* targetMission;

    // Pointer to Dispatcher: storing a direct pointer keeps DispatchEvent decoupled
    // from global simulation singletons while enabling execute() to route missions
    // directly through Dispatcher::submitMission() or Dispatcher::evaluatePreemption().
    domain::Dispatcher* dispatcher;

public:
    DispatchEvent(long long timestamp, int priority, domain::Mission* targetMission, domain::Dispatcher* dispatcher = nullptr);
    
    void execute() override;
    long long getTimestamp() const override;
    int getPriority() const override;

    domain::Dispatcher* getDispatcher() const;
    void setDispatcher(domain::Dispatcher* d);
    domain::Mission* getTargetMission() const;
    const char* getEventType() const override { return "DispatchEvent"; }
};

} // namespace events
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_DISPATCHEVENT_HPP
