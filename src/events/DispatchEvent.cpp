#include "../../include/events/DispatchEvent.hpp"
#include "../../include/domain/Dispatcher.hpp"
#include "../../include/concurrency/Logger.hpp"

namespace simulator {
namespace mesh {
namespace events {

DispatchEvent::DispatchEvent(
    long long timestamp,
    int priority,
    domain::Mission* targetMission,
    domain::Dispatcher* dispatcher
) : timestamp(timestamp),
    priority(priority),
    targetMission(targetMission),
    dispatcher(dispatcher) {}

void DispatchEvent::execute() {
    util::log("[DispatchEvent] Executing at timestamp: ", timestamp, ", priority: ", priority);

    if (!targetMission) {
        return;
    }

    if (dispatcher) {
        if (targetMission->isCritical()) {
            util::log("[DispatchEvent] Critical mission detected (", targetMission->getMissionId(), "). Calling evaluatePreemption().");
            dispatcher->evaluatePreemption(targetMission);
        } else {
            util::log("[DispatchEvent] Normal mission detected (", targetMission->getMissionId(), "). Calling submitMission().");
            dispatcher->submitMission(targetMission);
        }
    } else {
        // Fallback when no dispatcher attached (preserves existing smoke-test behavior)
        util::log("[DispatchEvent] No dispatcher attached. Setting mission state to ACTIVE directly.");
        targetMission->setState(domain::Mission::MissionState::ACTIVE);
    }
}

long long DispatchEvent::getTimestamp() const {
    return timestamp;
}

int DispatchEvent::getPriority() const {
    return priority;
}

domain::Dispatcher* DispatchEvent::getDispatcher() const {
    return dispatcher;
}

void DispatchEvent::setDispatcher(domain::Dispatcher* d) {
    dispatcher = d;
}

domain::Mission* DispatchEvent::getTargetMission() const {
    return targetMission;
}

} // namespace events
} // namespace mesh
} // namespace simulator
