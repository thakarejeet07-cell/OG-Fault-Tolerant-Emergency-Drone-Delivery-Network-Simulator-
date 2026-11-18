#include "../../include/events/FaultEvent.hpp"
#include <iostream>

namespace simulator {
namespace mesh {
namespace events {

FaultEvent::FaultEvent(long long timestamp, int priority, int errorCode, domain::Drone* targetDrone)
    : timestamp(timestamp), priority(priority), errorCode(errorCode), targetDrone(targetDrone) {}

void FaultEvent::execute() {
    std::cout << "[FaultEvent] Executing at timestamp: " << timestamp
              << ", priority: " << priority 
              << ", errorCode: " << errorCode << std::endl;
              
    if (targetDrone) {
        std::cout << "[FaultEvent] Setting drone state to FAULT_RECOVERY." << std::endl;
        targetDrone->setState(domain::Drone::DroneState::FAULT_RECOVERY);
    }
}

long long FaultEvent::getTimestamp() const { return timestamp; }
int FaultEvent::getPriority() const { return priority; }
int FaultEvent::getErrorCode() const { return errorCode; }

} // namespace events
} // namespace mesh
} // namespace simulator
