#include "../../include/domain/Mission.hpp"
#include <iostream>

namespace simulator {
namespace mesh {
namespace domain {

// Static variables must be DEFINED once at namespace scope, outside the
// class, even though they're DECLARED inside the class in the header.
int Mission::missionsCreated = 0;

Mission::Mission(const std::string& missionId)
    : missionId(missionId),
      priority(static_cast<int>(PriorityTier::NORMAL)),
      state(MissionState::PENDING),
      preempted(false) {
    ++missionsCreated;
    creationOrder = missionsCreated;
}

Mission::Mission(
    const std::string& missionId,
    int priority,
    const Coordinate& pickup,
    const Coordinate& dropoff
) : missionId(missionId),
    priority(priority),
    pickupLocation(pickup),
    dropoffLocation(dropoff),
    state(MissionState::PENDING),
    preempted(false) {
    ++missionsCreated;
    creationOrder = missionsCreated;
}

const std::string& Mission::getMissionId() const {
    return missionId;
}

int Mission::getPriority() const {
    return priority;
}

void Mission::setPriority(int p) {
    priority = p;
}

Priority Mission::getMissionPriority() const {
    return Priority(priority);
}

void Mission::setPriority(Priority p) {
    priority = static_cast<int>(p);
}

Mission::operator std::string() const {
    std::string stateStr;
    switch (state) {
        case MissionState::PENDING: stateStr = "PENDING"; break;
        case MissionState::ACTIVE: stateStr = "ACTIVE"; break;
        case MissionState::COMPLETED: stateStr = "COMPLETED"; break;
        case MissionState::ABORTED: stateStr = "ABORTED"; break;
    }
    std::string prioStr;
    if (priority >= Priority::CRITICAL_VAL) prioStr = "CRITICAL";
    else if (priority >= Priority::HIGH_VAL) prioStr = "HIGH";
    else if (priority >= Priority::NORMAL_VAL) prioStr = "NORMAL";
    else prioStr = "LOW";

    return missionId + "[" + prioStr + "," + stateStr + "]";
}

int Mission::getCreationOrder() const {
    return creationOrder;
}

const Coordinate& Mission::getPickupLocation() const {
    return pickupLocation;
}

void Mission::setPickupLocation(const Coordinate& coord) {
    pickupLocation = coord;
}

const Coordinate& Mission::getDropoffLocation() const {
    return dropoffLocation;
}

void Mission::setDropoffLocation(const Coordinate& coord) {
    dropoffLocation = coord;
}

Mission::MissionState Mission::getState() const {
    return state;
}

void Mission::setState(Mission::MissionState newState) {
    state = newState;
}

bool Mission::isCritical() const {
    return priority >= static_cast<int>(PriorityTier::CRITICAL);
}

bool Mission::wasPreempted() const {
    return preempted;
}

void Mission::setPreempted(bool p) {
    preempted = p;
}

int Mission::getMissionsCreated() {
    return missionsCreated;
}

} // namespace domain
} // namespace mesh
} // namespace simulator
