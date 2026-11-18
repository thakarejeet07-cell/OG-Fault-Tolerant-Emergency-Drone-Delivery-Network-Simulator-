#include "../../include/domain/Drone.hpp"
#include <stdexcept>

namespace simulator {
namespace mesh {
namespace domain {

// Static variable definition (one per program, not per-object).
int Drone::fleetCount = 0;

// Constructor
Drone::Drone(const std::string& droneId, const Coordinate& startLocation)
    : droneId(droneId),
      currentLocation(startLocation),
      batteryLevel(100.0),
      state(DroneState::IDLE),
      currentMission(nullptr),
      commsLost(false),
      heartbeatGeneration(0),
      routeGeneration(0),
      currentWaypointIndex(0) {
    ++fleetCount;
}

// Destructor: keeps the fleet count accurate as drones are destroyed
Drone::~Drone() {
    --fleetCount;
}

bool Drone::isValidTransition(DroneState from, DroneState to) {
    switch (from) {
        case DroneState::IDLE:
            return to == DroneState::EN_ROUTE;
        case DroneState::EN_ROUTE:
            return to == DroneState::REROUTING ||
                   to == DroneState::FAULT_RECOVERY ||
                   to == DroneState::IDLE;
        case DroneState::REROUTING:
            return to == DroneState::EN_ROUTE ||
                   to == DroneState::FAULT_RECOVERY;
        case DroneState::FAULT_RECOVERY:
            return to == DroneState::IDLE;
        default:
            return false;
    }
}

const std::string& Drone::getDroneId() const {
    return droneId;
}

Coordinate Drone::getCurrentLocation() const {
    std::lock_guard<std::mutex> lock(droneMutex);
    return currentLocation;
}

void Drone::setCurrentLocation(const Coordinate& loc) {
    std::lock_guard<std::mutex> lock(droneMutex);
    currentLocation = loc;
}

double Drone::getBatteryLevel() const {
    std::lock_guard<std::mutex> lock(droneMutex);
    return batteryLevel;
}

void Drone::setBatteryLevel(double level) {
    std::lock_guard<std::mutex> lock(droneMutex);
    BatteryLevel b(level);
    batteryLevel = static_cast<double>(static_cast<float>(b));
}

BatteryLevel Drone::getBattery() const {
    std::lock_guard<std::mutex> lock(droneMutex);
    return BatteryLevel(batteryLevel);
}

void Drone::setBatteryLevel(BatteryLevel level) {
    std::lock_guard<std::mutex> lock(droneMutex);
    batteryLevel = static_cast<double>(static_cast<float>(level));
}

Drone::DroneState Drone::getState() const {
    std::lock_guard<std::mutex> lock(droneMutex);
    return state;
}

bool Drone::updateState(DroneState newState) {
    std::lock_guard<std::mutex> lock(droneMutex);
    if (!isValidTransition(state, newState)) {
        return false;
    }
    state = newState;
    return true;
}

void Drone::setState(DroneState newState) {
    if (!updateState(newState)) {
        throw std::logic_error("Illegal drone state transition");
    }
}

Mission* Drone::getCurrentMission() const {
    std::lock_guard<std::mutex> lock(droneMutex);
    return currentMission;
}

void Drone::setCurrentMission(Mission* mission) {
    std::lock_guard<std::mutex> lock(droneMutex);
    currentMission = mission;
}

bool Drone::isCommsLost() const {
    std::lock_guard<std::mutex> lock(droneMutex);
    return commsLost;
}

void Drone::setCommsLost(bool lost) {
    std::lock_guard<std::mutex> lock(droneMutex);
    commsLost = lost;
}

uint64_t Drone::getHeartbeatGeneration() const {
    std::lock_guard<std::mutex> lock(droneMutex);
    return heartbeatGeneration;
}

uint64_t Drone::nextHeartbeatGeneration() {
    std::lock_guard<std::mutex> lock(droneMutex);
    return ++heartbeatGeneration;
}

uint64_t Drone::getRouteGeneration() const {
    std::lock_guard<std::mutex> lock(droneMutex);
    return routeGeneration;
}

uint64_t Drone::incrementRouteGeneration() {
    std::lock_guard<std::mutex> lock(droneMutex);
    return ++routeGeneration;
}

Path Drone::getActivePath() const {
    std::lock_guard<std::mutex> lock(droneMutex);
    return activePath;
}

void Drone::setActivePath(const Path& path) {
    std::lock_guard<std::mutex> lock(droneMutex);
    activePath = path;
}

size_t Drone::getCurrentWaypointIndex() const {
    std::lock_guard<std::mutex> lock(droneMutex);
    return currentWaypointIndex;
}

void Drone::setCurrentWaypointIndex(size_t index) {
    std::lock_guard<std::mutex> lock(droneMutex);
    currentWaypointIndex = index;
}

Mission* Drone::handleFault() {
    Mission* released = nullptr;
    FaultCallback cb;
    {
        std::lock_guard<std::mutex> lock(droneMutex);
        if (!isValidTransition(state, DroneState::FAULT_RECOVERY)) {
            return nullptr;
        }
        state = DroneState::FAULT_RECOVERY;
        released = currentMission;
        currentMission = nullptr;
        cb = faultCallback;
    }
    if (cb) {
        cb(this, released);
    }
    return released;
}

bool Drone::triggerCollisionAvoidance() {
    std::lock_guard<std::mutex> lock(droneMutex);
    if (!isValidTransition(state, DroneState::REROUTING)) {
        return false;
    }
    state = DroneState::REROUTING;
    // Local safety only: do not touch currentMission
    return true;
}

void Drone::setFaultCallback(FaultCallback callback) {
    std::lock_guard<std::mutex> lock(droneMutex);
    faultCallback = std::move(callback);
}

// STATIC FUNCTION: operates only on the static variable, no instance needed.
int Drone::getFleetCount() {
    return fleetCount;
}

// Battery drain method
void Drone::drainBattery(double amount) {
    std::lock_guard<std::mutex> lock(droneMutex);
    BatteryLevel b(batteryLevel - amount);
    batteryLevel = static_cast<double>(static_cast<float>(b));
}

Drone::operator std::string() const {
    std::lock_guard<std::mutex> lock(droneMutex);
    std::string stateStr;
    switch (state) {
        case DroneState::IDLE: stateStr = "IDLE"; break;
        case DroneState::EN_ROUTE: stateStr = "EN_ROUTE"; break;
        case DroneState::REROUTING: stateStr = "REROUTING"; break;
        case DroneState::FAULT_RECOVERY: stateStr = "FAULT_RECOVERY"; break;
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f%%", batteryLevel);
    return droneId + "[" + stateStr + "," + buf + "]";
}

} // namespace domain
} // namespace mesh
} // namespace simulator
