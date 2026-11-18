#ifndef DRONE_MESH_DRONE_HPP
#define DRONE_MESH_DRONE_HPP

#include <string>
#include <mutex>
#include <functional>
#include "../../include/domain/Coordinate.hpp"
#include "../../include/domain/PathPlanner.hpp"
#include "../../include/domain/BatteryLevel.hpp"

namespace simulator {
namespace mesh {
namespace domain {

class Mission; // Forward declaration for 0..1 association

class Drone {
public:
    enum class DroneState {
        IDLE,
        EN_ROUTE,
        REROUTING,
        FAULT_RECOVERY
    };

    using FaultCallback = std::function<void(Drone*, Mission*)>;

private:
    std::string droneId;
    Coordinate currentLocation;
    double batteryLevel;
    DroneState state{DroneState::IDLE};
    Mission* currentMission{nullptr};
    FaultCallback faultCallback;

    // Concurrency & fault tracking fields
    bool commsLost{false};
    uint64_t heartbeatGeneration{0};
    uint64_t routeGeneration{0};
    Path activePath;
    size_t currentWaypointIndex{0};

    // Mutex guarding drone internal state, ensuring thread-safe transitions
    mutable std::mutex droneMutex;

    // Helper for state machine transition validation
    static bool isValidTransition(DroneState from, DroneState to);

    // STATIC VARIABLE: tracks total active drones in fleet
    static int fleetCount;

public:
    // Constructor
    Drone(const std::string& droneId, const Coordinate& startLocation);

    // Destructor: decrements fleetCount
    ~Drone();

    // Prevent copy/move due to mutex and unique drone identity
    Drone(const Drone&) = delete;
    Drone& operator=(const Drone&) = delete;

    // Getters and Setters
    const std::string& getDroneId() const;

    Coordinate getCurrentLocation() const;
    void setCurrentLocation(const Coordinate& loc);

    double getBatteryLevel() const;
    void setBatteryLevel(double level);
    BatteryLevel getBattery() const;
    void setBatteryLevel(BatteryLevel level);

    // EXPLICIT conversion operator to std::string:
    // Returns log-friendly format: e.g. "DRN-001[EN_ROUTE,84.5%]"
    // Explicit to prevent unintentional string conversions or pointer decay.
    explicit operator std::string() const;
    
    DroneState getState() const;
    void setState(DroneState newState);

    // State machine update: returns false on illegal transition
    bool updateState(DroneState newState);

    // Mission association (Drone 0..1 --> Mission)
    Mission* getCurrentMission() const;
    void setCurrentMission(Mission* mission);

    // Communication loss status
    bool isCommsLost() const;
    void setCommsLost(bool lost);

    // Heartbeat generation tracking for timeout event validation
    uint64_t getHeartbeatGeneration() const;
    uint64_t nextHeartbeatGeneration();

    // Route tracking and versioning
    uint64_t getRouteGeneration() const;
    uint64_t incrementRouteGeneration();

    Path getActivePath() const;
    void setActivePath(const Path& path);

    size_t getCurrentWaypointIndex() const;
    void setCurrentWaypointIndex(size_t index);

    // Fault handling: transitions to FAULT_RECOVERY, releases current mission,
    // notifies Dispatcher via callback, and returns the released Mission*
    Mission* handleFault();

    // Collision avoidance: transitions to REROUTING without touching mission
    bool triggerCollisionAvoidance();

    void setFaultCallback(FaultCallback callback);

    // Battery drain method
    void drainBattery(double amount);

    // STATIC FUNCTION: no 'this' pointer, callable as Drone::getFleetCount()
    // even with zero drones in existence.
    static int getFleetCount();
};

} // namespace domain
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_DRONE_HPP
