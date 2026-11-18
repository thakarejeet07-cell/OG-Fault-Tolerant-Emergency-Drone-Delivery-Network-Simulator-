#ifndef DRONE_MESH_MISSION_HPP
#define DRONE_MESH_MISSION_HPP

#include <string>
#include "Coordinate.hpp"
#include "Priority.hpp"

namespace simulator {
namespace mesh {
namespace domain {

// PRIORITY DIRECTION SPECIFICATION & CONFLICT RESOLUTION:
// Documentation conflict: UC5 specifies "Tier 0 = Critical" (lower number = more urgent),
// whereas system object diagrams, EventScheduler (EventComparator orders higher priority first
// in a max-heap), and DispatchRecord use higher numeric values for higher urgency (e.g.
// Heart Transplant = 10, Blood Samples = 3).
// Resolution: We maintain HIGHER NUMERIC VALUE = MORE URGENT simulator-wide.
// To support UC5 terminology ("Tier 0 override"), we provide PriorityTier where
// PriorityTier::CRITICAL (10) represents Tier 0 critical override missions.
class Mission {
public:
    enum class PriorityTier : int {
        LOW = 1,
        NORMAL = 3,       // Standard delivery (e.g. Blood Samples)
        HIGH = 7,
        CRITICAL = 10     // UC5 Tier 0 override (e.g. Organ / Heart Transplant)
    };

    static constexpr int PRIORITY_LOW = static_cast<int>(PriorityTier::LOW);
    static constexpr int PRIORITY_NORMAL = static_cast<int>(PriorityTier::NORMAL);
    static constexpr int PRIORITY_HIGH = static_cast<int>(PriorityTier::HIGH);
    static constexpr int PRIORITY_CRITICAL = static_cast<int>(PriorityTier::CRITICAL);

    enum class MissionState {
        PENDING,
        ACTIVE,
        COMPLETED,
        ABORTED
    };
    using MissionStatus = MissionState;

private:
    std::string missionId;
    int priority{static_cast<int>(PriorityTier::NORMAL)};
    int creationOrder{0};
    Coordinate pickupLocation;
    Coordinate dropoffLocation;
    MissionState state{MissionState::PENDING};
    bool preempted{false};

    // STATIC VARIABLE: shared by every Mission instance, not per-object.
    // Tracks how many Mission objects have ever been constructed in the simulation.
    static int missionsCreated;

public:
    // Constructors
    Mission(const std::string& missionId);
    Mission(const std::string& missionId, int priority,
            const Coordinate& pickup = Coordinate(),
            const Coordinate& dropoff = Coordinate());

    ~Mission() = default;

    // Getters and Setters
    const std::string& getMissionId() const;

    int getPriority() const;
    void setPriority(int p);
    Priority getMissionPriority() const;
    void setPriority(Priority p);

    // EXPLICIT conversion operator to std::string:
    // Returns e.g. "MSN-001[NORMAL,PENDING]"
    explicit operator std::string() const;

    int getCreationOrder() const;

    const Coordinate& getPickupLocation() const;
    void setPickupLocation(const Coordinate& coord);

    const Coordinate& getDropoffLocation() const;
    void setDropoffLocation(const Coordinate& coord);

    MissionState getState() const;
    void setState(MissionState newState);

    bool isCritical() const;

    bool wasPreempted() const;
    void setPreempted(bool p);

    // STATIC FUNCTION: callable as Mission::getMissionsCreated(), no object
    // required, since it only touches the static variable above.
    static int getMissionsCreated();
};

} // namespace domain
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_MISSION_HPP
