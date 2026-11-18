#ifndef DRONE_MESH_DISPATCHER_HPP
#define DRONE_MESH_DISPATCHER_HPP

#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <map>
#include <atomic>
#include "../../include/domain/Mission.hpp"
#include "../../include/domain/Drone.hpp"

namespace simulator {
namespace mesh {

// Forward declaration for EventScheduler and Simulation to avoid circular header dependency
namespace engine {
class EventScheduler;
class Simulation;
}

namespace domain {

class Grid;

/*
 * ============================================================================
 * LOCK ORDERING HIERARCHY & DEADLOCK PREVENTION SPECIFICATION
 * ============================================================================
 * 
 * To ensure complete absence of deadlocks and data races in concurrent
 * execution under full system integration and ThreadSanitizer:
 * 
 * Lock acquisition MUST strictly follow this hierarchical level order:
 * 
 *   Level 1: Live Grid lock (domain::Grid::gridMutex - std::shared_mutex)
 *            - Held by hazard events modifying cells (unique_lock).
 *            - Held momentarily during Grid::snapshot() (shared_lock).
 *            - A* Path planning operates on immutable snapshots and holds NO lock.
 * 
 *   Level 2: Dispatcher lock (domain::Dispatcher::dispatcherMutex - std::mutex)
 *            - Protects activeMissions, missionQueue, completedMissions, fleet,
 *              audit maps, and sequence counters.
 * 
 *   Level 3: Per-Drone lock (domain::Drone::droneMutex - std::mutex)
 *            - Protects individual Drone state, position, battery, activePath,
 *              heartbeat generation, route generation, and commsLost flag.
 * 
 *   Level 4: Scheduler & ThreadPool locks
 *            - engine::EventScheduler::queueMutex (priority queue of timed events)
 *            - util::ThreadSafeQueue::mutex_ (worker task queue in ThreadPool)
 *            - concurrency::ThreadPool::idleMutex (drain barrier condition variable)
 * 
 * CRITICAL INVARIANT:
 * Never call out to the EventScheduler (e.g. scheduleEvent), simulation engine,
 * or Drone callbacks (such as faultCallback) while holding the Dispatcher mutex
 * if that could re-enter it or invert lock ordering.
 * 
 * Specifically:
 * - Drone::handleFault() releases droneMutex BEFORE executing faultCallback.
 * - Dispatcher performs state mutations under dispatcherMutex, collects any
 *   events to schedule into local collections, releases dispatcherMutex, and
 *   only then schedules events onto the scheduler.
 * ============================================================================
 */

// Comparator for missionQueue:
// Orders higher priority missions first (max-heap).
// Breaks ties FIFO by creation order (earlier creation order first).
struct MissionComparator {
    bool operator()(const Mission* a, const Mission* b) const {
        if (!a || !b) return a != nullptr;
        if (a->getPriority() != b->getPriority()) {
            return a->getPriority() < b->getPriority();
        }
        return a->getCreationOrder() > b->getCreationOrder();
    }
};

class Dispatcher {
public:
    // Constants for battery feasibility calculations:
    // Distance metric: Planar Euclidean distance d = sqrt((lat1-lat2)^2 + (lon1-lon2)^2)
    // using Coordinate::distanceTo().
    // Battery model: battery_used = total_distance * BATTERY_CONSUMPTION_PER_DISTANCE.
    // Drone must have at least battery_used + BATTERY_SAFETY_MARGIN to be feasible.
    static constexpr double BATTERY_CONSUMPTION_PER_DISTANCE = 1.0; // % per coordinate unit
    static constexpr double BATTERY_SAFETY_MARGIN = 10.0;            // 10% reserve margin

private:
    std::vector<Mission*> activeMissions;
    std::priority_queue<Mission*, std::vector<Mission*>, MissionComparator> missionQueue;
    std::vector<Mission*> completedMissions;
    std::vector<Drone*> fleet;
    mutable std::mutex dispatcherMutex;

    engine::Simulation* simulation{nullptr};
    Grid* grid{nullptr};

    // Audit sequence tracking for deterministic ordering verification
    std::atomic<int> auditCounter{0};
    std::map<std::string, int> assignmentSequence;
    std::map<std::string, int> requeueSequence;

    // Internal locked helper methods (assumes dispatcherMutex held)
    Drone* assignDroneLocked(Mission* mission);
    Drone* findBestIdleDroneLocked(const Mission* mission) const;
    bool isBatteryFeasibleLocked(const Drone* drone, const Mission* mission) const;
    void reallocateMissionLocked(Mission* mission);
    bool evaluatePreemptionLocked(Mission* newMission);
    Drone* findPreemptionCandidateLocked(const Mission* newMission, Mission*& outDisplacedMission) const;
    void interruptMissionLocked(Mission* displacedMission, Mission* newMission);
    Drone* findDroneForMissionLocked(const Mission* mission) const;
    void recordAssignment(const std::string& missionId, const std::string& droneId);
    void recordRequeue(const std::string& missionId);

public:
    // Constructor
    explicit Dispatcher(engine::EventScheduler* scheduler = nullptr);
    virtual ~Dispatcher() = default;

    // Prevent copy/move
    Dispatcher(const Dispatcher&) = delete;
    Dispatcher& operator=(const Dispatcher&) = delete;

    // Fleet management
    void registerDrone(Drone* drone);
    Drone* findDroneById(const std::string& droneId) const;
    std::vector<Drone*> getFleet() const;

    // UC3: Submit Delivery Request
    // Validates mission, sets PENDING, tries assignDrone().
    // If no drone available, pushes onto missionQueue. Never preempts for non-critical missions.
    bool submitMission(Mission* mission);

    // Pick best IDLE drone using straight-line planar distance and battery feasibility.
    // On success: mission -> ACTIVE, drone -> EN_ROUTE, added to activeMissions.
    Drone* assignDrone(Mission* mission);

    // Used when drone faults or is preempted:
    // Resets mission to PENDING, removes from activeMissions, and re-queues (or reassigns immediately).
    void reallocateMission(Mission* mission);

    // UC5: Dynamic Preemption
    // If idle feasible drone exists, assigns it.
    // Otherwise, if newMission is critical, finds closest drone executing strictly lower-priority
    // mission (battery-feasible), interrupts it, and assigns newMission.
    // If none, queues at head by priority.
    bool evaluatePreemption(Mission* newMission);

    // Marks displaced mission ABORTED, commands drone to abort/drop payload
    // (drone -> REROUTING -> EN_ROUTE onto newMission), and hands displaced mission to reallocateMission().
    void interruptMission(Mission* displacedMission, Mission* newMission = nullptr);

    // Battery feasibility check
    bool isBatteryFeasible(const Drone* drone, const Mission* mission) const;

    // Simulation & Grid integration
    void setSimulation(engine::Simulation* sim);
    void setGrid(Grid* g);

    // Grid change notification (Section 4):
    // For every EN_ROUTE drone whose remaining route intersects a NO_FLY cell,
    // replan from its current position by scheduling a ReplanEvent.
    void onGridChanged(uint64_t version);

    // Mission completion handling (Section 4):
    // Moves mission to completed, transitions drone to IDLE, and pulls next queued mission.
    void completeMission(Mission* mission, Drone* drone);

    // Assigns pending queued missions to newly available idle drones
    void checkAndDispatchQueuedMissions();

    // Inspection and query helpers for testing
    size_t getActiveMissionCount() const;
    size_t getQueuedMissionCount() const;
    size_t getCompletedMissionCount() const;
    std::vector<Mission*> getActiveMissions() const;
    std::vector<Mission*> getCompletedMissions() const;
    Mission* peekMissionQueue() const;
    Mission* popNextQueuedMission();
    bool isMissionActive(const std::string& missionId) const;
    bool isMissionQueued(const std::string& missionId) const;
    int getMissionAssignmentSequence(const std::string& missionId) const;
    int getMissionRequeueSequence(const std::string& missionId) const;
};

} // namespace domain
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_DISPATCHER_HPP
