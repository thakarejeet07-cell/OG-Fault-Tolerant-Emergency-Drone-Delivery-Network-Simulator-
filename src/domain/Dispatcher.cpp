#include "../../include/domain/Dispatcher.hpp"
#include "../../include/engine/EventScheduler.hpp"
#include "../../include/engine/Simulation.hpp"
#include "../../include/domain/Grid.hpp"
#include "../../include/domain/PathPlanner.hpp"
#include "../../include/events/WaypointArrivalEvent.hpp"
#include "../../include/events/MissionCompleteEvent.hpp"
#include "../../include/events/HeartbeatTimeoutEvent.hpp"
#include "../../include/events/ReplanEvent.hpp"
#include "../../include/concurrency/Logger.hpp"
#include <iostream>
#include <algorithm>
#include <limits>
#include <cmath>

namespace simulator {
namespace mesh {
namespace domain {

// --- Dispatcher Constructor -----------------------------------------------
Dispatcher::Dispatcher(engine::EventScheduler* /*scheduler*/) {
}

// --- Fleet Management -----------------------------------------------------
void Dispatcher::registerDrone(Drone* drone) {
    std::lock_guard<std::mutex> lock(dispatcherMutex);
    if (!drone) return;
    fleet.push_back(drone);
    drone->setFaultCallback([this](Drone* /*faultedDrone*/, Mission* faultedMission) {
        if (faultedMission) {
            this->reallocateMission(faultedMission);
        }
    });
}

Drone* Dispatcher::findDroneById(const std::string& droneId) const {
    std::lock_guard<std::mutex> lock(dispatcherMutex);
    for (Drone* d : fleet) {
        if (d && d->getDroneId() == droneId) {
            return d;
        }
    }
    return nullptr;
}

std::vector<Drone*> Dispatcher::getFleet() const {
    std::lock_guard<std::mutex> lock(dispatcherMutex);
    return fleet;
}

// --- Feasibility and Selection -------------------------------------------
bool Dispatcher::isBatteryFeasibleLocked(const Drone* drone, const Mission* mission) const {
    if (!drone || !mission) return false;
    double d1 = drone->getCurrentLocation().distanceTo(mission->getPickupLocation());
    double d2 = mission->getPickupLocation().distanceTo(mission->getDropoffLocation());
    double totalDist = d1 + d2;
    double requiredBattery = (totalDist * BATTERY_CONSUMPTION_PER_DISTANCE) + BATTERY_SAFETY_MARGIN;
    return drone->getBatteryLevel() >= requiredBattery;
}

bool Dispatcher::isBatteryFeasible(const Drone* drone, const Mission* mission) const {
    std::lock_guard<std::mutex> lock(dispatcherMutex);
    return isBatteryFeasibleLocked(drone, mission);
}

Drone* Dispatcher::findBestIdleDroneLocked(const Mission* mission) const {
    Drone* bestDrone = nullptr;
    double minDistance = std::numeric_limits<double>::infinity();

    for (Drone* drone : fleet) {
        if (drone->getState() == Drone::DroneState::IDLE) {
            if (isBatteryFeasibleLocked(drone, mission)) {
                // If grid is configured, validate path exists before selecting
                if (grid) {
                    Grid snap = grid->snapshot();
                    auto p1 = PathPlanner::plan(snap, drone->getCurrentLocation(), mission->getPickupLocation());
                    if (!p1.has_value()) continue;
                    auto p2 = PathPlanner::plan(snap, mission->getPickupLocation(), mission->getDropoffLocation());
                    if (!p2.has_value()) continue;
                }

                double dist = drone->getCurrentLocation().distanceTo(mission->getPickupLocation());
                if (dist < minDistance) {
                    minDistance = dist;
                    bestDrone = drone;
                }
            }
        }
    }
    return bestDrone;
}

Drone* Dispatcher::findDroneForMissionLocked(const Mission* mission) const {
    for (Drone* drone : fleet) {
        if (drone->getCurrentMission() == mission) {
            return drone;
        }
    }
    return nullptr;
}

void Dispatcher::recordAssignment(const std::string& missionId, const std::string& droneId) {
    assignmentSequence[missionId] = ++auditCounter;
    util::log("[Dispatcher] Assigned mission ", missionId, " to drone ", droneId);
}

void Dispatcher::recordRequeue(const std::string& missionId) {
    requeueSequence[missionId] = ++auditCounter;
    util::log("[Dispatcher] Re-queued mission ", missionId);
}

// Helper to schedule flight events after drone is assigned (MUST be called without dispatcherMutex held)
static void scheduleDroneFlight(engine::Simulation* sim, Dispatcher* disp, Drone* drone) {
    if (!sim || !drone) return;
    Path path = drone->getActivePath();
    if (path.waypoints.size() <= 1) return;

    double speed = sim->getConfig().droneSpeed > 0 ? sim->getConfig().droneSpeed : 15.0;
    double dist = path.waypoints[0].distanceTo(path.waypoints[1]);
    long long dt = std::max(1LL, static_cast<long long>(std::round((dist / speed) * 1000.0)));
    long long now = sim->getVirtualTime();

    sim->scheduleSimulationEvent(std::make_unique<events::WaypointArrivalEvent>(
        now + dt, 5, drone->getDroneId(), drone->getRouteGeneration(), 1, disp, sim));

    sim->scheduleSimulationEvent(std::make_unique<events::HeartbeatTimeoutEvent>(
        now + 5000, 8, drone->getDroneId(), drone->getHeartbeatGeneration(), disp, sim));
}

static bool routeDroneForMission(Grid* grid, Drone* drone, Mission* mission) {
    if (!grid) return true;
    Grid snap = grid->snapshot();
    auto p1 = PathPlanner::plan(snap, drone->getCurrentLocation(), mission->getPickupLocation());
    auto p2 = PathPlanner::plan(snap, mission->getPickupLocation(), mission->getDropoffLocation());
    if (!p1.has_value() || !p2.has_value()) {
        return false;
    }

    std::vector<Coordinate> waypoints = p1->waypoints;
    if (!p2->waypoints.empty()) {
        waypoints.insert(waypoints.end(), p2->waypoints.begin() + 1, p2->waypoints.end());
    }
    Path fullPath{std::move(waypoints), snap.getVersion(), p1->totalDistance + p2->totalDistance};
    drone->setActivePath(fullPath);
    drone->setCurrentWaypointIndex(0);
    drone->incrementRouteGeneration();
    return true;
}

// --- UC3: Submit Mission & Assignment --------------------------------------
Drone* Dispatcher::assignDroneLocked(Mission* mission) {
    if (!mission) return nullptr;
    Drone* drone = findBestIdleDroneLocked(mission);
    if (!drone) return nullptr;

    if (!routeDroneForMission(grid, drone, mission)) {
        return nullptr;
    }

    drone->setCurrentMission(mission);
    drone->updateState(Drone::DroneState::EN_ROUTE);
    mission->setState(Mission::MissionState::ACTIVE);
    activeMissions.push_back(mission);

    recordAssignment(mission->getMissionId(), drone->getDroneId());
    return drone;
}

Drone* Dispatcher::assignDrone(Mission* mission) {
    Drone* drone = nullptr;
    {
        std::lock_guard<std::mutex> lock(dispatcherMutex);
        drone = assignDroneLocked(mission);
    }
    if (drone && simulation) {
        scheduleDroneFlight(simulation, this, drone);
    }
    return drone;
}

bool Dispatcher::submitMission(Mission* mission) {
    Drone* drone = nullptr;
    bool success = false;
    {
        std::lock_guard<std::mutex> lock(dispatcherMutex);
        if (!mission) return false;
        mission->setState(Mission::MissionState::PENDING);
        drone = assignDroneLocked(mission);
        if (!drone) {
            missionQueue.push(mission);
            recordRequeue(mission->getMissionId());
            success = false;
        } else {
            success = true;
        }
    }
    if (drone && simulation) {
        scheduleDroneFlight(simulation, this, drone);
    }
    return success;
}

void Dispatcher::reallocateMissionLocked(Mission* mission) {
    if (!mission) return;

    mission->setState(Mission::MissionState::PENDING);

    auto it = std::find(activeMissions.begin(), activeMissions.end(), mission);
    if (it != activeMissions.end()) {
        activeMissions.erase(it);
    }

    Drone* drone = assignDroneLocked(mission);
    if (!drone) {
        missionQueue.push(mission);
        recordRequeue(mission->getMissionId());
    }
}

void Dispatcher::reallocateMission(Mission* mission) {
    Drone* drone = nullptr;
    {
        std::lock_guard<std::mutex> lock(dispatcherMutex);
        if (!mission) return;
        mission->setState(Mission::MissionState::PENDING);
        auto it = std::find(activeMissions.begin(), activeMissions.end(), mission);
        if (it != activeMissions.end()) {
            activeMissions.erase(it);
        }
        drone = assignDroneLocked(mission);
        if (!drone) {
            missionQueue.push(mission);
            recordRequeue(mission->getMissionId());
        }
    }
    if (simulation) {
        simulation->getMetrics()->reallocationsCount.fetch_add(1, std::memory_order_relaxed);
        if (drone) {
            scheduleDroneFlight(simulation, this, drone);
        }
    }
}

// --- UC5: Dynamic Preemption ---------------------------------------------
Drone* Dispatcher::findPreemptionCandidateLocked(const Mission* newMission, Mission*& outDisplacedMission) const {
    Drone* bestDrone = nullptr;
    outDisplacedMission = nullptr;
    double minDistance = std::numeric_limits<double>::infinity();

    for (Drone* drone : fleet) {
        Mission* curM = drone->getCurrentMission();
        if (!curM || curM->getPriority() >= newMission->getPriority()) continue;
        if (!isBatteryFeasibleLocked(drone, newMission)) continue;

        if (grid) {
            Grid snap = grid->snapshot();
            if (!PathPlanner::plan(snap, drone->getCurrentLocation(), newMission->getPickupLocation()).has_value()) continue;
            if (!PathPlanner::plan(snap, newMission->getPickupLocation(), newMission->getDropoffLocation()).has_value()) continue;
        }

        double dist = drone->getCurrentLocation().distanceTo(newMission->getPickupLocation());
        if (dist < minDistance) {
            minDistance = dist;
            bestDrone = drone;
            outDisplacedMission = curM;
        }
    }
    return bestDrone;
}

bool Dispatcher::evaluatePreemptionLocked(Mission* newMission) {
    if (!newMission) return false;

    // a) If an IDLE feasible drone exists, just assign it.
    if (assignDroneLocked(newMission)) {
        return true;
    }

    // b) Only critical missions preempt lower-priority active deliveries.
    if (!newMission->isCritical()) {
        newMission->setState(Mission::MissionState::PENDING);
        missionQueue.push(newMission);
        recordRequeue(newMission->getMissionId());
        return false;
    }

    Mission* displacedMission = nullptr;
    Drone* bestDrone = findPreemptionCandidateLocked(newMission, displacedMission);
    if (!bestDrone || !displacedMission) {
        newMission->setState(Mission::MissionState::PENDING);
        missionQueue.push(newMission);
        recordRequeue(newMission->getMissionId());
        return false;
    }

    // c) Interrupt candidate drone and assign critical mission
    interruptMissionLocked(displacedMission, newMission);
    return true;
}

bool Dispatcher::evaluatePreemption(Mission* newMission) {
    bool success = false;
    Drone* targetDrone = nullptr;
    {
        std::lock_guard<std::mutex> lock(dispatcherMutex);
        success = evaluatePreemptionLocked(newMission);
        if (success) {
            targetDrone = findDroneForMissionLocked(newMission);
        }
    }
    if (success && simulation) {
        simulation->getMetrics()->preemptionsCount.fetch_add(1, std::memory_order_relaxed);
        if (targetDrone) {
            scheduleDroneFlight(simulation, this, targetDrone);
        }
    }
    return success;
}

void Dispatcher::interruptMissionLocked(Mission* displacedMission, Mission* newMission) {
    if (!displacedMission) return;

    Drone* targetDrone = findDroneForMissionLocked(displacedMission);

    // Mark ABORTED for the preempted run
    displacedMission->setState(Mission::MissionState::ABORTED);
    displacedMission->setPreempted(true);

    auto it = std::find(activeMissions.begin(), activeMissions.end(), displacedMission);
    if (it != activeMissions.end()) {
        activeMissions.erase(it);
    }

    if (targetDrone) {
        targetDrone->incrementRouteGeneration(); // Invalidate old waypoint events
        targetDrone->updateState(Drone::DroneState::REROUTING);

        if (newMission) {
            routeDroneForMission(grid, targetDrone, newMission);

            targetDrone->setCurrentMission(newMission);
            targetDrone->updateState(Drone::DroneState::EN_ROUTE);
            newMission->setState(Mission::MissionState::ACTIVE);
            activeMissions.push_back(newMission);
            recordAssignment(newMission->getMissionId(), targetDrone->getDroneId());
        }
    }

    // Reallocate the displaced mission back to PENDING in queue
    reallocateMissionLocked(displacedMission);
}

void Dispatcher::interruptMission(Mission* displacedMission, Mission* newMission) {
    Drone* targetDrone = nullptr;
    {
        std::lock_guard<std::mutex> lock(dispatcherMutex);
        interruptMissionLocked(displacedMission, newMission);
        if (newMission) {
            targetDrone = findDroneForMissionLocked(newMission);
        }
    }
    if (simulation && targetDrone) {
        scheduleDroneFlight(simulation, this, targetDrone);
    }
}

// --- Simulation & Grid Integration ----------------------------------------
void Dispatcher::setSimulation(engine::Simulation* sim) {
    std::lock_guard<std::mutex> lock(dispatcherMutex);
    simulation = sim;
}

void Dispatcher::setGrid(Grid* g) {
    std::lock_guard<std::mutex> lock(dispatcherMutex);
    grid = g;
}

void Dispatcher::onGridChanged(uint64_t version) {
    std::vector<std::string> dronesToReplan;
    {
        std::lock_guard<std::mutex> lock(dispatcherMutex);
        if (!grid) return;

        for (Drone* drone : fleet) {
            if (drone && drone->getState() == Drone::DroneState::EN_ROUTE) {
                Path path = drone->getActivePath();
                size_t currentIdx = drone->getCurrentWaypointIndex();

                bool intersectsNoFly = false;
                for (size_t i = currentIdx; i < path.waypoints.size(); ++i) {
                    auto [cx, cy] = grid->coordinateToCell(path.waypoints[i]);
                    if (grid->getCellState(cx, cy) == CellState::NO_FLY_ZONE) {
                        intersectsNoFly = true;
                        break;
                    }
                }

                if (intersectsNoFly) {
                    drone->updateState(Drone::DroneState::REROUTING);
                    drone->incrementRouteGeneration();
                    dronesToReplan.push_back(drone->getDroneId());
                }
            }
        }
    }

    if (simulation) {
        long long now = simulation->getVirtualTime();
        for (const auto& droneId : dronesToReplan) {
            simulation->scheduleSimulationEvent(std::make_unique<events::ReplanEvent>(
                now, 8, droneId, version, 0, this, simulation));
            simulation->getMetrics()->replansCount.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

void Dispatcher::completeMission(Mission* mission, Drone* drone) {
    Drone* nextAssignedDrone = nullptr;
    {
        std::lock_guard<std::mutex> lock(dispatcherMutex);
        if (!mission || !drone) return;

        auto it = std::find(activeMissions.begin(), activeMissions.end(), mission);
        if (it != activeMissions.end()) {
            activeMissions.erase(it);
        }

        mission->setState(Mission::MissionState::COMPLETED);
        completedMissions.push_back(mission);

        drone->setCurrentMission(nullptr);
        drone->updateState(Drone::DroneState::IDLE);

        if (!missionQueue.empty()) {
            Mission* nextM = missionQueue.top();
            missionQueue.pop();
            nextAssignedDrone = assignDroneLocked(nextM);
            if (!nextAssignedDrone) {
                missionQueue.push(nextM);
            }
        }
    }

    if (simulation) {
        simulation->getMetrics()->missionsCompletedCount.fetch_add(1, std::memory_order_relaxed);
        if (nextAssignedDrone) {
            scheduleDroneFlight(simulation, this, nextAssignedDrone);
        }
    }
}

void Dispatcher::checkAndDispatchQueuedMissions() {
    std::vector<Drone*> assignedDrones;
    {
        std::lock_guard<std::mutex> lock(dispatcherMutex);
        while (!missionQueue.empty()) {
            Mission* m = missionQueue.top();
            Drone* d = assignDroneLocked(m);
            if (d) {
                missionQueue.pop();
                assignedDrones.push_back(d);
            } else {
                break;
            }
        }
    }
    if (simulation) {
        for (Drone* d : assignedDrones) {
            scheduleDroneFlight(simulation, this, d);
        }
    }
}

// --- Inspection and Query Helpers -----------------------------------------
size_t Dispatcher::getActiveMissionCount() const {
    std::lock_guard<std::mutex> lock(dispatcherMutex);
    return activeMissions.size();
}

size_t Dispatcher::getQueuedMissionCount() const {
    std::lock_guard<std::mutex> lock(dispatcherMutex);
    return missionQueue.size();
}

size_t Dispatcher::getCompletedMissionCount() const {
    std::lock_guard<std::mutex> lock(dispatcherMutex);
    return completedMissions.size();
}

std::vector<Mission*> Dispatcher::getActiveMissions() const {
    std::lock_guard<std::mutex> lock(dispatcherMutex);
    return activeMissions;
}

std::vector<Mission*> Dispatcher::getCompletedMissions() const {
    std::lock_guard<std::mutex> lock(dispatcherMutex);
    return completedMissions;
}

Mission* Dispatcher::peekMissionQueue() const {
    std::lock_guard<std::mutex> lock(dispatcherMutex);
    if (missionQueue.empty()) return nullptr;
    return missionQueue.top();
}

Mission* Dispatcher::popNextQueuedMission() {
    std::lock_guard<std::mutex> lock(dispatcherMutex);
    if (missionQueue.empty()) return nullptr;
    Mission* m = missionQueue.top();
    missionQueue.pop();
    return m;
}

bool Dispatcher::isMissionActive(const std::string& missionId) const {
    std::lock_guard<std::mutex> lock(dispatcherMutex);
    for (const Mission* m : activeMissions) {
        if (m && m->getMissionId() == missionId) {
            return true;
        }
    }
    return false;
}

bool Dispatcher::isMissionQueued(const std::string& missionId) const {
    std::lock_guard<std::mutex> lock(dispatcherMutex);
    auto tempQueue = missionQueue;
    while (!tempQueue.empty()) {
        if (tempQueue.top() && tempQueue.top()->getMissionId() == missionId) {
            return true;
        }
        tempQueue.pop();
    }
    return false;
}

int Dispatcher::getMissionAssignmentSequence(const std::string& missionId) const {
    std::lock_guard<std::mutex> lock(dispatcherMutex);
    auto it = assignmentSequence.find(missionId);
    if (it != assignmentSequence.end()) return it->second;
    return 0;
}

int Dispatcher::getMissionRequeueSequence(const std::string& missionId) const {
    std::lock_guard<std::mutex> lock(dispatcherMutex);
    auto it = requeueSequence.find(missionId);
    if (it != requeueSequence.end()) return it->second;
    return 0;
}

} // namespace domain
} // namespace mesh
} // namespace simulator
