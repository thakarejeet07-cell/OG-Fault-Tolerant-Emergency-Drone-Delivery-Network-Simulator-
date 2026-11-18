#include "../../include/engine/ScenarioRunner.hpp"
#include "../../include/domain/Dispatcher.hpp"
#include "../../include/domain/Drone.hpp"
#include "../../include/domain/Mission.hpp"
#include "../../include/domain/Grid.hpp"
#include "../../include/hazards/HazardGenerator.hpp"
#include "../../include/hazards/CommLossEvent.hpp"
#include "../../include/hazards/NoFlyZoneEvent.hpp"
#include "../../include/events/DispatchEvent.hpp"
#include "../../include/events/FaultEvent.hpp"
#include <iostream>
#include <set>
#include <chrono>
#include <thread>
#include <random>
#include <typeinfo>

namespace simulator {
namespace mesh {
namespace engine {

namespace {

void checkMissionInvariants(const domain::Dispatcher* disp,
                            const std::vector<domain::Mission*>& allMissions,
                            InvariantResult& result) {
    // Invariant a: every mission is in exactly one of {queue, active, completed}
    for (domain::Mission* m : allMissions) {
        if (!m) continue;
        const std::string& mId = m->getMissionId();
        int inActive = disp->isMissionActive(mId) ? 1 : 0;
        int inQueue = disp->isMissionQueued(mId) ? 1 : 0;
        int isCompleted = (m->getState() == domain::Mission::MissionState::COMPLETED) ? 1 : 0;

        int totalLocations = inActive + inQueue + isCompleted;
        if (totalLocations != 1) {
            result.passed = false;
            result.violations.push_back("Invariant (a) violated: Mission " + mId +
                                        " found in " + std::to_string(totalLocations) + " states");
        }
    }

    // Invariant b: no drone holds two missions; no mission is held by two drones
    std::set<domain::Mission*> heldMissions;
    for (domain::Drone* d : disp->getFleet()) {
        if (!d || !d->getCurrentMission()) continue;
        if (!heldMissions.insert(d->getCurrentMission()).second) {
            result.passed = false;
            result.violations.push_back("Invariant (b) violated: Mission " +
                                        d->getCurrentMission()->getMissionId() + " held by multiple drones!");
        }
    }
}

void checkDroneInvariants(const domain::Dispatcher* disp,
                          const domain::Grid* grid,
                          const Simulation& sim,
                          InvariantResult& result) {
    for (domain::Drone* d : disp->getFleet()) {
        if (!d) continue;
        auto state = d->getState();
        // Invariant c: legal state and no stuck fault recovery
        if (state == domain::Drone::DroneState::FAULT_RECOVERY && sim.getEventScheduler()->empty()) {
            result.passed = false;
            result.violations.push_back("Invariant (c) violated: Drone " + d->getDroneId() + " stuck in FAULT_RECOVERY");
        }

        // Invariant d: no route passes through active NO_FLY cell without rerouting
        if (grid && state == domain::Drone::DroneState::EN_ROUTE) {
            domain::Path path = d->getActivePath();
            for (size_t i = d->getCurrentWaypointIndex(); i < path.waypoints.size(); ++i) {
                auto [cx, cy] = grid->coordinateToCell(path.waypoints[i]);
                if (path.gridVersion == grid->getVersion() &&
                    grid->getCellState(cx, cy) == domain::CellState::NO_FLY_ZONE) {
                    result.passed = false;
                    result.violations.push_back("Invariant (d) violated: Drone " + d->getDroneId() + " on NO_FLY cell");
                    break;
                }
            }
        }
    }
}

void checkEventAndPreemptionInvariants(const Simulation& sim,
                                       bool isDemoScenario,
                                       InvariantResult& result) {
    if (isDemoScenario && sim.getMetrics()->preemptionsCount.load() < 1) {
        result.passed = false;
        result.violations.push_back("Invariant (f) violated: Expected >= 1 preemption in demo scenario");
    }

    // Invariant g: Polymorphic dynamic_cast verification on Event hierarchy
    for (const auto& evPtr : sim.getAllocatedEvents()) {
        const events::Event* ev = evPtr.get();
        if (!ev) continue;

        // Dynamic pointer cast with nullptr check
        if (const auto* dispatchEv = dynamic_cast<const events::DispatchEvent*>(ev)) {
            if (!dispatchEv->getTargetMission()) {
                result.passed = false;
                result.violations.push_back("Invariant (g) violated: DispatchEvent has null target mission");
            }
        } else if (const auto* faultEv = dynamic_cast<const events::FaultEvent*>(ev)) {
            if (faultEv->getErrorCode() < 0) {
                result.passed = false;
                result.violations.push_back("Invariant (g) violated: FaultEvent has negative error code");
            }
        }

        // Dynamic reference cast with bad_cast catch demonstration
        try {
            const events::DispatchEvent& dispRef = dynamic_cast<const events::DispatchEvent&>(*ev);
            (void)dispRef;
        } catch (const std::bad_cast&) {
            // Expected for non-DispatchEvent types
        }
    }
}

void runWithWatchdog(Simulation& sim, int timeoutSec, const char* scenarioName) {
    std::atomic<bool> completed{false};
    std::thread watchdog([&completed, timeoutSec, scenarioName]() {
        for (int i = 0; i < timeoutSec; ++i) {
            if (completed.load()) return;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        if (!completed.load()) {
            std::cerr << "\n[WATCHDOG ERROR] " << scenarioName << " timed out (possible deadlock)!\n";
            std::terminate();
        }
    });

    sim.start();
    completed.store(true);
    if (watchdog.joinable()) {
        watchdog.join();
    }
}

std::vector<domain::Mission*> generateStressMissions(Simulation& sim, domain::Dispatcher* disp,
                                                    uint64_t seed, int numMissions) {
    std::vector<domain::Mission*> allMissions;
    allMissions.reserve(numMissions);
    std::mt19937_64 rng(seed + 101);
    std::uniform_real_distribution<double> posDist(10.0, 500.0);
    int priorities[] = {
        domain::Mission::PRIORITY_LOW,
        domain::Mission::PRIORITY_NORMAL,
        domain::Mission::PRIORITY_HIGH,
        domain::Mission::PRIORITY_CRITICAL
    };

    for (int i = 0; i < numMissions; ++i) {
        int pri = priorities[rng() % 4];
        std::string mId = "MSN-STR-" + std::to_string(i + 1);
        domain::Coordinate pickup(posDist(rng), posDist(rng));
        domain::Coordinate dropoff(posDist(rng), posDist(rng));

        auto m = std::make_unique<domain::Mission>(mId, pri, pickup, dropoff);
        allMissions.push_back(m.get());

        long long dispatchTime = (i < 50) ? 0 : (rng() % 15000);
        sim.scheduleSimulationEvent(std::make_unique<events::DispatchEvent>(dispatchTime, pri, m.get(), disp));
        sim.registerMission(std::move(m));
    }
    return allMissions;
}

} // anonymous namespace

InvariantResult ScenarioRunner::verifyInvariants(Simulation& sim,
                                                 const std::vector<domain::Mission*>& allMissions,
                                                 bool isDemoScenario) {
    InvariantResult result;
    domain::Dispatcher* disp = sim.getDispatcher();
    domain::Grid* grid = sim.getGrid();

    if (!disp) {
        result.passed = false;
        result.violations.push_back("Dispatcher is null");
        return result;
    }

    checkMissionInvariants(disp, allMissions, result);
    checkDroneInvariants(disp, grid, sim, result);
    checkEventAndPreemptionInvariants(sim, isDemoScenario, result);

    return result;
}

bool ScenarioRunner::runDemo(SimulationConfig config, bool printMetrics) {
    std::cout << "\n=======================================================\n";
    std::cout << "               RUNNING DEMO SCENARIO                  \n";
    std::cout << "=======================================================\n";

    config.numDrones = 3;
    config.gridWidth = 20;
    config.gridHeight = 20;
    config.cellSizeMeters = 10.0;
    config.workerThreadCount = 4;
    config.simulationDurationMs = 25000;
    config.droneSpeed = 20.0;
    config.batteryDrainRate = 0.01;
    config.commLossProb = 0.0;
    config.batteryDrainProb = 0.0;
    config.noFlyZoneProb = 0.0;

    Simulation sim;
    if (!sim.initialize(config)) return false;

    domain::Dispatcher* disp = sim.getDispatcher();
    domain::Grid* grid = sim.getGrid();

    // Routine background missions
    std::vector<domain::Mission*> allMissions;
    auto m1 = std::make_unique<domain::Mission>("MSN-001", domain::Mission::PRIORITY_NORMAL,
        domain::Coordinate(10.0, 10.0), domain::Coordinate(40.0, 40.0));
    auto m2 = std::make_unique<domain::Mission>("MSN-002", domain::Mission::PRIORITY_NORMAL,
        domain::Coordinate(20.0, 10.0), domain::Coordinate(50.0, 40.0));
    auto m3 = std::make_unique<domain::Mission>("MSN-003", domain::Mission::PRIORITY_NORMAL,
        domain::Coordinate(30.0, 10.0), domain::Coordinate(60.0, 40.0));

    allMissions.push_back(m1.get());
    allMissions.push_back(m2.get());
    allMissions.push_back(m3.get());
    disp->submitMission(m1.get());
    disp->submitMission(m2.get());
    disp->submitMission(m3.get());
    sim.registerMission(std::move(m1));
    sim.registerMission(std::move(m2));
    sim.registerMission(std::move(m3));

    // Critical organ transplant mission at t=100 triggers preemption
    auto mCrit = std::make_unique<domain::Mission>("MSN-CRIT-HEART", domain::Mission::PRIORITY_CRITICAL,
        domain::Coordinate(15.0, 15.0), domain::Coordinate(35.0, 35.0));
    allMissions.push_back(mCrit.get());
    sim.scheduleSimulationEvent(std::make_unique<events::DispatchEvent>(
        100, domain::Mission::PRIORITY_CRITICAL, mCrit.get(), disp));
    sim.registerMission(std::move(mCrit));

    // Hazard events
    sim.scheduleSimulationEvent(std::make_unique<hazards::NoFlyZoneEvent>(
        200, 7, domain::Coordinate(30.0, 30.0), 15.0, 8000, grid, disp, &sim));
    sim.scheduleSimulationEvent(std::make_unique<hazards::CommLossEvent>(
        300, 7, "DRN-003", 4000, disp, &sim));

    runWithWatchdog(sim, 15, "Demo");

    if (printMetrics) {
        sim.getMetrics()->printSummaryTable();
        sim.getMetrics()->writeMetricsJson("metrics.json");
        std::cout << "Fleet Final Status (explicit std::string conversion):\n";
        for (domain::Drone* d : disp->getFleet()) {
            if (d) std::cout << "  - " << static_cast<std::string>(*d) << "\n";
        }
    }

    auto inv = verifyInvariants(sim, allMissions, true);
    if (!inv.passed) {
        std::cerr << "DEMO FAILED: Invariant violations:\n";
        for (const auto& v : inv.violations) std::cerr << "  - " << v << "\n";
        return false;
    }

    std::cout << "DEMO PASSED: All invariants satisfied!\n";
    return true;
}

bool ScenarioRunner::runStress(SimulationConfig config, bool printMetrics) {
    std::cout << "\n=======================================================\n";
    std::cout << "               RUNNING STRESS SCENARIO                 \n";
    std::cout << "=======================================================\n";

    config.numDrones = 50;
    config.gridWidth = 60;
    config.gridHeight = 60;
    config.cellSizeMeters = 10.0;
    config.droneSpeed = 25.0;
    config.batteryDrainRate = 0.005;
    config.simulationDurationMs = 30000;
    config.commLossProb = 0.3;
    config.batteryDrainProb = 0.2;
    config.noFlyZoneProb = 0.25;

    Simulation sim;
    if (!sim.initialize(config)) return false;

    hazards::HazardGenerator::generateHazards(config, &sim);
    std::vector<domain::Mission*> allMissions = generateStressMissions(
        sim, sim.getDispatcher(), config.seed, 300);

    runWithWatchdog(sim, 20, "Stress");

    if (printMetrics) {
        sim.getMetrics()->printSummaryTable();
        sim.getMetrics()->writeMetricsJson("metrics.json");
    }

    auto inv = verifyInvariants(sim, allMissions, false);
    if (!inv.passed) {
        std::cerr << "STRESS FAILED: Invariant violations:\n";
        for (const auto& v : inv.violations) std::cerr << "  - " << v << "\n";
        return false;
    }

    std::cout << "STRESS PASSED: All invariants satisfied!\n";
    return true;
}

} // namespace engine
} // namespace mesh
} // namespace simulator
