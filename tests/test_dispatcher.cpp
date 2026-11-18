#include <gtest/gtest.h>
#include "../include/domain/Dispatcher.hpp"
#include "../include/domain/Drone.hpp"
#include "../include/domain/Mission.hpp"
#include "../include/events/DispatchEvent.hpp"
#include "../include/engine/EventScheduler.hpp"
#include "../include/concurrency/ThreadPool.hpp"

#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <set>

using namespace simulator::mesh;

// ---------------------------------------------------------------------------
// Deliverable: Isolated end-to-end test reproducing the object diagram scenario
// ---------------------------------------------------------------------------
TEST(DispatcherTest, EndToEndHeartPreemptsBlood) {
    // 1. Setup EventScheduler and ThreadPool
    engine::EventScheduler scheduler;
    concurrency::ThreadPool pool(2);

    // 2. Setup Dispatcher and drone_42 (DRN-042, 84.5% battery, at 40.7128,-74.0060)
    domain::Dispatcher dispatcher(&scheduler);
    domain::Drone drone_42("DRN-042", domain::Coordinate(40.7128, -74.0060));
    drone_42.setBatteryLevel(84.5);
    dispatcher.registerDrone(&drone_42);

    // 3. Setup MSN-1001 (Blood Samples, priority 3, ACTIVE on drone_42)
    // Fleet is otherwise empty/busy so no idle drone exists.
    domain::Mission bloodMission(
        "MSN-1001",
        domain::Mission::PRIORITY_NORMAL, // priority 3
        domain::Coordinate(40.7130, -74.0060),
        domain::Coordinate(40.7200, -74.0100)
    );
    bool bloodSubmitted = dispatcher.submitMission(&bloodMission);
    ASSERT_TRUE(bloodSubmitted);
    ASSERT_EQ(drone_42.getState(), domain::Drone::DroneState::EN_ROUTE);
    ASSERT_EQ(bloodMission.getState(), domain::Mission::MissionState::ACTIVE);
    ASSERT_EQ(drone_42.getCurrentMission(), &bloodMission);
    ASSERT_EQ(dispatcher.getActiveMissionCount(), 1u);
    ASSERT_EQ(dispatcher.getQueuedMissionCount(), 0u);

    // 4. Action: submit MSN-9999 (Organ: Heart, priority 10) as a DispatchEvent
    // through the EventScheduler -> ThreadPool path.
    domain::Mission heartMission(
        "MSN-9999",
        domain::Mission::PRIORITY_CRITICAL, // priority 10
        domain::Coordinate(40.7135, -74.0065),
        domain::Coordinate(40.7250, -74.0150)
    );
    events::DispatchEvent heartEvent(100, 10, &heartMission, &dispatcher);

    pool.startPool();
    scheduler.scheduleEvent(&heartEvent);

    // Drain event from scheduler and execute via ThreadPool
    events::Event* ev = scheduler.popNextEvent();
    ASSERT_NE(ev, nullptr);
    pool.assignTask(ev);
    pool.shutdown(); // Waits for worker thread to complete execution

    // 5. Assert:
    // - MSN-9999 becomes ACTIVE on drone_42
    EXPECT_EQ(heartMission.getState(), domain::Mission::MissionState::ACTIVE);
    EXPECT_EQ(drone_42.getCurrentMission(), &heartMission);

    // - MSN-1001 is no longer on the drone, is PENDING in missionQueue (not lost)
    EXPECT_NE(drone_42.getCurrentMission(), &bloodMission);
    EXPECT_EQ(bloodMission.getState(), domain::Mission::MissionState::PENDING);
    EXPECT_TRUE(bloodMission.wasPreempted());
    EXPECT_TRUE(dispatcher.isMissionQueued("MSN-1001"));
    EXPECT_EQ(dispatcher.getQueuedMissionCount(), 1u);
    EXPECT_EQ(dispatcher.peekMissionQueue(), &bloodMission);

    // - drone_42 ended in a legal state (EN_ROUTE)
    EXPECT_EQ(drone_42.getState(), domain::Drone::DroneState::EN_ROUTE);

    // - the heart mission was assigned BEFORE the blood mission was re-queued/reassigned
    int heartAssignSeq = dispatcher.getMissionAssignmentSequence("MSN-9999");
    int bloodRequeueSeq = dispatcher.getMissionRequeueSequence("MSN-1001");
    EXPECT_GT(heartAssignSeq, 0);
    EXPECT_GT(bloodRequeueSeq, 0);
    EXPECT_LT(heartAssignSeq, bloodRequeueSeq);
}

// ---------------------------------------------------------------------------
// idle drone available -> no preemption occurs
// ---------------------------------------------------------------------------
TEST(DispatcherTest, IdleDroneAvailableNoPreemption) {
    domain::Dispatcher dispatcher;
    domain::Drone drone1("DRN-001", domain::Coordinate(40.7100, -74.0000));
    domain::Drone drone2("DRN-002", domain::Coordinate(40.7200, -74.0100));
    dispatcher.registerDrone(&drone1);
    dispatcher.registerDrone(&drone2);

    domain::Mission bloodMission(
        "MSN-BLOOD",
        domain::Mission::PRIORITY_NORMAL,
        domain::Coordinate(40.7105, -74.0005),
        domain::Coordinate(40.7150, -74.0050)
    );
    dispatcher.submitMission(&bloodMission);

    // drone1 is EN_ROUTE on bloodMission, drone2 is IDLE
    EXPECT_EQ(drone1.getState(), domain::Drone::DroneState::EN_ROUTE);
    EXPECT_EQ(drone2.getState(), domain::Drone::DroneState::IDLE);
    EXPECT_EQ(drone1.getCurrentMission(), &bloodMission);

    // Submit critical heart mission
    domain::Mission heartMission(
        "MSN-HEART",
        domain::Mission::PRIORITY_CRITICAL,
        domain::Coordinate(40.7205, -74.0105),
        domain::Coordinate(40.7300, -74.0200)
    );
    bool assigned = dispatcher.evaluatePreemption(&heartMission);
    EXPECT_TRUE(assigned);

    // Assert: drone2 got heartMission, drone1 was NOT preempted
    EXPECT_EQ(drone2.getState(), domain::Drone::DroneState::EN_ROUTE);
    EXPECT_EQ(drone2.getCurrentMission(), &heartMission);
    EXPECT_EQ(heartMission.getState(), domain::Mission::MissionState::ACTIVE);

    EXPECT_EQ(drone1.getState(), domain::Drone::DroneState::EN_ROUTE);
    EXPECT_EQ(drone1.getCurrentMission(), &bloodMission);
    EXPECT_EQ(bloodMission.getState(), domain::Mission::MissionState::ACTIVE);
    EXPECT_FALSE(bloodMission.wasPreempted());

    EXPECT_EQ(dispatcher.getActiveMissionCount(), 2u);
    EXPECT_EQ(dispatcher.getQueuedMissionCount(), 0u);
}

// ---------------------------------------------------------------------------
// critical request when all drones run equal/higher priority -> queued, no abort
// ---------------------------------------------------------------------------
TEST(DispatcherTest, CriticalRequestAllEqualOrHigherPriorityQueuedNoAbort) {
    domain::Dispatcher dispatcher;
    domain::Drone drone1("DRN-001", domain::Coordinate(40.7100, -74.0000));
    dispatcher.registerDrone(&drone1);

    // drone1 already runs a critical mission (priority 10)
    domain::Mission critical1(
        "MSN-CRIT-1",
        domain::Mission::PRIORITY_CRITICAL,
        domain::Coordinate(40.7105, -74.0005),
        domain::Coordinate(40.7150, -74.0050)
    );
    dispatcher.submitMission(&critical1);
    ASSERT_EQ(drone1.getCurrentMission(), &critical1);

    // New critical mission arrives (priority 10)
    domain::Mission critical2(
        "MSN-CRIT-2",
        domain::Mission::PRIORITY_CRITICAL,
        domain::Coordinate(40.7105, -74.0005),
        domain::Coordinate(40.7150, -74.0050)
    );
    bool preempted = dispatcher.evaluatePreemption(&critical2);
    EXPECT_FALSE(preempted);

    // Assert: critical1 not aborted, critical2 queued at head
    EXPECT_EQ(drone1.getCurrentMission(), &critical1);
    EXPECT_EQ(critical1.getState(), domain::Mission::MissionState::ACTIVE);
    EXPECT_EQ(critical2.getState(), domain::Mission::MissionState::PENDING);
    EXPECT_EQ(dispatcher.getQueuedMissionCount(), 1u);
    EXPECT_EQ(dispatcher.peekMissionQueue(), &critical2);
}

// ---------------------------------------------------------------------------
// illegal state transitions rejected
// ---------------------------------------------------------------------------
TEST(DispatcherTest, IllegalStateTransitionsRejected) {
    domain::Drone drone("DRN-SM", domain::Coordinate(0, 0));
    // Initial state: IDLE
    EXPECT_EQ(drone.getState(), domain::Drone::DroneState::IDLE);

    // From IDLE: only EN_ROUTE is legal
    EXPECT_FALSE(drone.updateState(domain::Drone::DroneState::REROUTING));
    EXPECT_FALSE(drone.updateState(domain::Drone::DroneState::FAULT_RECOVERY));
    EXPECT_FALSE(drone.updateState(domain::Drone::DroneState::IDLE));
    EXPECT_THROW(drone.setState(domain::Drone::DroneState::FAULT_RECOVERY), std::logic_error);

    // Transition IDLE -> EN_ROUTE
    EXPECT_TRUE(drone.updateState(domain::Drone::DroneState::EN_ROUTE));
    EXPECT_EQ(drone.getState(), domain::Drone::DroneState::EN_ROUTE);

    // From EN_ROUTE: EN_ROUTE -> EN_ROUTE is illegal
    EXPECT_FALSE(drone.updateState(domain::Drone::DroneState::EN_ROUTE));

    // Transition EN_ROUTE -> REROUTING
    EXPECT_TRUE(drone.updateState(domain::Drone::DroneState::REROUTING));
    EXPECT_EQ(drone.getState(), domain::Drone::DroneState::REROUTING);

    // From REROUTING: REROUTING -> IDLE is illegal
    EXPECT_FALSE(drone.updateState(domain::Drone::DroneState::IDLE));
    EXPECT_FALSE(drone.updateState(domain::Drone::DroneState::REROUTING));

    // Transition REROUTING -> FAULT_RECOVERY
    EXPECT_TRUE(drone.updateState(domain::Drone::DroneState::FAULT_RECOVERY));
    EXPECT_EQ(drone.getState(), domain::Drone::DroneState::FAULT_RECOVERY);

    // From FAULT_RECOVERY: only IDLE is legal
    EXPECT_FALSE(drone.updateState(domain::Drone::DroneState::EN_ROUTE));
    EXPECT_FALSE(drone.updateState(domain::Drone::DroneState::REROUTING));
    EXPECT_FALSE(drone.updateState(domain::Drone::DroneState::FAULT_RECOVERY));

    // Transition FAULT_RECOVERY -> IDLE
    EXPECT_TRUE(drone.updateState(domain::Drone::DroneState::IDLE));
    EXPECT_EQ(drone.getState(), domain::Drone::DroneState::IDLE);
}

// ---------------------------------------------------------------------------
// low battery drone not chosen / not preempted
// ---------------------------------------------------------------------------
TEST(DispatcherTest, LowBatteryDroneNotChosenOrPreempted) {
    domain::Dispatcher dispatcher;
    domain::Drone lowBatteryDrone("DRN-LOW", domain::Coordinate(0, 0));
    lowBatteryDrone.setBatteryLevel(5.0); // 5% battery
    dispatcher.registerDrone(&lowBatteryDrone);

    domain::Mission mission(
        "MSN-NORMAL",
        domain::Mission::PRIORITY_NORMAL,
        domain::Coordinate(0.01, 0.01),
        domain::Coordinate(0.02, 0.02)
    );

    // Submit mission: drone has insufficient battery (requires >= 10% safety margin)
    bool assigned = dispatcher.submitMission(&mission);
    EXPECT_FALSE(assigned);
    EXPECT_EQ(lowBatteryDrone.getState(), domain::Drone::DroneState::IDLE);
    EXPECT_EQ(mission.getState(), domain::Mission::MissionState::PENDING);
    EXPECT_TRUE(dispatcher.isMissionQueued("MSN-NORMAL"));

    // Now test preemption: if a low-battery drone is running a low-priority mission,
    // it must not be preempted for a critical mission it cannot complete.
    domain::Dispatcher dispatcher2;
    domain::Drone lowDroneRunning("DRN-LOW2", domain::Coordinate(0, 0));
    lowDroneRunning.setBatteryLevel(5.0);
    dispatcher2.registerDrone(&lowDroneRunning);

    // Force drone into EN_ROUTE on low priority mission
    domain::Mission lowPriMission("MSN-LOW-PRI", 1, domain::Coordinate(0, 0), domain::Coordinate(0, 0));
    lowDroneRunning.setCurrentMission(&lowPriMission);
    lowDroneRunning.updateState(domain::Drone::DroneState::EN_ROUTE);
    lowPriMission.setState(domain::Mission::MissionState::ACTIVE);

    domain::Mission criticalMission("MSN-CRIT", 10, domain::Coordinate(10, 10), domain::Coordinate(20, 20));
    bool preemptSuccess = dispatcher2.evaluatePreemption(&criticalMission);
    EXPECT_FALSE(preemptSuccess);
    EXPECT_EQ(criticalMission.getState(), domain::Mission::MissionState::PENDING);
    EXPECT_EQ(lowDroneRunning.getCurrentMission(), &lowPriMission);
}

// ---------------------------------------------------------------------------
// handleFault() releases the mission and Dispatcher reallocates it
// ---------------------------------------------------------------------------
TEST(DispatcherTest, HandleFaultReleasesMissionAndReallocates) {
    domain::Dispatcher dispatcher;
    domain::Drone drone1("DRN-001", domain::Coordinate(0, 0));
    domain::Drone drone2("DRN-002", domain::Coordinate(0, 0));
    dispatcher.registerDrone(&drone1);
    dispatcher.registerDrone(&drone2);

    domain::Mission mission("MSN-FAULT-TEST", domain::Mission::PRIORITY_NORMAL, domain::Coordinate(0, 0), domain::Coordinate(1, 1));
    dispatcher.submitMission(&mission);

    // drone1 was assigned mission
    EXPECT_EQ(drone1.getCurrentMission(), &mission);
    EXPECT_EQ(drone1.getState(), domain::Drone::DroneState::EN_ROUTE);

    // Trigger fault on drone1
    domain::Mission* released = drone1.handleFault();
    EXPECT_EQ(released, &mission);
    EXPECT_EQ(drone1.getState(), domain::Drone::DroneState::FAULT_RECOVERY);
    EXPECT_EQ(drone1.getCurrentMission(), nullptr);

    // Dispatcher callback should have reallocated mission to drone2!
    EXPECT_EQ(drone2.getCurrentMission(), &mission);
    EXPECT_EQ(drone2.getState(), domain::Drone::DroneState::EN_ROUTE);
    EXPECT_EQ(mission.getState(), domain::Mission::MissionState::ACTIVE);

    // If drone2 also faults, mission should return to missionQueue
    drone2.handleFault();
    EXPECT_EQ(drone2.getState(), domain::Drone::DroneState::FAULT_RECOVERY);
    EXPECT_EQ(drone2.getCurrentMission(), nullptr);
    EXPECT_EQ(mission.getState(), domain::Mission::MissionState::PENDING);
    EXPECT_TRUE(dispatcher.isMissionQueued("MSN-FAULT-TEST"));
}

// ---------------------------------------------------------------------------
// triggerCollisionAvoidance(): local-only safety, does not touch mission
// ---------------------------------------------------------------------------
TEST(DispatcherTest, LocalCollisionAvoidanceDoesNotAffectMission) {
    domain::Drone drone("DRN-COLLISION", domain::Coordinate(0, 0));
    domain::Mission mission("MSN-COLLISION-TEST", domain::Mission::PRIORITY_NORMAL);

    drone.updateState(domain::Drone::DroneState::EN_ROUTE);
    drone.setCurrentMission(&mission);
    mission.setState(domain::Mission::MissionState::ACTIVE);

    bool rerouted = drone.triggerCollisionAvoidance();
    EXPECT_TRUE(rerouted);
    EXPECT_EQ(drone.getState(), domain::Drone::DroneState::REROUTING);
    EXPECT_EQ(drone.getCurrentMission(), &mission);
    EXPECT_EQ(mission.getState(), domain::Mission::MissionState::ACTIVE);

    // Return to EN_ROUTE
    EXPECT_TRUE(drone.updateState(domain::Drone::DroneState::EN_ROUTE));
    EXPECT_EQ(drone.getState(), domain::Drone::DroneState::EN_ROUTE);
    EXPECT_EQ(drone.getCurrentMission(), &mission);
}

// ---------------------------------------------------------------------------
// EventScheduler priority ordering for DispatchEvent
// ---------------------------------------------------------------------------
TEST(DispatcherTest, EventSchedulerPriorityOrderingForDispatch) {
    engine::EventScheduler scheduler;
    domain::Mission mNormal("M-NORMAL", 3);
    domain::Mission mCrit("M-CRITICAL", 10);

    events::DispatchEvent eNormal(100, 3, &mNormal);
    events::DispatchEvent eCrit(100, 10, &mCrit); // same timestamp, higher priority

    scheduler.scheduleEvent(&eNormal);
    scheduler.scheduleEvent(&eCrit);

    // Critical event must pop first due to higher priority at same timestamp
    events::Event* first = scheduler.popNextEvent();
    ASSERT_NE(first, nullptr);
    EXPECT_EQ(first->getPriority(), 10);

    events::Event* second = scheduler.popNextEvent();
    ASSERT_NE(second, nullptr);
    EXPECT_EQ(second->getPriority(), 3);
}


// ---------------------------------------------------------------------------
// Concurrent submissions (several threads) leave no mission duplicated or lost
// Invariant: every mission is in exactly one of: queue, active, completed
// ---------------------------------------------------------------------------
TEST(DispatcherTest, ConcurrentSubmissionsNoMissionLostOrDuplicated) {
    domain::Dispatcher dispatcher;
    // Register 4 drones using unique_ptr RAII
    std::vector<std::unique_ptr<domain::Drone>> drones;
    for (int i = 0; i < 4; ++i) {
        drones.push_back(std::make_unique<domain::Drone>("DRN-" + std::to_string(i), domain::Coordinate(0, 0)));
        dispatcher.registerDrone(drones.back().get());
    }

    constexpr int NUM_THREADS = 8;
    constexpr int MISSIONS_PER_THREAD = 15;
    constexpr int TOTAL_MISSIONS = NUM_THREADS * MISSIONS_PER_THREAD;

    std::vector<domain::Mission> missions;
    missions.reserve(TOTAL_MISSIONS);
    for (int i = 0; i < TOTAL_MISSIONS; ++i) {
        missions.emplace_back("MSN-CONC-" + std::to_string(i), (i % 10) + 1);
    }

    std::vector<std::thread> threads;
    threads.reserve(NUM_THREADS);
    for (int t = 0; t < NUM_THREADS; ++t) {
        threads.emplace_back([&dispatcher, &missions, t]() {
            for (int j = 0; j < MISSIONS_PER_THREAD; ++j) {
                int idx = t * MISSIONS_PER_THREAD + j;
                if (missions[idx].isCritical()) {
                    dispatcher.evaluatePreemption(&missions[idx]);
                } else {
                    dispatcher.submitMission(&missions[idx]);
                }
            }
        });
    }

    for (auto& th : threads) {
        th.join();
    }

    // Invariant check:
    // Total active + total queued must equal TOTAL_MISSIONS.
    size_t activeCount = dispatcher.getActiveMissionCount();
    size_t queuedCount = dispatcher.getQueuedMissionCount();
    EXPECT_EQ(activeCount + queuedCount, static_cast<size_t>(TOTAL_MISSIONS));
    EXPECT_LE(activeCount, 4u); // At most 4 drones can be active simultaneously

    // Verify all mission IDs are uniquely present
    std::set<std::string> seenMissions;
    for (const domain::Mission* m : dispatcher.getActiveMissions()) {
        ASSERT_NE(m, nullptr);
        EXPECT_EQ(m->getState(), domain::Mission::MissionState::ACTIVE);
        EXPECT_TRUE(seenMissions.insert(m->getMissionId()).second);
    }

    while (dispatcher.getQueuedMissionCount() > 0) {
        domain::Mission* m = dispatcher.popNextQueuedMission();
        ASSERT_NE(m, nullptr);
        EXPECT_EQ(m->getState(), domain::Mission::MissionState::PENDING);
        EXPECT_TRUE(seenMissions.insert(m->getMissionId()).second);
    }

    EXPECT_EQ(seenMissions.size(), static_cast<size_t>(TOTAL_MISSIONS));
}
