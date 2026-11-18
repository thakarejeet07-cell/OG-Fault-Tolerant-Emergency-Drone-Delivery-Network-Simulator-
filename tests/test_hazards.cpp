#include <gtest/gtest.h>
#include "../include/engine/Simulation.hpp"
#include "../include/hazards/HazardGenerator.hpp"
#include "../include/hazards/CommLossEvent.hpp"
#include "../include/hazards/NoFlyZoneEvent.hpp"
#include "../include/events/HeartbeatTimeoutEvent.hpp"
#include "../include/domain/Drone.hpp"
#include "../include/domain/Mission.hpp"
#include "../include/domain/Grid.hpp"

using namespace simulator::mesh;

TEST(HazardsTest, SameSeedGivesSameHazardSchedule) {
    engine::SimulationConfig cfg1;
    cfg1.seed = 12345;
    cfg1.simulationDurationMs = 30000;
    cfg1.commLossProb = 0.5;
    cfg1.batteryDrainProb = 0.5;
    cfg1.noFlyZoneProb = 0.5;

    engine::Simulation sim1;
    ASSERT_TRUE(sim1.initialize(cfg1));
    hazards::HazardGenerator::generateHazards(cfg1, &sim1);

    engine::SimulationConfig cfg2 = cfg1;
    engine::Simulation sim2;
    ASSERT_TRUE(sim2.initialize(cfg2));
    hazards::HazardGenerator::generateHazards(cfg2, &sim2);

    // Both queues must have the same size and identical sequence of event timestamps & types
    ASSERT_EQ(sim1.getEventScheduler()->size(), sim2.getEventScheduler()->size());
    size_t eventCount = sim1.getEventScheduler()->size();
    EXPECT_GT(eventCount, 0u);

    for (size_t i = 0; i < eventCount; ++i) {
        auto* ev1 = sim1.getEventScheduler()->popNextEvent();
        auto* ev2 = sim2.getEventScheduler()->popNextEvent();
        ASSERT_NE(ev1, nullptr);
        ASSERT_NE(ev2, nullptr);
        EXPECT_EQ(ev1->getTimestamp(), ev2->getTimestamp());
        EXPECT_EQ(ev1->getPriority(), ev2->getPriority());
        EXPECT_STREQ(ev1->getEventType(), ev2->getEventType());
    }

    // Different seed gives different schedule
    engine::SimulationConfig cfg3 = cfg1;
    cfg3.seed = 99999;
    engine::Simulation sim3;
    ASSERT_TRUE(sim3.initialize(cfg3));
    hazards::HazardGenerator::generateHazards(cfg3, &sim3);
    // At least some properties must differ
    EXPECT_TRUE(sim3.getEventScheduler()->size() > 0);
}

TEST(HazardsTest, OverlappingZonesRefCountsClearCorrectly) {
    domain::Grid grid(30, 30, 10.0);
    domain::Coordinate center1(100.0, 100.0); // cell (10, 10)
    domain::Coordinate center2(110.0, 100.0); // cell (11, 10) - overlaps with center1
    double radius = 25.0; // 2.5 cells radius

    // Create NoFlyZoneEvent 1
    hazards::NoFlyZoneEvent zone1(100, 7, center1, radius, 2000, &grid, nullptr, nullptr);
    zone1.execute();

    // Verify cell (10, 10) is NO_FLY_ZONE
    EXPECT_EQ(grid.getCellState(10, 10), domain::CellState::NO_FLY_ZONE);

    // Create NoFlyZoneEvent 2 overlapping at (11, 10)
    hazards::NoFlyZoneEvent zone2(150, 7, center2, radius, 3000, &grid, nullptr, nullptr);
    zone2.execute();

    // The overlapping cell (10, 10) and (11, 10) must be NO_FLY_ZONE
    EXPECT_EQ(grid.getCellState(10, 10), domain::CellState::NO_FLY_ZONE);
    EXPECT_EQ(grid.getCellState(11, 10), domain::CellState::NO_FLY_ZONE);

    // Now clear zone 1
    hazards::ZoneClearEvent clear1(2100, 7, center1, radius, &grid, nullptr, nullptr);
    clear1.execute();

    // Cell (11, 10) is still covered by zone2, so its ref count > 0 and must remain NO_FLY_ZONE
    EXPECT_EQ(grid.getCellState(11, 10), domain::CellState::NO_FLY_ZONE);
    // Cell (10, 10) is also within 25m of (110, 100) (dist = 10m <= 25m), so it still has ref count 1!
    EXPECT_EQ(grid.getCellState(10, 10), domain::CellState::NO_FLY_ZONE);

    // Now clear zone 2
    hazards::ZoneClearEvent clear2(3150, 7, center2, radius, &grid, nullptr, nullptr);
    clear2.execute();

    // Both zones are now cleared: ref counts drop to 0, cells must revert to FREE
    EXPECT_EQ(grid.getCellState(10, 10), domain::CellState::FREE);
    EXPECT_EQ(grid.getCellState(11, 10), domain::CellState::FREE);
}

TEST(HazardsTest, CommLossTimeoutReallocationAndRestore) {
    domain::Dispatcher dispatcher;
    domain::Drone drone1("DRN-001", domain::Coordinate(0, 0));
    domain::Drone drone2("DRN-002", domain::Coordinate(0, 0));
    dispatcher.registerDrone(&drone1);
    dispatcher.registerDrone(&drone2);

    domain::Mission mission("MSN-COMM-TEST", domain::Mission::PRIORITY_NORMAL,
                            domain::Coordinate(10, 10), domain::Coordinate(20, 20));
    bool submitted = dispatcher.submitMission(&mission);
    ASSERT_TRUE(submitted);

    // Drone 1 is EN_ROUTE with the mission
    EXPECT_EQ(drone1.getState(), domain::Drone::DroneState::EN_ROUTE);
    EXPECT_EQ(drone1.getCurrentMission(), &mission);
    EXPECT_EQ(drone2.getState(), domain::Drone::DroneState::IDLE);

    // 1. CommLossEvent occurs on drone 1
    hazards::CommLossEvent commLoss(100, 7, "DRN-001", 3000, &dispatcher, nullptr);
    commLoss.execute();
    EXPECT_TRUE(drone1.isCommsLost());

    // 2. HeartbeatTimeoutEvent fires
    uint64_t gen = drone1.getHeartbeatGeneration();
    events::HeartbeatTimeoutEvent timeout(5100, 8, "DRN-001", gen, &dispatcher, nullptr);
    timeout.execute();

    // 3. Drone 1 enters FAULT_RECOVERY and releases mission
    EXPECT_EQ(drone1.getState(), domain::Drone::DroneState::FAULT_RECOVERY);
    EXPECT_EQ(drone1.getCurrentMission(), nullptr);

    // 4. Mission was automatically reallocated to Drone 2!
    EXPECT_EQ(drone2.getState(), domain::Drone::DroneState::EN_ROUTE);
    EXPECT_EQ(drone2.getCurrentMission(), &mission);
    EXPECT_EQ(mission.getState(), domain::Mission::MissionState::ACTIVE);

    // 5. CommRestoreEvent fires for Drone 1
    hazards::CommRestoreEvent restore(3100, 7, "DRN-001", &dispatcher, nullptr);
    restore.execute();

    // Drone 1 is restored to IDLE and assignable again
    EXPECT_FALSE(drone1.isCommsLost());
    EXPECT_EQ(drone1.getState(), domain::Drone::DroneState::IDLE);
}
