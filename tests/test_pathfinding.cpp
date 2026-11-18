#include <gtest/gtest.h>
#include "../include/domain/Grid.hpp"
#include "../include/domain/PathPlanner.hpp"
#include <cmath>

using namespace simulator::mesh::domain;

TEST(PathfindingTest, StraightPath) {
    Grid grid(10, 10, 10.0);
    auto pathOpt = PathPlanner::plan(grid, 0, 0, 0, 5);
    ASSERT_TRUE(pathOpt.has_value());
    
    const auto& path = pathOpt.value();
    // Path from (0,0) to (0,5) should have 6 waypoints: (0,0), (0,1), (0,2), (0,3), (0,4), (0,5)
    EXPECT_EQ(path.waypoints.size(), 6u);
    EXPECT_DOUBLE_EQ(path.totalDistance, 50.0);
    EXPECT_DOUBLE_EQ(path.waypoints.front().getLatitude(), 0.0);
    EXPECT_DOUBLE_EQ(path.waypoints.front().getLongitude(), 0.0);
    EXPECT_DOUBLE_EQ(path.waypoints.back().getLatitude(), 0.0);
    EXPECT_DOUBLE_EQ(path.waypoints.back().getLongitude(), 50.0);
}

TEST(PathfindingTest, PathAroundObstacles) {
    Grid grid(10, 10, 10.0);
    // Place a wall of obstacles across x=1..8 at y=5
    for (int x = 1; x <= 8; ++x) {
        grid.setStaticObstacle(x, 5);
    }
    
    // Plan path from (5, 2) to (5, 8)
    auto pathOpt = PathPlanner::plan(grid, 5, 2, 5, 8);
    ASSERT_TRUE(pathOpt.has_value());
    
    const auto& path = pathOpt.value();
    // Verify none of the waypoints pass through an obstacle
    for (const auto& wp : path.waypoints) {
        auto [cx, cy] = grid.coordinateToCell(wp);
        EXPECT_TRUE(grid.isPassable(cx, cy));
    }
}

TEST(PathfindingTest, NoPathReturnsNullopt) {
    Grid grid(10, 10, 10.0);
    // Completely enclose goal (5, 5) with obstacles
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx != 0 || dy != 0) {
                grid.setStaticObstacle(5 + dx, 5 + dy);
            }
        }
    }

    auto pathOpt = PathPlanner::plan(grid, 0, 0, 5, 5);
    EXPECT_FALSE(pathOpt.has_value());

    // Also goal is obstacle
    grid.setStaticObstacle(0, 5);
    EXPECT_FALSE(PathPlanner::plan(grid, 0, 0, 0, 5).has_value());
}

TEST(PathfindingTest, ZoneAppearingMidRouteChangesResult) {
    Grid grid(10, 10, 10.0);
    // First plan straight line from (0, 0) to (0, 5)
    auto snap1 = grid.snapshot();
    auto path1 = PathPlanner::plan(snap1, 0, 0, 0, 5);
    ASSERT_TRUE(path1.has_value());
    EXPECT_EQ(path1->waypoints.size(), 6u);

    // Now place NO_FLY_ZONE at (0, 3)
    grid.addNoFlyZoneCell(0, 3);
    EXPECT_GT(grid.getVersion(), snap1.getVersion());

    auto snap2 = grid.snapshot();
    auto path2 = PathPlanner::plan(snap2, 0, 0, 0, 5);
    ASSERT_TRUE(path2.has_value());

    // The new path must navigate around (0, 3) and thus have a different waypoint sequence
    bool intersectsNoFly = false;
    for (const auto& wp : path2->waypoints) {
        auto [cx, cy] = snap2.coordinateToCell(wp);
        if (cx == 0 && cy == 3) {
            intersectsNoFly = true;
        }
    }
    EXPECT_FALSE(intersectsNoFly);
    EXPECT_GT(path2->totalDistance, path1->totalDistance);
}

TEST(PathfindingTest, SnapshotIsolation) {
    Grid grid(10, 10, 10.0);
    auto snap = grid.snapshot();
    uint64_t snapVersion = snap.getVersion();

    // Mutate live grid heavily
    grid.setStaticObstacle(2, 2);
    grid.addNoFlyZoneCell(3, 3);
    grid.addNoFlyZoneCell(4, 4);

    EXPECT_GT(grid.getVersion(), snapVersion);
    EXPECT_FALSE(grid.isPassable(2, 2));
    EXPECT_FALSE(grid.isPassable(3, 3));

    // Snapshot remains untouched
    EXPECT_EQ(snap.getVersion(), snapVersion);
    EXPECT_TRUE(snap.isPassable(2, 2));
    EXPECT_TRUE(snap.isPassable(3, 3));

    // Planning on snap succeeds through cells that are blocked in live grid
    auto pathSnap = PathPlanner::plan(snap, 2, 0, 2, 4);
    ASSERT_TRUE(pathSnap.has_value());
    EXPECT_EQ(pathSnap->gridVersion, snapVersion);
}
