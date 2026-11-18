#include <gtest/gtest.h>
#include "../include/engine/ScenarioRunner.hpp"

using namespace simulator::mesh::engine;

TEST(IntegrationTest, FullDemoScenarioRunsAndPassesAllInvariants) {
    SimulationConfig config;
    config.seed = 42;
    bool success = ScenarioRunner::runDemo(config, false);
    EXPECT_TRUE(success);
}

TEST(IntegrationTest, FullStressScenarioRunsAndPassesAllInvariants) {
    SimulationConfig config;
    config.seed = 42;
    config.workerThreadCount = 4;
    bool success = ScenarioRunner::runStress(config, false);
    EXPECT_TRUE(success);
}
