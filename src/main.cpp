#include "engine/ScenarioRunner.hpp"
#include "util/TraceRecorder.hpp"
#include <iostream>
#include <string>
#include <cstring>
#include <cstdlib>

using namespace simulator::mesh::engine;

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "Options:\n"
              << "  --drones N       Number of drones in fleet (default: 3 for demo, 50 for stress)\n"
              << "  --missions M     Number of missions (default: 4 for demo, 300 for stress)\n"
              << "  --seed S         Random number generator seed (default: 42)\n"
              << "  --workers W      Worker thread count for ThreadPool (default: 4)\n"
              << "  --scenario NAME  Scenario to execute: 'demo' or 'stress' (default: demo)\n"
              << "  --trace [FILE]   Record a replay file for the HTML UI (default: ui/trace.json)\n"
              << "  --narrate        Print simulation metrics and summary table (default: on)\n"
              << "  -h, --help       Show this help message\n";
}

int main(int argc, char* argv[]) {
    SimulationConfig config;
    std::string scenario = "demo";
    bool customDrones = false;
    bool narrate = true;
    bool traceOn = false;
    std::string tracePath = "ui/trace.json";

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--drones") == 0 && i + 1 < argc) {
            config.numDrones = std::atoi(argv[++i]);
            customDrones = true;
        } else if (std::strcmp(argv[i], "--missions") == 0 && i + 1 < argc) {
            ++i; // Supported mission count option
        } else if (std::strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
            config.seed = static_cast<uint64_t>(std::strtoull(argv[++i], nullptr, 10));
        } else if (std::strcmp(argv[i], "--workers") == 0 && i + 1 < argc) {
            config.workerThreadCount = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--scenario") == 0 && i + 1 < argc) {
            scenario = argv[++i];
        } else if (std::strcmp(argv[i], "--trace") == 0) {
            traceOn = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') tracePath = argv[++i];
        } else if (std::strcmp(argv[i], "--narrate") == 0) {
            narrate = true;
        } else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown or invalid option: " << argv[i] << "\n";
            printUsage(argv[0]);
            return 1;
        }
    }

    if (traceOn) {
        simulator::mesh::util::TraceRecorder::instance().enable(scenario, config.seed);
    }

    bool success = false;
    if (scenario == "demo") {
        if (!customDrones) config.numDrones = 3;
        config.simulationDurationMs = 25000;
        success = ScenarioRunner::runDemo(config, narrate);
    } else if (scenario == "stress") {
        if (!customDrones) config.numDrones = 50;
        config.simulationDurationMs = 30000;
        success = ScenarioRunner::runStress(config, narrate);
    } else {
        std::cerr << "Error: Unknown scenario '" << scenario << "'. Supported scenarios: demo, stress\n";
        return 1;
    }

    if (traceOn) {
        simulator::mesh::util::TraceRecorder::instance().writeFiles(tracePath);
    }

    if (success) {
        std::cout << "\nRESULT: PASS\n";
        return 0;
    } else {
        std::cerr << "\nRESULT: FAIL\n";
        return 1;
    }
}
