// extra_scenarios.cpp
// OPTIONAL stand-alone program (does not touch the original project code).
// It builds extra scenarios using the simulator's own classes and records each one
// as a replay file for ui/index.html.
//
// Usage:  extra_scenarios <scenario-name> <output.json> [seed]
// Names:  hospital_rush, storm_no_fly, comms_blackout, battery_crisis,
//         perfect_storm, rush_hour

#include "../include/engine/Simulation.hpp"
#include "../include/engine/ScenarioRunner.hpp"
#include "../include/domain/Mission.hpp"
#include "../include/domain/Coordinate.hpp"
#include "../include/events/DispatchEvent.hpp"
#include "../include/hazards/NoFlyZoneEvent.hpp"
#include "../include/hazards/CommLossEvent.hpp"
#include "../include/hazards/BatteryDrainEvent.hpp"
#include "../include/util/TraceRecorder.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

using namespace simulator::mesh;
using domain::Coordinate;
using domain::Mission;

namespace {

struct Ctx {
    engine::Simulation sim;
    std::vector<Mission*> all;
    domain::Dispatcher* disp{nullptr};
    domain::Grid* grid{nullptr};
};

engine::SimulationConfig baseConfig(int drones, int w, int h, uint64_t seed, long long durMs) {
    engine::SimulationConfig c;
    c.numDrones = drones;
    c.gridWidth = w;
    c.gridHeight = h;
    c.cellSizeMeters = 10.0;
    c.droneSpeed = 20.0;
    c.batteryDrainRate = 0.01;
    c.commLossProb = 0.0;      // hazards are scripted below, not random
    c.batteryDrainProb = 0.0;
    c.noFlyZoneProb = 0.0;
    c.seed = seed;
    c.workerThreadCount = 4;
    c.simulationDurationMs = durMs;
    return c;
}

void addMission(Ctx& c, const std::string& id, int pri, Coordinate pick, Coordinate drop, long long t) {
    auto m = std::make_unique<Mission>(id, pri, pick, drop);
    c.all.push_back(m.get());
    c.sim.scheduleSimulationEvent(std::make_unique<events::DispatchEvent>(t, pri, m.get(), c.disp));
    c.sim.registerMission(std::move(m));
}

bool noHaz() { return std::getenv("DRONE_NOHAZ") != nullptr; }

void addZone(Ctx& c, long long t, double x, double y, double radius, long long dur) {
    if (noHaz()) return;
    c.sim.scheduleSimulationEvent(std::make_unique<hazards::NoFlyZoneEvent>(
        t, 7, Coordinate(x, y), radius, dur, c.grid, c.disp, &c.sim));
}
void addCommLoss(Ctx& c, long long t, const std::string& drone, long long dur) {
    if (noHaz()) return;
    c.sim.scheduleSimulationEvent(std::make_unique<hazards::CommLossEvent>(
        t, 7, drone, dur, c.disp, &c.sim));
}
void addBatteryDrain(Ctx& c, long long t, const std::string& drone, double amount) {
    if (noHaz()) return;
    c.sim.scheduleSimulationEvent(std::make_unique<hazards::BatteryDrainEvent>(
        t, 7, drone, amount, c.disp, &c.sim));
}

// Descriptive mission names (so the UI shows what each drone is carrying)
const std::vector<std::string> NORMAL_NAMES = {"BLOOD-SAMPLES","PHARMACY-RX","LAB-RESULTS","VACCINE-BOX","IV-FLUIDS",
    "XRAY-FILMS","INSULIN-PACK","BANDAGE-KIT","ANTIBIOTICS","OXYGEN-MASK","SURGICAL-TOOLS","SALINE-BAGS"};
const std::vector<std::string> HIGH_NAMES   = {"BLOOD-BAG","ANTIVENOM","DEFIBRILLATOR","TRAUMA-KIT"};
const std::vector<std::string> LOW_NAMES    = {"ADMIN-FORMS","LINEN-PACK","MEAL-TRAY","SUPPLY-RESTOCK"};

// Short routine missions near the drone row (the dispatcher only accepts trips of ~90 m or less).
void routineMissions(Ctx& c, int count, uint64_t seed, long long tStart, long long tEnd, int pri,
                     const std::vector<std::string>& names, double maxX) {
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> px(10.0, maxX);
    std::uniform_real_distribution<double> py(10.0, 40.0);
    std::uniform_real_distribution<double> leg(-25.0, 25.0);
    for (int i = 0; i < count; ++i) {
        double x = px(rng), y = py(rng);
        double dx = std::max(10.0, x + leg(rng));
        double dy = std::max(10.0, y + 15.0 + std::abs(leg(rng)));
        long long t = tStart + (tEnd > tStart ? static_cast<long long>(rng() % (tEnd - tStart)) : 0);
        addMission(c, names[i % names.size()] + "-" + std::to_string(i + 1), pri, Coordinate(x, y), Coordinate(dx, dy), t);
    }
}

void build(const std::string& name, Ctx& c, uint64_t seed, engine::SimulationConfig& cfg) {
    if (name == "hospital_rush") {
        cfg = baseConfig(6, 20, 20, seed, 20000);
        if (!c.sim.initialize(cfg)) std::exit(2);
        c.disp = c.sim.getDispatcher(); c.grid = c.sim.getGrid();
        routineMissions(c, 8, seed + 1, 0, 3000, Mission::PRIORITY_NORMAL, NORMAL_NAMES, 60.0);
        addMission(c, "ORGAN-HEART",  Mission::PRIORITY_CRITICAL, Coordinate(20, 20), Coordinate(45, 45), 1000);
        addMission(c, "ORGAN-KIDNEY", Mission::PRIORITY_CRITICAL, Coordinate(30, 15), Coordinate(55, 40), 2500);
        addMission(c, "ORGAN-LIVER",  Mission::PRIORITY_CRITICAL, Coordinate(15, 25), Coordinate(40, 50), 4000);
    } else if (name == "storm_no_fly") {
        cfg = baseConfig(6, 20, 20, seed, 22000);
        if (!c.sim.initialize(cfg)) std::exit(2);
        c.disp = c.sim.getDispatcher(); c.grid = c.sim.getGrid();
        routineMissions(c, 10, seed + 2, 0, 4000, Mission::PRIORITY_NORMAL, NORMAL_NAMES, 60.0);
        addMission(c, "MED-SUPPLY", Mission::PRIORITY_HIGH, Coordinate(20, 15), Coordinate(50, 45), 1500);
        addZone(c, 1500, 35, 30, 12, 3500);
        addZone(c, 3500, 50, 25, 12, 3500);
    } else if (name == "comms_blackout") {
        cfg = baseConfig(8, 20, 20, seed, 30000);
        cfg.droneSpeed = 8.0;   // slower drones stay in the air long enough for the blackout to matter
        if (!c.sim.initialize(cfg)) std::exit(2);
        c.disp = c.sim.getDispatcher(); c.grid = c.sim.getGrid();
        routineMissions(c, 10, seed + 3, 0, 4000, Mission::PRIORITY_NORMAL, NORMAL_NAMES, 70.0);
        addMission(c, "ORGAN-LUNG", Mission::PRIORITY_CRITICAL, Coordinate(25, 20), Coordinate(55, 45), 1500);
        // targets chosen because these drones are mid-flight at those moments
        addCommLoss(c, 2200, "DRN-004", 4000);
        addCommLoss(c, 3000, "DRN-001", 5000);
        addCommLoss(c, 3200, "DRN-002", 6000);
        addCommLoss(c, 3500, "DRN-007", 5000);
    } else if (name == "battery_crisis") {
        cfg = baseConfig(5, 20, 20, seed, 22000);
        if (!c.sim.initialize(cfg)) std::exit(2);
        c.disp = c.sim.getDispatcher(); c.grid = c.sim.getGrid();
        routineMissions(c, 8, seed + 4, 0, 4000, Mission::PRIORITY_NORMAL, NORMAL_NAMES, 60.0);
        addMission(c, "ORGAN-CORNEA", Mission::PRIORITY_CRITICAL, Coordinate(20, 20), Coordinate(50, 40), 1500);
        // targets chosen because these drones are mid-flight at those moments
        addBatteryDrain(c, 3000, "DRN-001", 88.0);
        addBatteryDrain(c, 3400, "DRN-003", 82.0);
        addBatteryDrain(c, 3800, "DRN-002", 90.0);
    } else if (name == "perfect_storm") {
        cfg = baseConfig(10, 25, 25, seed, 26000);
        if (!c.sim.initialize(cfg)) std::exit(2);
        c.disp = c.sim.getDispatcher(); c.grid = c.sim.getGrid();
        routineMissions(c, 14, seed + 5, 0, 5000, Mission::PRIORITY_NORMAL, NORMAL_NAMES, 80.0);
        routineMissions(c, 4, seed + 6, 500, 5000, Mission::PRIORITY_HIGH, HIGH_NAMES, 80.0);
        addMission(c, "ORGAN-HEART", Mission::PRIORITY_CRITICAL, Coordinate(25, 20), Coordinate(55, 45), 1200);
        addMission(c, "ORGAN-KIDNEY", Mission::PRIORITY_CRITICAL, Coordinate(40, 15), Coordinate(60, 45), 3500);
        addZone(c, 900,  35, 30, 18, 7000);
        addZone(c, 3000, 55, 25, 20, 6000);
        addCommLoss(c, 1000, "DRN-002", 5000);
        addCommLoss(c, 2500, "DRN-006", 6000);
        addBatteryDrain(c, 1800, "DRN-004", 85.0);
    } else if (name == "rush_hour") {
        cfg = baseConfig(15, 20, 20, seed, 22000);
        if (!c.sim.initialize(cfg)) std::exit(2);
        c.disp = c.sim.getDispatcher(); c.grid = c.sim.getGrid();
        routineMissions(c, 20, seed + 7, 0, 6000, Mission::PRIORITY_NORMAL, NORMAL_NAMES, 120.0);
        routineMissions(c, 6, seed + 8, 0, 6000, Mission::PRIORITY_HIGH, HIGH_NAMES, 120.0);
        routineMissions(c, 4, seed + 9, 0, 6000, Mission::PRIORITY_LOW, LOW_NAMES, 120.0);
    } else {
        std::cerr << "Unknown scenario: " << name << "\n";
        std::exit(1);
    }
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: extra_scenarios <name> <out.json> [seed]\n";
        return 1;
    }
    std::string name = argv[1], out = argv[2];
    uint64_t seed = (argc > 3) ? std::stoull(argv[3]) : 42;

    util::TraceRecorder::instance().enable(name, seed);
    Ctx ctx;
    engine::SimulationConfig cfg;
    build(name, ctx, seed, cfg);

    ctx.sim.start();

    auto inv = engine::ScenarioRunner::verifyInvariants(ctx.sim, ctx.all, false);
    util::TraceRecorder::instance().writeFiles(out);

    int done = 0, pending = 0, aborted = 0;
    for (auto* m : ctx.all) {
        switch (m->getState()) {
            case Mission::MissionState::COMPLETED: ++done; break;
            case Mission::MissionState::ABORTED: ++aborted; break;
            default: ++pending; break;
        }
    }
    std::cout << "[" << name << "] missions=" << ctx.all.size() << " completed=" << done
              << " pending/active=" << pending << " aborted=" << aborted
              << " invariants=" << (inv.passed ? "PASS" : "FAIL") << "\n";
    for (auto& v : inv.violations) std::cout << "  violation: " << v << "\n";
    return inv.passed ? 0 : 3;
}
