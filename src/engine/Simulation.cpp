#include "../../include/engine/Simulation.hpp"
#include "../../include/domain/Drone.hpp"
#include "../../include/domain/GISProvider.hpp"
#include "../../include/util/TraceRecorder.hpp"
#include <iostream>
#include <cstdio>

namespace simulator {
namespace mesh {
namespace engine {

namespace {

int deployFleet(domain::Grid* grid, domain::Dispatcher* dispatcher,
                std::vector<std::unique_ptr<domain::Drone>>& fleetDrones,
                int width, int height, int numDrones) {
    fleetDrones.clear();
    int placed = 0;
    for (int y = 0; y < height && placed < numDrones; ++y) {
        for (int x = 0; x < width && placed < numDrones; ++x) {
            if (!grid->isPassable(x, y)) continue;
            placed++;
            char idBuf[32];
            std::snprintf(idBuf, sizeof(idBuf), "DRN-%03d", placed);
            auto drone = std::make_unique<domain::Drone>(idBuf, grid->cellToCoordinate(x, y));
            dispatcher->registerDrone(drone.get());
            fleetDrones.push_back(std::move(drone));
        }
    }
    return placed;
}

void dispatchSameTimestampEvents(EventScheduler* scheduler,
                                 concurrency::ThreadPool* pool,
                                 long long evTime) {
    while (true) {
        events::Event* peeked = scheduler->peekNextEvent();
        if (!peeked || peeked->getTimestamp() != evTime) {
            break;
        }
        pool->assignTask(scheduler->popNextEvent());
    }
}

} // anonymous namespace

Simulation::Simulation()
    : isRunning(false), simState(SimulationState::UNINITIALIZED), virtualClock(0) {}

Simulation::~Simulation() {
    stop();
}

void Simulation::initialize() {
    initialize(SimulationConfig());
}

bool Simulation::initialize(const SimulationConfig& cfg) {
    config = cfg;
    auto errors = config.validate();
    if (!errors.empty()) {
        std::cerr << "[Simulation] Configuration validation failed with " << errors.size() << " error(s):\n";
        for (const auto& err : errors) {
            std::cerr << "  - " << err << "\n";
        }
        isRunning.store(false, std::memory_order_release);
        simState.store(SimulationState::UNINITIALIZED, std::memory_order_release);
        return false;
    }

    try {
        // 1. Synthetic GIS Provider loads grid
        domain::GISProvider gisProvider;
        grid = std::make_unique<domain::Grid>(gisProvider.loadGrid(config));

        // 2. Metrics & ThreadPool initialization
        metrics = std::make_unique<metrics::Metrics>(config.workerThreadCount);
        threadPool = std::make_unique<concurrency::ThreadPool>(config.workerThreadCount, metrics.get());
        eventScheduler = std::make_unique<EventScheduler>();

        // 3. Dispatcher setup
        dispatcher = std::make_unique<domain::Dispatcher>(eventScheduler.get());
        dispatcher->setGrid(grid.get());
        dispatcher->setSimulation(this);

        // 4. Deploy N drones on free grid cells
        int placed = deployFleet(grid.get(), dispatcher.get(), fleetDrones,
                                 config.gridWidth, config.gridHeight, config.numDrones);

        if (placed < config.numDrones) {
            std::cerr << "[Simulation] Warning: Not enough free grid cells to place all " 
                      << config.numDrones << " drones (placed " << placed << ").\n";
        }

        virtualClock.store(0, std::memory_order_release);

        // Transition to READY, and only then set isRunning=true
        simState.store(SimulationState::READY, std::memory_order_release);
        isRunning.store(true, std::memory_order_release);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[Simulation] Exception during initialize(): " << e.what() << std::endl;
        isRunning.store(false, std::memory_order_release);
        simState.store(SimulationState::UNINITIALIZED, std::memory_order_release);
        return false;
    } catch (...) {
        std::cerr << "[Simulation] Unknown exception during initialize()." << std::endl;
        isRunning.store(false, std::memory_order_release);
        simState.store(SimulationState::UNINITIALIZED, std::memory_order_release);
        return false;
    }
}

void Simulation::scheduleSimulationEvent(std::unique_ptr<events::Event> event) {
    if (!event) return;
    std::lock_guard<std::mutex> lock(eventsMutex);
    events::Event* raw = event.get();
    allocatedEvents.push_back(std::move(event));
    if (eventScheduler) {
        eventScheduler->scheduleEvent(raw);
    }
}

void Simulation::registerMission(std::unique_ptr<domain::Mission> mission) {
    if (!mission) return;
    std::lock_guard<std::mutex> lock(eventsMutex);
    createdMissions.push_back(std::move(mission));
}

void Simulation::start() {
    if (!isRunning.load(std::memory_order_acquire) ||
        simState.load(std::memory_order_acquire) != SimulationState::READY) {
        return;
    }

    simState.store(SimulationState::RUNNING, std::memory_order_release);
    if (metrics) {
        metrics->recordStart();
    }
    if (threadPool) {
        threadPool->startPool();
    }

    virtualClock.store(0, std::memory_order_release);

    // UI trace: record the starting state (does nothing unless --trace was given)
    util::TraceRecorder::instance().captureFrame(*this);

    /*
     * VIRTUAL CLOCK SIMULATION LOOP WITH DRAIN BARRIER:
     * 
     * 1. Pops the next due event from EventScheduler.
     * 2. Advances the virtual clock to that event's timestamp.
     * 3. Dispatches the event and all subsequent events sharing the exact same
     *    timestamp to the ThreadPool so they execute concurrently across worker threads.
     * 4. DRAIN BARRIER: threadPool->waitUntilIdle() blocks via condition_variable
     *    (zero busy-waiting) until all worker threads finish processing tasks for
     *    the current timestamp.
     * 5. If tasks running at that timestamp scheduled additional events at the
     *    same timestamp, those are executed before advancing the clock.
     * 6. The clock advances monotonically and never goes backward.
     */
    while (isRunning.load(std::memory_order_acquire)) {
        events::Event* nextEvent = eventScheduler->popNextEvent();
        if (!nextEvent) {
            // Event queue drained to quiescence
            break;
        }

        long long evTime = nextEvent->getTimestamp();
        if (evTime > config.simulationDurationMs) {
            // Reached simulation duration limit
            break;
        }

        long long currentClock = virtualClock.load(std::memory_order_acquire);
        if (evTime > currentClock) {
            virtualClock.store(evTime, std::memory_order_release);
        }

        threadPool->assignTask(nextEvent);
        dispatchSameTimestampEvents(eventScheduler.get(), threadPool.get(), evTime);

        // Drain barrier: wait until all worker threads finish executing tasks at current timestamp
        threadPool->waitUntilIdle();

        // UI trace: workers are idle here, so the state is consistent
        util::TraceRecorder::instance().captureFrame(*this);
    }

    stop();
}

void Simulation::stop() {
    bool expected = true;
    if (isRunning.compare_exchange_strong(expected, false, std::memory_order_acq_rel)) {
        simState.store(SimulationState::STOPPED, std::memory_order_release);
        if (threadPool) {
            threadPool->shutdown();
        }
        if (metrics) {
            metrics->recordEnd();
        }
    }
}

} // namespace engine
} // namespace mesh
} // namespace simulator
