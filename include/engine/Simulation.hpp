#ifndef DRONE_MESH_SIMULATION_HPP
#define DRONE_MESH_SIMULATION_HPP

#include "EventScheduler.hpp"
#include "SimulationConfig.hpp"
#include "../concurrency/ThreadPool.hpp"
#include "../domain/Grid.hpp"
#include "../domain/Dispatcher.hpp"
#include "../domain/Drone.hpp"
#include "../domain/Mission.hpp"
#include "../metrics/Metrics.hpp"
#include <memory>
#include <atomic>
#include <vector>
#include <mutex>

namespace simulator {
namespace mesh {
namespace engine {

enum class SimulationState {
    UNINITIALIZED,
    READY,
    RUNNING,
    STOPPED
};

class Simulation {
private:
    SimulationConfig config;

    std::unique_ptr<concurrency::ThreadPool> threadPool;
    std::unique_ptr<EventScheduler> eventScheduler;
    std::unique_ptr<domain::Dispatcher> dispatcher;
    std::unique_ptr<domain::Grid> grid;
    std::unique_ptr<metrics::Metrics> metrics;

    std::vector<std::unique_ptr<domain::Drone>> fleetDrones;
    std::vector<std::unique_ptr<domain::Mission>> createdMissions;

    std::atomic<bool> isRunning{false};
    std::atomic<SimulationState> simState{SimulationState::UNINITIALIZED};
    std::atomic<long long> virtualClock{0};

    mutable std::mutex eventsMutex;
    std::vector<std::unique_ptr<events::Event>> allocatedEvents;

public:
    Simulation();
    ~Simulation();

    // Prevent copy/move
    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;

    void initialize();
    bool initialize(const SimulationConfig& cfg);

    // Runs the simulation loop using virtual clock and drain barrier
    void start();
    void stop();

    bool getIsRunning() const { return isRunning.load(std::memory_order_acquire); }
    SimulationState getState() const { return simState.load(std::memory_order_acquire); }
    long long getVirtualTime() const { return virtualClock.load(std::memory_order_acquire); }

    // Thread-safe dynamic event scheduling
    void scheduleSimulationEvent(std::unique_ptr<events::Event> event);

    domain::Dispatcher* getDispatcher() const { return dispatcher.get(); }
    domain::Grid* getGrid() const { return grid.get(); }
    metrics::Metrics* getMetrics() const { return metrics.get(); }
    EventScheduler* getEventScheduler() const { return eventScheduler.get(); }
    const SimulationConfig& getConfig() const { return config; }
    const std::vector<std::unique_ptr<domain::Mission>>& getCreatedMissions() const { return createdMissions; }
    const std::vector<std::unique_ptr<events::Event>>& getAllocatedEvents() const { return allocatedEvents; }
    void registerMission(std::unique_ptr<domain::Mission> mission);
};

} // namespace engine
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_SIMULATION_HPP
