# Drone Mesh Simulator (C++17)

A concurrent, discrete-event drone mesh delivery simulator written in Modern C++17. The simulator models autonomous delivery fleets navigating dynamic environments with static obstacles, spreading no-fly zones, communication failures, battery depletion, dynamic preemption of routine deliveries for critical organ transplants, and autonomous A* path replanning.

For a guided walkthrough of the codebase and where each C++ concept lives, see [docs/CODE_TOUR.md](docs/CODE_TOUR.md).

---

## Architecture: Virtual Time vs. Wall-Clock Metrics

```text
  main.cpp ──> ScenarioRunner ──> Simulation (virtualClock + drain barrier)
                                     │
         ┌───────────────────────────┼───────────────────────────┐
         ▼                           ▼                           ▼
   EventScheduler              ThreadPool                    Metrics
 (min-heap by time,       (ThreadSafeQueue<Event*>        (mutex-protected
  max by priority, FIFO)   + worker threads)               sample vectors)
         │                           │
         └──────────────> Event::execute() ──> Dispatcher ──> Drone / Grid / PathPlanner
```

### 1. Virtual Time (Discrete-Event Simulation)
- **Virtual Clock (`long long`, ms)**: The simulation timeline progresses purely based on discrete scheduled events (`timestamp` in ms) rather than sleeping or polling real time.
- **Monotonic Advancement**: `Simulation::start()` pops the next due event from `EventScheduler` and advances `virtualClock` to that event's timestamp.
- **Concurrent Dispatch at Same Timestamp**: Events sharing the exact same virtual timestamp are popped together (`dispatchSameTimestampEvents`) and pushed into `ThreadPool` so worker threads execute them concurrently.
- **Condition-Variable Drain Barrier**: `ThreadPool::waitUntilIdle()` blocks via `std::condition_variable` (`idleCv`, zero busy-waiting) until `inFlightTasks` drops to zero before advancing the virtual clock.

### 2. Wall-Clock Metrics & Telemetry
- **Queue Latency & Durations**: Events are timestamped with `std::chrono::steady_clock::now()` when scheduled in `EventScheduler::scheduleEvent()`, and worker threads record queue latency and handler duration in `Metrics::recordEventExecution()`.
- **Percentile Summary**: `Metrics` stores samples in `std::vector<double>` guarded by `metricsMutex` and computes p50/p95/p99 percentiles by sorting at summary/export time (`printSummaryTable()`, `writeMetricsJson()`).

---

## Lock Ordering & Concurrency Architecture

To guarantee zero deadlocks and zero data races under ThreadSanitizer, lock acquisition strictly follows this level order:

1. **Level 1: Live Grid lock (`domain::Grid::gridMutex` - `std::shared_mutex`)**
   - Readers create immutable snapshots via `Grid::snapshot()` (acquiring `shared_lock` briefly during the copy).
   - Writers (no-fly zone placement and clearing) acquire `unique_lock`.
   - A* path planning (`PathPlanner::plan`) operates exclusively on immutable `Grid` snapshots and holds **no** locks during search.
2. **Level 2: Dispatcher lock (`domain::Dispatcher::dispatcherMutex` - `std::mutex`)**
   - Protects fleet registry, active missions, `missionQueue`, completed missions, and audit sequence counters.
3. **Level 3: Per-Drone lock (`domain::Drone::droneMutex` - `std::mutex`)**
   - Protects drone state, position, battery level, `commsLost` flag, `activePath`, route generations, and heartbeat counters.
4. **Level 4: Scheduler & ThreadPool locks**
   - `engine::EventScheduler::queueMutex` (`std::mutex`) protects the event priority queue.
   - `util::ThreadSafeQueue<events::Event*>::mutex_` (`std::mutex`) protects the worker task queue.
   - `concurrency::ThreadPool::idleMutex` (`std::mutex`) coordinates the `idleCv` drain barrier.

**Critical Concurrency Invariant**: `Dispatcher` never calls `Simulation::scheduleSimulationEvent()` or invokes `Drone` callbacks while holding `dispatcherMutex`, and `Drone::handleFault()` releases `droneMutex` before calling `faultCallback`.

---

## Repository Structure

```text
drone-mesh-simulator-cpp/
├── CMakeLists.txt
├── README.md
├── docs/
│   └── CODE_TOUR.md                  # Reading guide, event trace, course concept map
├── include/
│   ├── concurrency/                  # Logger.hpp, ThreadPool.hpp
│   ├── domain/                       # BatteryLevel, Coordinate, Dispatcher, Drone,
│   │                                 # GISProvider, Grid, Mission, PathPlanner,
│   │                                 # PlanarPoint, Priority, SimTime, Units
│   ├── engine/                       # EventScheduler, ScenarioRunner, Simulation, SimulationConfig
│   ├── events/                       # Event base + Dispatch, Fault, FaultRecovery,
│   │                                 # HeartbeatTimeout, MissionComplete, Replan, WaypointArrival
│   ├── hazards/                      # BatteryDrainEvent, CommLossEvent, HazardGenerator, NoFlyZoneEvent
│   ├── metrics/                      # Metrics.hpp
│   └── util/                         # MathUtils.hpp (clampTo<T>), ThreadSafeQueue.hpp
├── src/                              # Implementations matching include/ + main.cpp
└── tests/                            # 8 GoogleTest executables
```

---

## Building the Simulator

### Prerequisites
- Modern C++17 compiler (Clang 10+ or GCC 9+)
- CMake 3.14+
- POSIX threads (`pthread`)

### Standard Build
```bash
cmake -B build -S .
cmake --build build -j
```

### Build with ThreadSanitizer (TSan) - Recommended
To verify zero data races under full concurrency:
```bash
cmake -B build -S . -DENABLE_TSAN=ON -DCMAKE_CXX_FLAGS=""
cmake --build build -j
```

### Build with AddressSanitizer (ASan)
To verify memory safety (buffer bounds, zero leaks):
```bash
cmake -B build -S . -DENABLE_ASAN=ON -DENABLE_TSAN=OFF -DCMAKE_CXX_FLAGS=""
cmake --build build -j
```
*(Note: CMake enforces that `-DENABLE_TSAN=ON` and `-DENABLE_ASAN=ON` cannot be enabled simultaneously).*

---

## Running the Test Suite

Run all 8 unit and integration test suites via CTest:
```bash
ctest --test-dir build --output-on-failure
```

Individual test suites:
- `./build/test_event_queue` (Event ordering, FIFO tie-breaking, concurrent enqueue)
- `./build/test_thread_pool` (Thread pool task execution, shutdown, drain barrier)
- `./build/test_dispatcher` (UC3 mission submission, UC5 preemption, state machine)
- `./build/test_pathfinding` (A* 8-connected path search, obstacles, snapshot isolation)
- `./build/test_hazards` (Deterministic hazard generator seeds, zone ref counts, comm loss)
- `./build/test_integration` (Full `demo` and `stress` scenario runs with invariant checks)
- `./build/test_casting` (Basic-to-Class, Class-to-Basic, Class-to-Class, `dynamic_cast`)
- `./build/test_templates` (`ThreadSafeQueue<T>` and `clampTo<T>`)

---

## Running Scenarios (`drone_simulator`)

```text
Usage: ./build/drone_simulator [options]
Options:
  --drones N       Number of drones in fleet (default: 3 for demo, 50 for stress)
  --missions M     Number of missions (default: 4 for demo, 300 for stress)
  --seed S         Random number generator seed (default: 42)
  --workers W      Worker thread count for ThreadPool (default: 4)
  --scenario NAME  Scenario to execute: 'demo' or 'stress' (default: demo)
  --narrate        Print simulation metrics and summary table (default: on)
  -h, --help       Show this help message
```

### 1. Scripted Demo Scenario
Executes a deterministic scripted workflow demonstrating:
- A 3-drone fleet initialized and assigned routine deliveries.
- Injection of a Critical Organ (Heart) Transplant delivery (`MSN-CRIT-HEART`) at `t=100` while all drones are busy, triggering live dynamic preemption (UC5) of a routine delivery.
- A dynamic No-Fly Zone appearing at `t=200` across active routes, triggering automated A* replanning.
- Simulated communication loss (`t=300`) with mission reallocation and subsequent comm restore.

```bash
./build/drone_simulator --scenario demo
```

### 2. High-Concurrency Stress Scenario
Executes a large-scale simulation with 50 drones, 300 missions of mixed priorities, and concurrent randomized hazards:

```bash
./build/drone_simulator --scenario stress --seed 42 --workers 4
```

At the end of each run, `ScenarioRunner::verifyInvariants()` validates all mission, drone, and event invariants, prints the performance summary table, and writes `metrics.json`.
