# Code Tour — Drone Mesh Simulator (C++17)

A one-page guide for reading the simulator from entry point to worker execution.

---

## 1. Where to Start Reading

Read these files in order to understand the whole program quickly:

1. `src/main.cpp` — Parses CLI flags (`--scenario`, `--drones`, `--seed`, `--workers`, `--narrate`) and launches `ScenarioRunner::runDemo` or `ScenarioRunner::runStress`.
2. `src/engine/ScenarioRunner.cpp` — Sets up the `Simulation`, submits initial missions/hazards, runs `Simulation::start()` under a deadlock watchdog, and checks post-run invariants (`verifyInvariants`).
3. `src/engine/Simulation.cpp` — Owns the fleet, `Grid`, `Dispatcher`, `EventScheduler`, and `ThreadPool`; drives the virtual-time loop (`start()`) using a condition-variable drain barrier.
4. `src/domain/Dispatcher.cpp` — Core decision engine: assigns idle drones, manages the priority `missionQueue`, performs critical mission preemption (`evaluatePreemption`), handles fault reallocation (`reallocateMission`), and triggers A* replanning (`onGridChanged`).
5. `include/domain/Drone.hpp` & `src/domain/Drone.cpp` — Thread-safe drone state machine (`IDLE`, `EN_ROUTE`, `REROUTING`, `FAULT_RECOVERY`), battery clamping, route generation tracking, and `handleFault()` callback.

---

## 2. Lifecycle of One Event (`DispatchEvent` -> `Dispatcher` -> `Drone`)

Trace a critical organ-transplant dispatch (`MSN-CRIT-HEART` at `t = 100`) from scheduling to drone flight:

1. **Schedule (`ScenarioRunner::runDemo` -> `Simulation::scheduleSimulationEvent` -> `EventScheduler::scheduleEvent`)**
   - `ScenarioRunner.cpp` constructs `events::DispatchEvent(100, 10, mCrit.get(), disp)` and passes ownership to `Simulation::scheduleSimulationEvent()`.
   - `EventScheduler::scheduleEvent()` (`src/engine/EventScheduler.cpp`) stamps sequence number + wall-clock enqueue time and pushes the `Event*` into the priority queue (ordered by earliest `timestamp`, highest `priority`, FIFO sequence).
2. **Pop & Worker Dispatch (`Simulation::start` -> `ThreadPool::assignTask`)**
   - `Simulation::start()` (`src/engine/Simulation.cpp`) pops `nextEvent` (`timestamp == 100`), advances `virtualClock` to `100`, pops any other events at `t = 100`, and pushes them into `ThreadPool::taskQueue` (`ThreadSafeQueue<events::Event*>`).
   - `Simulation::start()` then calls `threadPool->waitUntilIdle()`, sleeping on `idleCv` until worker threads finish all `t = 100` events.
3. **Execute (`ThreadPool::workerThread` -> `DispatchEvent::execute`)**
   - A worker thread pops the `DispatchEvent*` and calls `task->execute()`.
   - `DispatchEvent::execute()` (`src/events/DispatchEvent.cpp`) sees `targetMission->isCritical()` is `true` and calls `dispatcher->evaluatePreemption(targetMission)`.
4. **Dispatcher Decision & Drone Mutation (`Dispatcher::evaluatePreemption` -> `Drone`)**
   - Under `dispatcherMutex`, `Dispatcher::evaluatePreemptionLocked()` (`src/domain/Dispatcher.cpp`) checks for an idle feasible drone; when none is idle, `findPreemptionCandidateLocked()` picks the closest drone running a lower-priority mission.
   - `interruptMissionLocked()` aborts/preempts the old mission, increments `targetDrone->incrementRouteGeneration()` (invalidating in-flight `WaypointArrivalEvent`s for the old route), plans an A* path (`routeDroneForMission` -> `PathPlanner::plan`), sets `targetDrone->setCurrentMission(newMission)` and state `EN_ROUTE`, and calls `reallocateMissionLocked(displacedMission)` so the displaced mission is re-queued (never dropped).
   - **Lock-safe scheduling**: `dispatcherMutex` is released *before* `scheduleDroneFlight()` schedules the first `WaypointArrivalEvent` and `HeartbeatTimeoutEvent` back into `Simulation`.

---

## 3. Course Concepts Map (File & Function Locations)

| Concept | Implementation | Real Simulator Usage |
| :--- | :--- | :--- |
| **(A) Basic-to-Class (Implicit)** | `Meters(double)` in `include/domain/Units.hpp` (`Meters`) | `reconstructPath()` in `src/domain/PathPlanner.cpp` (`Meters pathMeters(totalDistance)`) |
| **(A) Basic-to-Class (Explicit)** | `explicit BatteryLevel(float)` / `explicit BatteryLevel(double)` in `include/domain/BatteryLevel.hpp` | `Drone::setBatteryLevel()` and `Drone::drainBattery()` in `src/domain/Drone.cpp` |
| **(B) Class-to-Basic (Implicit)** | `SimTime::operator long()` in `include/domain/SimTime.hpp` | Verified in `tests/test_casting.cpp` (`SimTimeImplicitConversions`) |
| **(B) Class-to-Basic (Explicit)** | `explicit Drone::operator std::string()` (`src/domain/Drone.cpp`) and `explicit Mission::operator std::string()` (`src/domain/Mission.cpp`) | `ScenarioRunner::runDemo()` in `src/engine/ScenarioRunner.cpp` (`static_cast<std::string>(*d)`) |
| **(C) Class-to-Class** | `PlanarPoint(const Coordinate&)` & `PlanarPoint::operator Coordinate()` in `include/domain/PlanarPoint.hpp` | `reconstructPath()` in `src/domain/PathPlanner.cpp` converts grid `Coordinate` <-> `PlanarPoint` |
| **(C) Polymorphic `dynamic_cast`** | Pointer `dynamic_cast<const DispatchEvent*>` / `FaultEvent*` and reference `dynamic_cast<const DispatchEvent&>` | `checkEventAndPreemptionInvariants()` in `src/engine/ScenarioRunner.cpp` |
| **(D) Class Template** | `template <typename T> class ThreadSafeQueue` in `include/util/ThreadSafeQueue.hpp` | `ThreadPool::taskQueue` (`ThreadSafeQueue<events::Event*>`) in `include/concurrency/ThreadPool.hpp` |
| **(D) Function Template** | `template <typename T> constexpr T clampTo(T, T, T)` in `include/util/MathUtils.hpp` | `BatteryLevel` constructors in `include/domain/BatteryLevel.hpp` (`util::clampTo<float>`) |
