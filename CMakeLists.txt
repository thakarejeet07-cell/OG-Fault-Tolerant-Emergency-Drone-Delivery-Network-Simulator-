cmake_minimum_required(VERSION 3.14)

if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

project(DroneMeshSimulator CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")

option(ENABLE_TSAN "Enable ThreadSanitizer" OFF)
option(ENABLE_ASAN "Enable AddressSanitizer" OFF)

if(ENABLE_TSAN AND ENABLE_ASAN)
  message(FATAL_ERROR "Cannot enable both ENABLE_TSAN and ENABLE_ASAN at the same time")
endif()

if(ENABLE_TSAN)
  message(STATUS "ThreadSanitizer enabled: -fsanitize=thread -g -O1")
  add_compile_options(-fsanitize=thread -g -O1)
  add_link_options(-fsanitize=thread)
elseif(ENABLE_ASAN)
  message(STATUS "AddressSanitizer enabled: -fsanitize=address -g -O1")
  add_compile_options(-fsanitize=address -g -O1)
  add_link_options(-fsanitize=address)
endif()

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
# Suppress deprecation warnings from third-party dependencies (googletest)
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

find_package(Threads REQUIRED)

add_library(simulator_lib
    src/domain/Coordinate.cpp
    src/domain/Grid.cpp
    src/domain/GISProvider.cpp
    src/domain/PathPlanner.cpp
    src/domain/Dispatcher.cpp
    src/domain/Drone.cpp
    src/domain/Mission.cpp
    src/concurrency/ThreadPool.cpp
    src/engine/EventScheduler.cpp
    src/engine/Simulation.cpp
    src/engine/ScenarioRunner.cpp
    src/events/FaultEvent.cpp
    src/events/FaultRecoveryEvent.cpp
    src/events/DispatchEvent.cpp
    src/events/WaypointArrivalEvent.cpp
    src/events/MissionCompleteEvent.cpp
    src/events/HeartbeatTimeoutEvent.cpp
    src/events/ReplanEvent.cpp
    src/hazards/CommLossEvent.cpp
    src/hazards/BatteryDrainEvent.cpp
    src/hazards/NoFlyZoneEvent.cpp
    src/hazards/HazardGenerator.cpp
    src/metrics/Metrics.cpp
)
target_include_directories(simulator_lib PUBLIC include)
target_link_libraries(simulator_lib PRIVATE Threads::Threads)

# Main simulation executable
add_executable(drone_simulator src/main.cpp)
target_link_libraries(drone_simulator PRIVATE simulator_lib Threads::Threads)

# Standalone extra scenarios executable
add_executable(extra_scenarios tools/extra_scenarios.cpp)
target_link_libraries(extra_scenarios PRIVATE simulator_lib Threads::Threads)

enable_testing()

add_executable(test_event_queue tests/test_event_queue.cpp)
target_link_libraries(test_event_queue PRIVATE simulator_lib gtest_main Threads::Threads)
add_test(NAME test_event_queue COMMAND test_event_queue)

add_executable(test_thread_pool tests/test_thread_pool.cpp)
target_link_libraries(test_thread_pool PRIVATE simulator_lib gtest_main Threads::Threads)
add_test(NAME test_thread_pool COMMAND test_thread_pool)

add_executable(test_dispatcher tests/test_dispatcher.cpp)
target_link_libraries(test_dispatcher PRIVATE simulator_lib gtest_main Threads::Threads)
add_test(NAME test_dispatcher COMMAND test_dispatcher)

add_executable(test_pathfinding tests/test_pathfinding.cpp)
target_link_libraries(test_pathfinding PRIVATE simulator_lib gtest_main Threads::Threads)
add_test(NAME test_pathfinding COMMAND test_pathfinding)

add_executable(test_hazards tests/test_hazards.cpp)
target_link_libraries(test_hazards PRIVATE simulator_lib gtest_main Threads::Threads)
add_test(NAME test_hazards COMMAND test_hazards)

add_executable(test_integration tests/test_integration.cpp)
target_link_libraries(test_integration PRIVATE simulator_lib gtest_main Threads::Threads)
add_test(NAME test_integration COMMAND test_integration)

add_executable(test_casting tests/test_casting.cpp)
target_link_libraries(test_casting PRIVATE simulator_lib gtest_main Threads::Threads)
add_test(NAME test_casting COMMAND test_casting)

add_executable(test_templates tests/test_templates.cpp)
target_link_libraries(test_templates PRIVATE simulator_lib gtest_main Threads::Threads)
add_test(NAME test_templates COMMAND test_templates)
