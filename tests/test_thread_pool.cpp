#include <gtest/gtest.h>
#include "../include/concurrency/ThreadPool.hpp"
#include "../include/events/Event.hpp"
#include <thread>
#include <atomic>
#include <chrono>

using namespace simulator::mesh;

class TrackedEvent : public events::Event {
public:
    std::atomic<bool> executed{false};
    std::thread::id executionThreadId;

    void execute() override {
        // Sleep to simulate work and ensure tasks pile up
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        executionThreadId = std::this_thread::get_id();
        executed = true;
    }
    long long getTimestamp() const override { return 0; }
    int getPriority() const override { return 0; }
};

TEST(ThreadPoolTest, ExecutesOnWorkerThread) {
    concurrency::ThreadPool pool(2);
    pool.startPool();
    
    TrackedEvent e;
    std::thread::id mainThreadId = std::this_thread::get_id();
    
    pool.assignTask(&e);
    
    pool.shutdown(); // waits for the task to finish
    
    EXPECT_TRUE(e.executed);
    EXPECT_NE(e.executionThreadId, mainThreadId);
}

TEST(ThreadPoolTest, ShutdownDoesNotDropTasks) {
    concurrency::ThreadPool pool(2);
    pool.startPool();
    
    std::vector<TrackedEvent> events(10);
    for (int i = 0; i < 10; ++i) {
        pool.assignTask(&events[i]);
    }
    
    // Shutting down while tasks are likely still in queue or running
    pool.shutdown();
    
    for (int i = 0; i < 10; ++i) {
        EXPECT_TRUE(events[i].executed);
    }
}

TEST(ThreadPoolTest, SafeMultipleShutdowns) {
    concurrency::ThreadPool pool(2);
    pool.startPool();
    pool.shutdown();
    // Second shutdown should not crash or deadlock
    pool.shutdown();
}

TEST(ThreadPoolTest, SafeIdleShutdown) {
    concurrency::ThreadPool pool(2);
    // startPool and then immediately shutdown with no tasks
    pool.startPool();
    pool.shutdown();
}

TEST(ThreadPoolTest, ShutdownWithoutStart) {
    concurrency::ThreadPool pool(2);
    // shutting down a pool that was never started
    pool.shutdown();
}
