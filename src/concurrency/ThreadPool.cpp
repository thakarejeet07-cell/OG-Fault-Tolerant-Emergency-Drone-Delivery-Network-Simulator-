#include "../../include/concurrency/ThreadPool.hpp"
#include "../../include/metrics/Metrics.hpp"
#include <chrono>

namespace simulator {
namespace mesh {
namespace concurrency {

ThreadPool::ThreadPool(int numThreads, metrics::Metrics* metrics)
    : numThreads(numThreads), stop(false), inFlightTasks(0), metrics(metrics) {}

ThreadPool::~ThreadPool() {
    shutdown();
}

void ThreadPool::setMetrics(metrics::Metrics* m) {
    metrics = m;
}

void ThreadPool::startPool() {
    if (stop) return; // Cannot restart a stopped pool
    
    // Only start if not already started
    if (workers.empty() && numThreads > 0) {
        for (int i = 0; i < numThreads; ++i) {
            workers.emplace_back(&ThreadPool::workerThread, this, i);
        }
    }
}

void ThreadPool::workerThread(int workerId) {
    while (true) {
        events::Event* task = nullptr;
        if (!taskQueue.pop(task)) {
            // Queue stopped and drained -> worker exits cleanly
            return;
        }
        
        if (task) {
            auto startWall = std::chrono::steady_clock::now();
            uint64_t latencyUs = 0;
            if (task->getEnqueueWallTime().time_since_epoch().count() > 0) {
                latencyUs = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        startWall - task->getEnqueueWallTime()).count());
            }

            task->execute();

            auto endWall = std::chrono::steady_clock::now();
            uint64_t durationUs = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    endWall - startWall).count());

            if (metrics) {
                metrics->recordEventExecution(workerId, task->getEventType(), latencyUs, durationUs);
            }

            if (inFlightTasks.fetch_sub(1, std::memory_order_acq_rel) == 1) {
                std::unique_lock<std::mutex> lock(idleMutex);
                idleCv.notify_all();
            }
        }
    }
}

void ThreadPool::shutdown() {
    bool expected = false;
    // Ensure shutdown is only executed once.
    if (!stop.compare_exchange_strong(expected, true)) {
        return;
    }
    
    taskQueue.stop();
    
    for (std::thread& worker : workers) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    workers.clear();
}

void ThreadPool::assignTask(events::Event* e) {
    if (stop.load(std::memory_order_relaxed)) return; // Do not accept tasks if shutting down
    
    inFlightTasks.fetch_add(1, std::memory_order_relaxed);
    taskQueue.push(e);
}

void ThreadPool::waitUntilIdle() {
    std::unique_lock<std::mutex> lock(idleMutex);
    idleCv.wait(lock, [this] {
        return inFlightTasks.load(std::memory_order_acquire) == 0;
    });
}

int ThreadPool::getInFlightTasks() const {
    return inFlightTasks.load(std::memory_order_relaxed);
}

} // namespace concurrency
} // namespace mesh
} // namespace simulator
