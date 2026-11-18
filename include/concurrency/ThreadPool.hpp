#ifndef DRONE_MESH_THREADPOOL_HPP
#define DRONE_MESH_THREADPOOL_HPP

#include "../events/Event.hpp"
#include "../util/ThreadSafeQueue.hpp"
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace simulator {
namespace mesh {

namespace metrics {
class Metrics;
}

namespace concurrency {

class ThreadPool {
private:
    int numThreads;
    std::vector<std::thread> workers;
    
    util::ThreadSafeQueue<events::Event*> taskQueue;
    
    std::atomic<bool> stop;

    // Drain barrier tracking: inFlightTasks tracks queued + executing tasks.
    // idleCv is notified without busy-waiting when inFlightTasks drops to zero.
    std::atomic<int> inFlightTasks{0};
    std::mutex idleMutex;
    std::condition_variable idleCv;

    metrics::Metrics* metrics{nullptr};

    void workerThread(int workerId);

public:
    explicit ThreadPool(int numThreads, metrics::Metrics* metrics = nullptr);
    ~ThreadPool();
    
    // Prevent copy/move
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;
    
    void setMetrics(metrics::Metrics* m);

    void startPool();
    void shutdown();
    void assignTask(events::Event* e);

    // Drain barrier: blocks until all in-flight tasks have finished executing.
    // Uses condition_variable waiting on inFlightTasks == 0 (no busy-waiting).
    void waitUntilIdle();

    int getInFlightTasks() const;
};

} // namespace concurrency
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_THREADPOOL_HPP
