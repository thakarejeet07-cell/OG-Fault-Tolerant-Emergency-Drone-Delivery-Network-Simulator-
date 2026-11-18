#ifndef DRONE_MESH_THREADSAFEQUEUE_HPP
#define DRONE_MESH_THREADSAFEQUEUE_HPP

#include <queue>
#include <mutex>
#include <condition_variable>
#include <cstddef>
#include <utility>

namespace simulator {
namespace mesh {
namespace util {

/**
 * @brief ThreadSafeQueue is a generic thread-safe FIFO queue template.
 *
 * Implements synchronization using std::mutex and std::condition_variable.
 * Provides both blocking pop() and non-blocking try_pop().
 */
template <typename T>
class ThreadSafeQueue {
private:
    std::queue<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool stopped_{false};

public:
    ThreadSafeQueue() = default;

    ~ThreadSafeQueue() {
        stop();
    }

    ThreadSafeQueue(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

    /**
     * @brief Pushes an item to the back of the queue and notifies one waiting thread.
     */
    void push(T value) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopped_) return;
            queue_.push(std::move(value));
        }
        cv_.notify_one();
    }

    /**
     * @brief Blocking pop: waits until an item is available or the queue is stopped.
     * @param value Output parameter populated with the popped item.
     * @return true if an item was successfully popped; false if stopped and empty.
     */
    bool pop(T& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] {
            return stopped_ || !queue_.empty();
        });

        if (queue_.empty()) {
            return false;
        }

        value = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    /**
     * @brief Non-blocking try_pop: attempts to pop an item immediately.
     * @param value Output parameter populated with the popped item if available.
     * @return true if an item was popped; false if the queue was empty.
     */
    bool try_pop(T& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        value = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    /**
     * @brief Unblocks all waiting threads and prevents subsequent pushes.
     */
    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopped_ = true;
        }
        cv_.notify_all();
    }

    bool is_stopped() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stopped_;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
};

} // namespace util
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_THREADSAFEQUEUE_HPP
