#ifndef DRONE_MESH_EVENTSCHEDULER_HPP
#define DRONE_MESH_EVENTSCHEDULER_HPP

#include "../events/Event.hpp"
#include <queue>
#include <vector>
#include <mutex>
#include <atomic>
#include <cstdint>

namespace simulator {
namespace mesh {
namespace engine {

// Custom comparator for priority queue
// Comparator order: earlier timestamp first, then higher priority,
// then insertion sequence number (FIFO).
struct EventComparator {
    bool operator()(const events::Event* a, const events::Event* b) const {
        if (!a || !b) return a != nullptr;
        // 1. Earlier timestamp first (min-heap on timestamp)
        if (a->getTimestamp() != b->getTimestamp()) {
            return a->getTimestamp() > b->getTimestamp();
        }
        // 2. Higher priority first (max-heap on priority: higher int = more urgent)
        if (a->getPriority() != b->getPriority()) {
            return a->getPriority() < b->getPriority();
        }
        // 3. Monotonically increasing sequence counter (FIFO: earlier sequence first)
        return a->getSequenceNumber() > b->getSequenceNumber();
    }
};

class EventScheduler {
private:
    std::priority_queue<events::Event*, std::vector<events::Event*>, EventComparator> eventQueue;
    mutable std::mutex queueMutex;
    std::atomic<uint64_t> sequenceCounter{0};

public:
    void scheduleEvent(events::Event* event);
    
    // Returns the next event from the queue, or nullptr if empty.
    events::Event* popNextEvent();

    // Peeks at the next event without removing it, or nullptr if empty.
    events::Event* peekNextEvent() const;

    bool empty() const;
    size_t size() const;
};

} // namespace engine
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_EVENTSCHEDULER_HPP
