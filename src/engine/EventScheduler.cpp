#include "../../include/engine/EventScheduler.hpp"

namespace simulator {
namespace mesh {
namespace engine {

void EventScheduler::scheduleEvent(events::Event* event) {
    if (!event) return;
    event->setSequenceNumber(++sequenceCounter);
    event->setEnqueueWallTime(std::chrono::steady_clock::now());
    std::lock_guard<std::mutex> lock(queueMutex);
    eventQueue.push(event);
}

events::Event* EventScheduler::popNextEvent() {
    std::lock_guard<std::mutex> lock(queueMutex);
    if (eventQueue.empty()) {
        return nullptr;
    }
    events::Event* top = eventQueue.top();
    eventQueue.pop();
    return top;
}

events::Event* EventScheduler::peekNextEvent() const {
    std::lock_guard<std::mutex> lock(queueMutex);
    if (eventQueue.empty()) {
        return nullptr;
    }
    return eventQueue.top();
}

bool EventScheduler::empty() const {
    std::lock_guard<std::mutex> lock(queueMutex);
    return eventQueue.empty();
}

size_t EventScheduler::size() const {
    std::lock_guard<std::mutex> lock(queueMutex);
    return eventQueue.size();
}

} // namespace engine
} // namespace mesh
} // namespace simulator
