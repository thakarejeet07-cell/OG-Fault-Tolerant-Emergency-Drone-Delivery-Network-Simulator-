#ifndef DRONE_MESH_EVENT_HPP
#define DRONE_MESH_EVENT_HPP

#include <chrono>
#include <cstdint>
#include "../domain/SimTime.hpp"
#include "../domain/Priority.hpp"

namespace simulator {
namespace mesh {
namespace events {

class Event {
protected:
    uint64_t sequenceNumber{0};
    std::chrono::steady_clock::time_point enqueueWallTime{};

public:
    virtual ~Event() = default;

    virtual void execute() = 0;
    virtual long long getTimestamp() const = 0;
    virtual int getPriority() const = 0;
    virtual const char* getEventType() const { return "Event"; }

    // Domain strong value accessors
    domain::SimTime getSimTime() const { return domain::SimTime(getTimestamp()); }
    domain::Priority getDomainPriority() const { return domain::Priority(getPriority()); }

    uint64_t getSequenceNumber() const { return sequenceNumber; }
    void setSequenceNumber(uint64_t seq) { sequenceNumber = seq; }

    std::chrono::steady_clock::time_point getEnqueueWallTime() const { return enqueueWallTime; }
    void setEnqueueWallTime(std::chrono::steady_clock::time_point tp) { enqueueWallTime = tp; }
};

} // namespace events
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_EVENT_HPP

