#ifndef DRONE_MESH_BATTERYDRAINEVENT_HPP
#define DRONE_MESH_BATTERYDRAINEVENT_HPP

#include "../events/Event.hpp"
#include <string>

namespace simulator {
namespace mesh {

namespace domain {
class Dispatcher;
}

namespace engine {
class Simulation;
}

namespace hazards {

class BatteryDrainEvent : public events::Event {
private:
    long long timestamp;
    int priority;
    std::string droneId;
    double amount;
    domain::Dispatcher* dispatcher;
    engine::Simulation* simulation;

public:
    static constexpr double BATTERY_CRITICAL_THRESHOLD = 15.0;

    BatteryDrainEvent(long long timestamp, int priority, const std::string& droneId,
                      double amount, domain::Dispatcher* dispatcher,
                      engine::Simulation* simulation);

    void execute() override;
    long long getTimestamp() const override { return timestamp; }
    int getPriority() const override { return priority; }
    const char* getEventType() const override { return "BatteryDrainEvent"; }

    const std::string& getDroneId() const { return droneId; }
    double getAmount() const { return amount; }
};

} // namespace hazards
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_BATTERYDRAINEVENT_HPP
