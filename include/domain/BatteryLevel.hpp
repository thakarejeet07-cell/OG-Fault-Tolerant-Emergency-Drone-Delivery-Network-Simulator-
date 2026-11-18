#ifndef DRONE_MESH_BATTERYLEVEL_HPP
#define DRONE_MESH_BATTERYLEVEL_HPP

#include "../util/MathUtils.hpp"
#include <iostream>

namespace simulator {
namespace mesh {
namespace domain {

/**
 * @brief BatteryLevel represents a bounded drone battery charge percentage [0.0%, 100.0%].
 *
 * Demonstrates:
 * - Basic-to-Class Casting: EXPLICIT converting constructor BatteryLevel(float percent).
 * - Class-to-Basic Casting: EXPLICIT conversion operator explicit operator float() const.
 * - Function template usage: util::clampTo<float>(percent, 0.0f, 100.0f).
 *
 * Design Rationale (Implicit vs. Explicit):
 * The converting constructor is EXPLICIT because silent conversion from raw float or double
 * is dangerous: arbitrary floating-point quantities (such as flight speed in m/s, coordinate
 * distances, or sensor noise) could otherwise be accidentally passed to drone battery APIs.
 *
 * Similarly, the conversion operator is EXPLICIT to prevent subtle bugs where a BatteryLevel
 * inadvertently participates in unrelated arithmetic expressions.
 * For example, if operator float() were implicit:
 *   float remainingFlightTime = distance / drone.getBattery(); // COMPILES SILENTLY IF IMPLICIT!
 * Here, dividing distance by battery level rather than speed is a catastrophic physics bug.
 * With an EXPLICIT conversion operator, the compiler rejects this code at compile time,
 * requiring static_cast<float>(drone.getBattery()) only when intentionally inspecting the value.
 */
class BatteryLevel {
private:
    float percent_{100.0f};

public:
    constexpr BatteryLevel() : percent_(100.0f) {}

    // EXPLICIT Converting Constructor:
    // Prevents accidental conversion from arbitrary floating-point numbers.
    // Clamps the given percent into [0.0f, 100.0f] via util::clampTo.
    explicit BatteryLevel(float percent)
        : percent_(util::clampTo<float>(percent, 0.0f, 100.0f)) {}

    explicit BatteryLevel(double percent)
        : percent_(util::clampTo<float>(static_cast<float>(percent), 0.0f, 100.0f)) {}

    float getPercent() const { return percent_; }

    // EXPLICIT Conversion Operator to float:
    // Requires static_cast<float>(battery) to extract the underlying scalar,
    // protecting against accidental participation in implicit arithmetic.
    explicit operator float() const { return percent_; }
    explicit operator double() const { return static_cast<double>(percent_); }

    bool operator==(const BatteryLevel& o) const { return percent_ == o.percent_; }
    bool operator!=(const BatteryLevel& o) const { return percent_ != o.percent_; }
    bool operator<(const BatteryLevel& o) const { return percent_ < o.percent_; }
    bool operator<=(const BatteryLevel& o) const { return percent_ <= o.percent_; }
    bool operator>(const BatteryLevel& o) const { return percent_ > o.percent_; }
    bool operator>=(const BatteryLevel& o) const { return percent_ >= o.percent_; }

    friend std::ostream& operator<<(std::ostream& os, const BatteryLevel& b) {
        return os << b.percent_ << "%";
    }
};

} // namespace domain
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_BATTERYLEVEL_HPP
