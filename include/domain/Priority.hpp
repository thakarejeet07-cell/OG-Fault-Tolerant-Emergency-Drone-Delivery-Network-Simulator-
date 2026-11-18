#ifndef DRONE_MESH_PRIORITY_HPP
#define DRONE_MESH_PRIORITY_HPP

#include <iostream>

namespace simulator {
namespace mesh {
namespace domain {

/**
 * @brief Priority represents an urgency ranking for missions and events.
 * Higher integer values denote higher urgency.
 *
 * Demonstrates:
 * - Basic-to-Class Casting: EXPLICIT converting constructor Priority(int val).
 * - Class-to-Basic Casting: EXPLICIT conversion operator explicit operator int() const.
 * - Named constants: Priority::CRITICAL, Priority::HIGH, Priority::NORMAL, Priority::LOW.
 *
 * Design Rationale (Implicit vs. Explicit):
 * The converting constructor is EXPLICIT because an integer in C++ can represent
 * virtually anything: an array index, an error code, a drone ID count, or a timeout duration.
 * Allowing implicit int -> Priority would permit erroneous code such as:
 *   void scheduleMission(const Priority& p);
 *   scheduleMission(drone.getCurrentWaypointIndex()); // COMPILES SILENTLY IF IMPLICIT!
 * Here, passing a waypoint index (e.g. 3) where a Priority was expected would silently
 * misclassify a delivery as NORMAL priority. Making the constructor EXPLICIT forces
 * intentional construction: scheduleMission(Priority(3)) or scheduleMission(Priority::NORMAL).
 *
 * Similarly, operator int() is EXPLICIT to prevent accidental decay into numeric types
 * in expressions (e.g., missionPriority + batteryLevel) without explicit static_cast<int>(p).
 */
class Priority {
public:
    static constexpr int LOW_VAL = 1;
    static constexpr int NORMAL_VAL = 3;
    static constexpr int HIGH_VAL = 7;
    static constexpr int CRITICAL_VAL = 10;

    static const Priority LOW;
    static const Priority NORMAL;
    static const Priority HIGH;
    static const Priority CRITICAL;

private:
    int value_{NORMAL_VAL};

public:
    constexpr Priority() : value_(NORMAL_VAL) {}

    // EXPLICIT Converting Constructor:
    // Prevents arbitrary integers (e.g. indices or error codes) from silently converting to Priority.
    explicit constexpr Priority(int val) : value_(val) {}

    constexpr int getValue() const { return value_; }

    // EXPLICIT Conversion Operator to int:
    // Prevents accidental numeric decay, requiring static_cast<int>(priority) when integer is needed.
    explicit constexpr operator int() const { return value_; }

    constexpr bool operator==(const Priority& o) const { return value_ == o.value_; }
    constexpr bool operator!=(const Priority& o) const { return value_ != o.value_; }
    constexpr bool operator<(const Priority& o) const { return value_ < o.value_; }
    constexpr bool operator>(const Priority& o) const { return value_ > o.value_; }
    constexpr bool operator<=(const Priority& o) const { return value_ <= o.value_; }
    constexpr bool operator>=(const Priority& o) const { return value_ >= o.value_; }

    friend std::ostream& operator<<(std::ostream& os, const Priority& p) {
        switch (p.value_) {
            case CRITICAL_VAL: return os << "CRITICAL(" << p.value_ << ")";
            case HIGH_VAL:     return os << "HIGH(" << p.value_ << ")";
            case NORMAL_VAL:   return os << "NORMAL(" << p.value_ << ")";
            case LOW_VAL:      return os << "LOW(" << p.value_ << ")";
            default:           return os << "PRIORITY(" << p.value_ << ")";
        }
    }
};

inline constexpr Priority Priority::LOW{Priority::LOW_VAL};
inline constexpr Priority Priority::NORMAL{Priority::NORMAL_VAL};
inline constexpr Priority Priority::HIGH{Priority::HIGH_VAL};
inline constexpr Priority Priority::CRITICAL{Priority::CRITICAL_VAL};

} // namespace domain
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_PRIORITY_HPP
