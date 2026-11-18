#ifndef DRONE_MESH_SIMTIME_HPP
#define DRONE_MESH_SIMTIME_HPP

#include <iostream>

namespace simulator {
namespace mesh {
namespace domain {

/**
 * @brief SimTime represents a discrete virtual-clock timestamp in milliseconds.
 *
 * Demonstrates:
 * - Basic-to-Class Casting: Implicit converting constructor SimTime(long ms).
 * - Class-to-Basic Casting: Implicit conversion operator operator long() const.
 *
 * Design Rationale (Implicit vs. Explicit):
 * In a discrete-event simulation engine, timestamps are purely linear monotonic time
 * offsets in milliseconds. Making the converting constructor and conversion operator
 * IMPLICIT is safe and natural because:
 *   1. Integer literals and arithmetic results (e.g., t + 500) seamlessly flow into
 *      event scheduling APIs without verbose constructor noise.
 *   2. SimTime can be compared directly (<, ==, >) against scalar clock values and
 *      passed directly into arithmetic expressions without needing explicit casts.
 * Unlike unitless identifiers or unbounded quantities, treating time offsets as
 * interoperable with long scalar time preserves ergonomic mathematical syntax.
 */
class SimTime {
private:
    long ms_{0};

public:
    constexpr SimTime() : ms_(0) {}

    // IMPLICIT Converting Constructor:
    // Safe and natural for virtual clock timestamps in milliseconds.
    constexpr SimTime(long ms) : ms_(ms) {}
    constexpr SimTime(long long ms) : ms_(static_cast<long>(ms)) {}
    constexpr SimTime(int ms) : ms_(static_cast<long>(ms)) {}

    constexpr long getMs() const { return ms_; }

    // IMPLICIT Conversion Operator to long:
    // Enables direct use in virtual-clock arithmetic and comparisons.
    constexpr operator long() const { return ms_; }

    friend std::ostream& operator<<(std::ostream& os, const SimTime& t) {
        return os << t.ms_ << "ms";
    }
};

} // namespace domain
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_SIMTIME_HPP
