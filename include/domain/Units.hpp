#ifndef DRONE_MESH_UNITS_HPP
#define DRONE_MESH_UNITS_HPP

#include <iostream>

namespace simulator {
namespace mesh {
namespace domain {

/**
 * @brief Meters represents a linear distance in meters.
 *
 * Demonstrates:
 * - Basic-to-Class Casting: IMPLICIT converting constructor Meters(double val).
 * - Class-to-Basic Casting: IMPLICIT conversion operator operator double() const.
 */
class Meters {
private:
    double value_{0.0};

public:
    constexpr Meters() : value_(0.0) {}

    // IMPLICIT Converting Constructor:
    constexpr Meters(double val) : value_(val) {}

    constexpr double getValue() const { return value_; }

    // IMPLICIT Conversion Operator to double:
    constexpr operator double() const { return value_; }

    friend std::ostream& operator<<(std::ostream& os, const Meters& m) {
        return os << m.value_ << "m";
    }
};

} // namespace domain
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_UNITS_HPP
