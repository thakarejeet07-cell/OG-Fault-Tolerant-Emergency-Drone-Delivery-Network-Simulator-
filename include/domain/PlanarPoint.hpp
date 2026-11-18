#ifndef DRONE_MESH_PLANARPOINT_HPP
#define DRONE_MESH_PLANARPOINT_HPP

#include "Coordinate.hpp"
#include <cmath>
#include <iostream>

namespace simulator {
namespace mesh {
namespace domain {

/**
 * @brief PlanarPoint represents 2D Cartesian local coordinates (x, y) in meters.
 *
 * Demonstrates Class-to-Class Casting between unrelated types:
 * - Direction 1 (Coordinate -> PlanarPoint): Converting Constructor
 *   PlanarPoint(const Coordinate& coord)
 * - Direction 2 (PlanarPoint -> Coordinate): Conversion Operator
 *   operator Coordinate() const
 *
 * PathPlanner and distance calculations use this to transform spherical/grid
 * coordinates to local metric Euclidean points and back.
 */
class PlanarPoint {
private:
    double x_{0.0};
    double y_{0.0};

public:
    constexpr PlanarPoint() : x_(0.0), y_(0.0) {}
    constexpr PlanarPoint(double x, double y) : x_(x), y_(y) {}

    // CLASS-TO-CLASS DIRECTION 1: Converting Constructor from Coordinate
    // Converts a Coordinate (latitude, longitude) into a local Cartesian PlanarPoint (x, y).
    PlanarPoint(const Coordinate& coord)
        : x_(coord.getLatitude()), y_(coord.getLongitude()) {}

    double getX() const { return x_; }
    double getY() const { return y_; }
    void setX(double x) { x_ = x; }
    void setY(double y) { y_ = y; }

    double distanceTo(const PlanarPoint& other) const {
        double dx = x_ - other.x_;
        double dy = y_ - other.y_;
        return std::sqrt(dx * dx + dy * dy);
    }

    // CLASS-TO-CLASS DIRECTION 2: Conversion Operator from PlanarPoint to Coordinate
    // Converts a local Cartesian PlanarPoint (x, y) back into a domain Coordinate.
    operator Coordinate() const {
        return Coordinate(x_, y_);
    }

    bool operator==(const PlanarPoint& other) const {
        return std::abs(x_ - other.x_) < 1e-6 && std::abs(y_ - other.y_) < 1e-6;
    }

    bool operator!=(const PlanarPoint& other) const {
        return !(*this == other);
    }

    friend std::ostream& operator<<(std::ostream& os, const PlanarPoint& pt) {
        return os << "(" << pt.x_ << ", " << pt.y_ << ")";
    }
};

} // namespace domain
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_PLANARPOINT_HPP
