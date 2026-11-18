#include "../../include/domain/Coordinate.hpp"
#include <cmath>

namespace simulator {
namespace mesh {
namespace domain {

Coordinate::Coordinate()
    : latitude(0.0), longitude(0.0) {
}

Coordinate::Coordinate(double latitude, double longitude)
    : latitude(latitude), longitude(longitude) {
}

double Coordinate::getLatitude() const {
    return latitude;
}

double Coordinate::getLongitude() const {
    return longitude;
}

void Coordinate::setLatitude(double lat) {
    latitude = lat;
}

void Coordinate::setLongitude(double lon) {
    longitude = lon;
}

double Coordinate::distanceTo(const Coordinate& other) const {
    return distanceTo(other.latitude, other.longitude);
}

double Coordinate::distanceTo(double targetLat, double targetLon) const {
    return std::hypot(latitude - targetLat, longitude - targetLon);
}

Coordinate Coordinate::operator+(const Coordinate& other) const {
    return Coordinate(
        latitude + other.latitude,
        longitude + other.longitude
    );
}

bool Coordinate::operator==(const Coordinate& other) const {
    return latitude == other.latitude &&
           longitude == other.longitude;
}
Coordinate midpoint(
    const Coordinate& a,
    const Coordinate& b
) {
    return Coordinate(
        (a.latitude + b.latitude) / 2.0,
        (a.longitude + b.longitude) / 2.0
    );
}

// STREAM OPERATOR OVERLOADING (friend functions)
// operator<< : lets any Coordinate be written straight to std::cout,
// a log stream, etc. -- used by Drone's telemetry logging.
std::ostream& operator<<(std::ostream& os, const Coordinate& coord) {
    os << "(" << coord.latitude << ", " << coord.longitude << ")";
    return os;
}

// operator>> : lets a Coordinate be read straight from std::cin or a file,
// e.g. when the Simulation Engineer loads a grid/waypoint file.
std::istream& operator>>(std::istream& is, Coordinate& coord) {
    is >> coord.latitude >> coord.longitude;
    return is;
}

} // namespace domain
} // namespace mesh
} // namespace simulator
