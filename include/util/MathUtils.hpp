#ifndef DRONE_MESH_MATHUTILS_HPP
#define DRONE_MESH_MATHUTILS_HPP

#include <type_traits>
#include <cmath>

namespace simulator {
namespace mesh {
namespace util {

/**
 * @brief Function template to clamp an arithmetic value to [lo, hi].
 * Handles NaN for floating point types by safely clamping to lo.
 */
template <typename T>
constexpr T clampTo(T value, T lo, T hi) {
    static_assert(std::is_arithmetic_v<T>, "clampTo requires an arithmetic type");
    if constexpr (std::is_floating_point_v<T>) {
        if (std::isnan(value)) {
            return lo;
        }
    }
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

} // namespace util
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_MATHUTILS_HPP
