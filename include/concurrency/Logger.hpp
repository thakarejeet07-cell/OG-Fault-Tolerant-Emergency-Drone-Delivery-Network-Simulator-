#ifndef DRONE_MESH_LOGGER_HPP
#define DRONE_MESH_LOGGER_HPP

#include <iostream>
#include <mutex>
#include <utility>
#include <functional>
#include <sstream>
#include <string>

namespace simulator {
namespace mesh {
namespace util {

inline std::mutex& getLogMutex() {
    static std::mutex logMutex;
    return logMutex;
}

// Optional hook used by the UI trace recorder. Empty (does nothing) by default.
inline std::function<void(const std::string&)>& logSink() {
    static std::function<void(const std::string&)> sink;
    return sink;
}

template <typename... Args>
inline void log(Args&&... args) {
    std::lock_guard<std::mutex> lock(getLogMutex());
    if (logSink()) {
        std::ostringstream oss;
        (oss << ... << std::forward<Args>(args));
        std::cout << oss.str() << std::endl;
        logSink()(oss.str());
    } else {
        (std::cout << ... << std::forward<Args>(args)) << std::endl;
    }
}

} // namespace util
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_LOGGER_HPP
