#ifndef DRONE_MESH_METRICS_HPP
#define DRONE_MESH_METRICS_HPP

#include <atomic>
#include <vector>
#include <string>
#include <chrono>
#include <cstdint>
#include <cstddef>
#include <mutex>

namespace simulator {
namespace mesh {
namespace metrics {

static constexpr size_t MAX_EVENT_TYPES = 16;

struct Percentiles {
    double p50{0.0};
    double p95{0.0};
    double p99{0.0};
};

class Metrics {
private:
    int workerCount{4};
    mutable std::mutex metricsMutex;
    std::vector<double> queueLatencies;
    std::vector<double> durationsPerType[MAX_EVENT_TYPES];
    uint64_t eventsPerType[MAX_EVENT_TYPES]{0};

    std::chrono::steady_clock::time_point startTime{};
    std::chrono::steady_clock::time_point endTime{};

    static Percentiles calculatePercentiles(std::vector<double> samples);

public:
    // Atomic counters
    std::atomic<uint64_t> eventsExecutedTotal{0};
    std::atomic<uint64_t> preemptionsCount{0};
    std::atomic<uint64_t> reallocationsCount{0};
    std::atomic<uint64_t> replansCount{0};
    std::atomic<uint64_t> replanFailuresCount{0};
    std::atomic<uint64_t> commLossFaultsCount{0};
    std::atomic<uint64_t> batteryFaultsCount{0};
    std::atomic<uint64_t> missionsCompletedCount{0};
    std::atomic<uint64_t> missionsPendingCount{0};
    std::atomic<uint64_t> missionsAbortedCount{0};

    explicit Metrics(int workers = 4);

    void reset(int workers);
    void recordStart();
    void recordEnd();

    // Record execution latency and duration for an event
    void recordEventExecution(int workerId, const char* eventType, uint64_t queueLatencyUs, uint64_t durationUs);

    static int getEventTypeIndex(const char* eventType);
    static const char* getEventTypeName(int index);

    void printSummaryTable() const;
    void writeMetricsJson(const std::string& filePath = "metrics.json") const;
};

} // namespace metrics
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_METRICS_HPP
