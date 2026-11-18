#include "../../include/metrics/Metrics.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <algorithm>

namespace simulator {
namespace mesh {
namespace metrics {

namespace {

const char* EVENT_TYPE_NAMES[MAX_EVENT_TYPES] = {
    "DispatchEvent",
    "FaultEvent",
    "CommLossEvent",
    "CommRestoreEvent",
    "BatteryDrainEvent",
    "NoFlyZoneEvent",
    "ZoneClearEvent",
    "WaypointArrivalEvent",
    "MissionCompleteEvent",
    "HeartbeatTimeoutEvent",
    "ReplanEvent",
    "FaultRecoveryEvent",
    "Other1",
    "Other2",
    "Other3",
    "Unknown"
};

} // anonymous namespace

Metrics::Metrics(int workers) : workerCount(workers > 0 ? workers : 1) {
}

void Metrics::reset(int workers) {
    workerCount = (workers > 0 ? workers : 1);
    std::lock_guard<std::mutex> lock(metricsMutex);
    queueLatencies.clear();
    for (size_t i = 0; i < MAX_EVENT_TYPES; ++i) {
        durationsPerType[i].clear();
        eventsPerType[i] = 0;
    }
    eventsExecutedTotal.store(0);
    preemptionsCount.store(0);
    reallocationsCount.store(0);
    replansCount.store(0);
    replanFailuresCount.store(0);
    commLossFaultsCount.store(0);
    batteryFaultsCount.store(0);
    missionsCompletedCount.store(0);
    missionsPendingCount.store(0);
    missionsAbortedCount.store(0);
}

void Metrics::recordStart() {
    startTime = std::chrono::steady_clock::now();
}

void Metrics::recordEnd() {
    endTime = std::chrono::steady_clock::now();
}

Percentiles Metrics::calculatePercentiles(std::vector<double> samples) {
    Percentiles p;
    if (samples.empty()) return p;

    std::sort(samples.begin(), samples.end());
    size_t n = samples.size();
    p.p50 = samples[n / 2];
    p.p95 = samples[std::min(n - 1, static_cast<size_t>(std::ceil(n * 0.95)) - 1)];
    p.p99 = samples[std::min(n - 1, static_cast<size_t>(std::ceil(n * 0.99)) - 1)];
    return p;
}

int Metrics::getEventTypeIndex(const char* eventType) {
    if (!eventType) return MAX_EVENT_TYPES - 1;
    for (size_t i = 0; i < MAX_EVENT_TYPES - 1; ++i) {
        if (std::strcmp(eventType, EVENT_TYPE_NAMES[i]) == 0) {
            return static_cast<int>(i);
        }
    }
    return MAX_EVENT_TYPES - 1;
}

const char* Metrics::getEventTypeName(int index) {
    if (index >= 0 && index < static_cast<int>(MAX_EVENT_TYPES)) {
        return EVENT_TYPE_NAMES[index];
    }
    return "Unknown";
}

void Metrics::recordEventExecution(int /*workerId*/, const char* eventType, uint64_t queueLatencyUs, uint64_t durationUs) {
    int typeIdx = getEventTypeIndex(eventType);
    {
        std::lock_guard<std::mutex> lock(metricsMutex);
        queueLatencies.push_back(static_cast<double>(queueLatencyUs));
        durationsPerType[typeIdx].push_back(static_cast<double>(durationUs));
        eventsPerType[typeIdx]++;
    }
    eventsExecutedTotal.fetch_add(1, std::memory_order_relaxed);
}

namespace {

struct MetricsSnapshot {
    double wallTimeSec{0.001};
    uint64_t totalEvents{0};
    double eventsPerSec{0.0};
    Percentiles queueLatency;
    Percentiles durPerType[MAX_EVENT_TYPES];
    uint64_t countPerType[MAX_EVENT_TYPES]{0};
};

} // anonymous namespace

static MetricsSnapshot computeSnapshot(const std::chrono::steady_clock::time_point& startTime,
                                       const std::chrono::steady_clock::time_point& endTime,
                                       uint64_t totalEvents,
                                       std::mutex& mtx,
                                       const std::vector<double>& queueLatencies,
                                       const std::vector<double> durationsPerType[MAX_EVENT_TYPES],
                                       const uint64_t eventsPerType[MAX_EVENT_TYPES],
                                       Percentiles (*calcFn)(std::vector<double>)) {
    MetricsSnapshot s;
    s.wallTimeSec = std::chrono::duration<double>(endTime - startTime).count();
    if (s.wallTimeSec <= 0.0) s.wallTimeSec = 0.001;
    s.totalEvents = totalEvents;
    s.eventsPerSec = static_cast<double>(totalEvents) / s.wallTimeSec;

    std::vector<double> latCopy;
    std::vector<double> durCopy[MAX_EVENT_TYPES];
    {
        std::lock_guard<std::mutex> lock(mtx);
        latCopy = queueLatencies;
        for (size_t t = 0; t < MAX_EVENT_TYPES; ++t) {
            durCopy[t] = durationsPerType[t];
            s.countPerType[t] = eventsPerType[t];
        }
    }
    s.queueLatency = calcFn(std::move(latCopy));
    for (size_t t = 0; t < MAX_EVENT_TYPES; ++t) {
        if (s.countPerType[t] > 0) {
            s.durPerType[t] = calcFn(std::move(durCopy[t]));
        }
    }
    return s;
}

void Metrics::printSummaryTable() const {
    MetricsSnapshot s = computeSnapshot(
        startTime, endTime, eventsExecutedTotal.load(std::memory_order_relaxed),
        metricsMutex, queueLatencies, durationsPerType, eventsPerType, &Metrics::calculatePercentiles);

    std::cout << "\n===============================================================================\n";
    std::cout << "                         SIMULATION METRICS SUMMARY                            \n";
    std::cout << "===============================================================================\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Wall Clock Time:         " << s.wallTimeSec << " s\n";
    std::cout << "Total Events Executed:   " << s.totalEvents << "\n";
    std::cout << "Throughput:              " << s.eventsPerSec << " events/sec\n";
    std::cout << "Queue Latency (us):      p50=" << s.queueLatency.p50 << " | p95=" << s.queueLatency.p95 << " | p99=" << s.queueLatency.p99 << "\n";
    std::cout << "-------------------------------------------------------------------------------\n";
    std::cout << "Fleet & Mission Counters:\n";
    std::cout << "  Missions Completed:    " << missionsCompletedCount.load() << "\n";
    std::cout << "  Missions Pending:      " << missionsPendingCount.load() << "\n";
    std::cout << "  Missions Aborted:      " << missionsAbortedCount.load() << "\n";
    std::cout << "  Preemptions:           " << preemptionsCount.load() << "\n";
    std::cout << "  Reallocations:         " << reallocationsCount.load() << "\n";
    std::cout << "  Replans (Attempted):   " << replansCount.load() << "\n";
    std::cout << "  Replan Failures:       " << replanFailuresCount.load() << "\n";
    std::cout << "  Comm-Loss Faults:      " << commLossFaultsCount.load() << "\n";
    std::cout << "  Battery Faults:        " << batteryFaultsCount.load() << "\n";
    std::cout << "-------------------------------------------------------------------------------\n";
    std::cout << std::left << std::setw(25) << "Event Type"
              << std::right << std::setw(10) << "Count"
              << std::setw(14) << "p50 dur (us)"
              << std::setw(14) << "p95 dur (us)"
              << std::setw(14) << "p99 dur (us)\n";
    std::cout << "-------------------------------------------------------------------------------\n";

    for (size_t t = 0; t < MAX_EVENT_TYPES; ++t) {
        if (s.countPerType[t] == 0) continue;
        std::cout << std::left << std::setw(25) << EVENT_TYPE_NAMES[t]
                  << std::right << std::setw(10) << s.countPerType[t]
                  << std::setw(14) << s.durPerType[t].p50
                  << std::setw(14) << s.durPerType[t].p95
                  << std::setw(14) << s.durPerType[t].p99 << "\n";
    }
    std::cout << "===============================================================================\n\n";
}

void Metrics::writeMetricsJson(const std::string& filePath) const {
    MetricsSnapshot s = computeSnapshot(
        startTime, endTime, eventsExecutedTotal.load(std::memory_order_relaxed),
        metricsMutex, queueLatencies, durationsPerType, eventsPerType, &Metrics::calculatePercentiles);

    std::ofstream ofs(filePath);
    if (!ofs.is_open()) return;

    ofs << "{\n";
    ofs << "  \"wall_time_seconds\": " << s.wallTimeSec << ",\n";
    ofs << "  \"events_executed_total\": " << s.totalEvents << ",\n";
    ofs << "  \"events_per_second\": " << s.eventsPerSec << ",\n";
    ofs << "  \"queue_latency_us\": {\n";
    ofs << "    \"p50\": " << s.queueLatency.p50 << ",\n";
    ofs << "    \"p95\": " << s.queueLatency.p95 << ",\n";
    ofs << "    \"p99\": " << s.queueLatency.p99 << "\n";
    ofs << "  },\n";
    ofs << "  \"counters\": {\n";
    ofs << "    \"preemptions\": " << preemptionsCount.load() << ",\n";
    ofs << "    \"reallocations\": " << reallocationsCount.load() << ",\n";
    ofs << "    \"replans\": " << replansCount.load() << ",\n";
    ofs << "    \"replan_failures\": " << replanFailuresCount.load() << ",\n";
    ofs << "    \"comm_loss_faults\": " << commLossFaultsCount.load() << ",\n";
    ofs << "    \"battery_faults\": " << batteryFaultsCount.load() << ",\n";
    ofs << "    \"missions_completed\": " << missionsCompletedCount.load() << ",\n";
    ofs << "    \"missions_pending\": " << missionsPendingCount.load() << ",\n";
    ofs << "    \"missions_aborted\": " << missionsAbortedCount.load() << "\n";
    ofs << "  },\n";
    ofs << "  \"event_types\": {\n";

    bool first = true;
    for (size_t t = 0; t < MAX_EVENT_TYPES; ++t) {
        if (s.countPerType[t] == 0) continue;
        if (!first) ofs << ",\n";
        first = false;
        ofs << "    \"" << EVENT_TYPE_NAMES[t] << "\": {\n";
        ofs << "      \"count\": " << s.countPerType[t] << ",\n";
        ofs << "      \"p50_duration_us\": " << s.durPerType[t].p50 << ",\n";
        ofs << "      \"p95_duration_us\": " << s.durPerType[t].p95 << ",\n";
        ofs << "      \"p99_duration_us\": " << s.durPerType[t].p99 << "\n";
        ofs << "    }";
    }
    ofs << "\n  }\n";
    ofs << "}\n";
}

} // namespace metrics
} // namespace mesh
} // namespace simulator
