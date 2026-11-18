#ifndef DRONE_MESH_TRACE_RECORDER_HPP
#define DRONE_MESH_TRACE_RECORDER_HPP

// TraceRecorder: records snapshots ("frames") of the simulation so the HTML UI
// (ui/index.html) can replay it. It only does anything when --trace is used.
//
// Thread safety: frames are captured by the simulation thread right AFTER the
// drain barrier (threadPool->waitUntilIdle()), so no worker is running and the
// state is consistent. Log lines can arrive from workers, so they are guarded
// by a mutex.

#include "../engine/Simulation.hpp"
#include "../concurrency/Logger.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <cstdint>

namespace simulator {
namespace mesh {
namespace util {

class TraceRecorder {
public:
    static TraceRecorder& instance() {
        static TraceRecorder rec;
        return rec;
    }

    // Turn recording on and start capturing log lines.
    void enable(const std::string& scenarioName, uint64_t seed) {
        enabled_ = true;
        scenario_ = scenarioName;
        seed_ = seed;
        logSink() = [this](const std::string& line) {
            std::lock_guard<std::mutex> lk(logMutex_);
            pendingLogs_.push_back(line);
        };
    }

    bool isEnabled() const { return enabled_; }

    // Called by Simulation::start() at t=0, after every timestamp, and at the end.
    void captureFrame(engine::Simulation& sim) {
        if (!enabled_) return;
        using namespace domain;

        Grid* grid = sim.getGrid();
        Dispatcher* disp = sim.getDispatcher();
        if (!grid || !disp) return;

        const int W = grid->getWidth();
        const int H = grid->getHeight();
        const double cs = grid->getCellSizeMeters();

        if (!metaSet_) {
            width_ = W; height_ = H; cell_ = cs;
            duration_ = sim.getConfig().simulationDurationMs;
            numDrones_ = static_cast<int>(disp->getFleet().size());
            metaSet_ = true;
        }

        // ---- missions: remember static info for any new mission ----
        const auto& created = sim.getCreatedMissions();
        std::unordered_map<const Mission*, int> missionIndex;
        for (size_t i = 0; i < created.size(); ++i) {
            const Mission* m = created[i].get();
            missionIndex[m] = static_cast<int>(i);
            if (i >= missionInfo_.size()) {
                std::ostringstream o;
                o << "{\"id\":\"" << esc(m->getMissionId()) << "\",\"pr\":" << m->getPriority()
                  << ",\"px\":" << num(m->getPickupLocation().getLatitude() / cs)
                  << ",\"py\":" << num(m->getPickupLocation().getLongitude() / cs)
                  << ",\"dx\":" << num(m->getDropoffLocation().getLatitude() / cs)
                  << ",\"dy\":" << num(m->getDropoffLocation().getLongitude() / cs) << "}";
                missionInfo_.push_back(o.str());
            }
        }

        // ---- body (everything except the timestamp, used for de-duplication) ----
        std::ostringstream body;

        // grid cells, only when the grid changed
        uint64_t ver = grid->getVersion();
        if (!hasGridVersion_ || ver != lastGridVersion_) {
            std::string cells;
            cells.reserve(static_cast<size_t>(W * H));
            for (int y = 0; y < H; ++y) {
                for (int x = 0; x < W; ++x) {
                    CellState cs2 = grid->getCellState(x, y);
                    cells += (cs2 == CellState::FREE) ? '0'
                           : (cs2 == CellState::STATIC_OBSTACLE) ? '1' : '2';
                }
            }
            body << "\"g\":\"" << cells << "\",";
            lastGridVersion_ = ver;
            hasGridVersion_ = true;
        }

        // drones: [x, y, state, battery, comms, missionIdx, (path if new route)]
        body << "\"d\":[";
        bool firstDrone = true;
        for (Drone* d : disp->getFleet()) {
            if (!d) continue;
            Coordinate loc = d->getCurrentLocation();
            Mission* cm = d->getCurrentMission();
            int mIdx = -1;
            if (cm) {
                auto it = missionIndex.find(cm);
                if (it != missionIndex.end()) mIdx = it->second;
            }
            if (!firstDrone) body << ",";
            firstDrone = false;
            body << "[" << num(loc.getLatitude() / cs) << "," << num(loc.getLongitude() / cs)
                 << "," << static_cast<int>(d->getState())
                 << "," << std::fixed << std::setprecision(1) << d->getBatteryLevel()
                 << "," << (d->isCommsLost() ? 1 : 0) << "," << mIdx;

            uint64_t gen = d->getRouteGeneration();
            auto git = lastRouteGen_.find(d->getDroneId());
            if (git == lastRouteGen_.end() || git->second != gen) {
                lastRouteGen_[d->getDroneId()] = gen;
                Path p = d->getActivePath();
                body << ",[";
                for (size_t k = 0; k < p.waypoints.size(); ++k) {
                    if (k) body << ",";
                    body << num(p.waypoints[k].getLatitude() / cs) << ","
                         << num(p.waypoints[k].getLongitude() / cs);
                }
                body << "]";
            }
            body << "]";
        }
        body << "],";

        // mission states: one char per mission (P/A/C/X)
        body << "\"ms\":\"";
        for (const auto& m : created) {
            switch (m->getState()) {
                case Mission::MissionState::PENDING:   body << 'P'; break;
                case Mission::MissionState::ACTIVE:    body << 'A'; break;
                case Mission::MissionState::COMPLETED: body << 'C'; break;
                case Mission::MissionState::ABORTED:   body << 'X'; break;
            }
        }
        body << "\",";

        // counters
        metrics::Metrics* mt = sim.getMetrics();
        body << "\"c\":[" << mt->eventsExecutedTotal.load()
             << "," << mt->preemptionsCount.load()
             << "," << mt->reallocationsCount.load()
             << "," << mt->replansCount.load()
             << "," << mt->replanFailuresCount.load()
             << "," << mt->commLossFaultsCount.load()
             << "," << mt->batteryFaultsCount.load()
             << "," << disp->getCompletedMissionCount()
             << "," << disp->getActiveMissionCount()
             << "," << disp->getQueuedMissionCount() << "]";

        // log lines since last frame
        std::vector<std::string> logs;
        {
            std::lock_guard<std::mutex> lk(logMutex_);
            logs.swap(pendingLogs_);
        }
        std::string bodyStr = body.str();
        if (bodyStr == lastBody_ && logs.empty() && !frames_.empty()) return; // nothing new

        lastBody_ = bodyStr;
        std::ostringstream frame;
        frame << "{\"t\":" << sim.getVirtualTime() << "," << bodyStr;
        if (!logs.empty()) {
            frame << ",\"l\":[";
            for (size_t i = 0; i < logs.size(); ++i) {
                if (i) frame << ",";
                frame << "\"" << esc(logs[i]) << "\"";
            }
            frame << "]";
        }
        frame << "}";
        frames_.push_back(frame.str());
    }

    // Writes <path> (JSON) and <path without .json>.js (same data for file:// loading)
    bool writeFiles(const std::string& path) {
        if (!enabled_) return false;
        std::ostringstream out;
        out << "{\"meta\":{\"scenario\":\"" << esc(scenario_) << "\",\"seed\":" << seed_
            << ",\"w\":" << width_ << ",\"h\":" << height_ << ",\"cell\":" << cell_
            << ",\"duration\":" << duration_ << ",\"drones\":" << numDrones_ << "},\"missions\":[";
        for (size_t i = 0; i < missionInfo_.size(); ++i) {
            if (i) out << ",";
            out << missionInfo_[i];
        }
        out << "],\"frames\":[\n";
        for (size_t i = 0; i < frames_.size(); ++i) {
            if (i) out << ",\n";
            out << frames_[i];
        }
        out << "\n]}";
        std::string json = out.str();

        std::ofstream f(path);
        if (!f) return false;
        f << json;
        f.close();

        std::string jsPath = path;
        size_t dot = jsPath.rfind(".json");
        if (dot != std::string::npos) jsPath = jsPath.substr(0, dot);
        jsPath += ".js";
        std::ofstream jf(jsPath);
        if (jf) { jf << "window.TRACE = " << json << ";\n"; }

        std::cout << "[Trace] Wrote " << frames_.size() << " frames to " << path
                  << " and " << jsPath << "\n";
        logSink() = nullptr;
        return true;
    }

private:
    TraceRecorder() = default;

    static std::string num(double v) {
        std::ostringstream o;
        o << std::fixed << std::setprecision(2) << v;
        return o.str();
    }
    static std::string esc(const std::string& s) {
        std::string r;
        for (char ch : s) {
            switch (ch) {
                case '"':  r += "\\\""; break;
                case '\\': r += "\\\\"; break;
                case '\n': r += "\\n";  break;
                case '\r': break;
                case '\t': r += "\\t";  break;
                default:
                    if (static_cast<unsigned char>(ch) < 0x20) r += ' ';
                    else r += ch;
            }
        }
        return r;
    }

    bool enabled_{false};
    bool metaSet_{false};
    std::string scenario_;
    uint64_t seed_{0};
    int width_{0}, height_{0}, numDrones_{0};
    double cell_{10.0};
    long long duration_{0};

    std::vector<std::string> frames_;
    std::vector<std::string> missionInfo_;
    std::string lastBody_;
    uint64_t lastGridVersion_{0};
    bool hasGridVersion_{false};
    std::unordered_map<std::string, uint64_t> lastRouteGen_;

    std::mutex logMutex_;
    std::vector<std::string> pendingLogs_;
};

} // namespace util
} // namespace mesh
} // namespace simulator

#endif // DRONE_MESH_TRACE_RECORDER_HPP
