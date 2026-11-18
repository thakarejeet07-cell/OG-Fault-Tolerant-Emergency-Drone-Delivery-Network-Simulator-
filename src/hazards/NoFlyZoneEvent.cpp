#include "../../include/hazards/NoFlyZoneEvent.hpp"
#include "../../include/domain/Grid.hpp"
#include "../../include/domain/Dispatcher.hpp"
#include "../../include/engine/Simulation.hpp"
#include "../../include/concurrency/Logger.hpp"
#include <cmath>

namespace simulator {
namespace mesh {
namespace hazards {

namespace {

void applyNoFlyZone(domain::Grid* grid, const domain::Coordinate& center, double radiusMeters, bool add) {
    auto [centerX, centerY] = grid->coordinateToCell(center);
    double cellRadius = radiusMeters / grid->getCellSizeMeters();
    int r = static_cast<int>(std::ceil(cellRadius));

    for (int dx = -r; dx <= r; ++dx) {
        for (int dy = -r; dy <= r; ++dy) {
            if (std::sqrt(dx * dx + dy * dy) <= cellRadius) {
                if (add) {
                    grid->addNoFlyZoneCell(centerX + dx, centerY + dy);
                } else {
                    grid->removeNoFlyZoneCell(centerX + dx, centerY + dy);
                }
            }
        }
    }
}

} // anonymous namespace

// --- ZoneClearEvent ---
ZoneClearEvent::ZoneClearEvent(long long timestamp, int priority,
                               const domain::Coordinate& center,
                               double radiusMeters,
                               domain::Grid* grid,
                               domain::Dispatcher* dispatcher,
                               engine::Simulation* simulation)
    : timestamp(timestamp),
      priority(priority),
      center(center),
      radiusMeters(radiusMeters),
      grid(grid),
      dispatcher(dispatcher),
      simulation(simulation) {}

void ZoneClearEvent::execute() {
    if (!grid) return;

    util::log("[Hazards] ZoneClearEvent at t=", timestamp,
              " center=(", center.getLatitude(), ",", center.getLongitude(),
              ") radius=", radiusMeters, "m");

    applyNoFlyZone(grid, center, radiusMeters, false);

    if (dispatcher) {
        dispatcher->onGridChanged(grid->getVersion());
    }
}

// --- NoFlyZoneEvent ---
NoFlyZoneEvent::NoFlyZoneEvent(long long timestamp, int priority,
                             const domain::Coordinate& center,
                             double radiusMeters,
                             long long duration,
                             domain::Grid* grid,
                             domain::Dispatcher* dispatcher,
                             engine::Simulation* simulation)
    : timestamp(timestamp),
      priority(priority),
      center(center),
      radiusMeters(radiusMeters),
      duration(duration),
      grid(grid),
      dispatcher(dispatcher),
      simulation(simulation) {}

void NoFlyZoneEvent::execute() {
    if (!grid) return;

    util::log("[Hazards] NoFlyZoneEvent at t=", timestamp,
              " center=(", center.getLatitude(), ",", center.getLongitude(),
              ") radius=", radiusMeters, "m duration=", duration, "ms");

    applyNoFlyZone(grid, center, radiusMeters, true);

    if (dispatcher) {
        dispatcher->onGridChanged(grid->getVersion());
    }

    if (simulation) {
        simulation->scheduleSimulationEvent(std::make_unique<ZoneClearEvent>(
            timestamp + duration, priority, center, radiusMeters, grid, dispatcher, simulation));
    }
}

} // namespace hazards
} // namespace mesh
} // namespace simulator
