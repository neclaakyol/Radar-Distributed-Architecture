#include "radar/corroboration.hpp"

#include <cmath>
#include <utility>

namespace radar {

Corroborator::Corroborator(SensorGeometry geometry, CorroborationConfig config)
    : geometry_(std::move(geometry)), config_(config) {}

Corroborator::BandMatch Corroborator::overlap_band(const RadarPoint &point) const {
  BandMatch result;
  switch (point.sensor_id) {
  case 0: // main: local range [0,120]
    if (static_cast<double>(point.angle_deg) <= config_.main_boundary_band_deg) {
      result.in_band = true;
      result.paired_sensor_id = 2; // local angle near 0 -> right flank's boundary
    } else if (static_cast<double>(point.angle_deg) >=
               120.0 - config_.main_boundary_band_deg) {
      result.in_band = true;
      result.paired_sensor_id = 1; // local angle near 120 -> left flank's boundary
    }
    break;
  case 1: // left flank
  case 2: // right flank
    if (static_cast<double>(point.angle_deg) <= config_.flank_inward_band_deg) {
      result.in_band = true;
      result.paired_sensor_id = 0; // local angle near 0 -> main's matching boundary
    }
    break;
  default:
    break;
  }
  return result;
}

bool Corroborator::find_corroborator(const WorldPoint &candidate,
                                     std::uint8_t paired_sensor_id) const {
  if (paired_sensor_id >= kNumSensors) {
    return false;
  }

  for (const WorldPoint &entry : recent_by_sensor_[paired_sensor_id]) {
    const auto delta = candidate.source.timestamp >= entry.source.timestamp
                            ? candidate.source.timestamp - entry.source.timestamp
                            : entry.source.timestamp - candidate.source.timestamp;
    if (delta > config_.match_time_window) {
      continue;
    }

    const double dx = candidate.world_x_mm - entry.world_x_mm;
    const double dy = candidate.world_y_mm - entry.world_y_mm;
    if (std::sqrt(dx * dx + dy * dy) <= config_.match_distance_tolerance_mm) {
      return true;
    }
  }
  return false;
}

void Corroborator::prune(std::deque<WorldPoint> &buffer, RadarClock::time_point now) const {
  while (!buffer.empty()) {
    const auto front_timestamp = buffer.front().source.timestamp;
    const auto age = now >= front_timestamp ? now - front_timestamp
                                            : RadarClock::duration::zero();
    if (age > config_.match_time_window) {
      buffer.pop_front();
    } else {
      break;
    }
  }
}

WorldPoint Corroborator::update(const RadarPoint &point) {
  const double max_range_mm = geometry_.mount_for(point.sensor_id).max_range_mm;
  if (point.distance_mm == 0 ||
      static_cast<double>(point.distance_mm) > max_range_mm) {
    // Bad reading (timeout, or beyond this sensor's max range): leave
    // confirmation untouched (Unchecked) and never let it become a
    // corroboration target for another sensor.
    return to_world(point, geometry_);
  }

  WorldPoint world = to_world(point, geometry_);
  const BandMatch band = overlap_band(point);

  if (band.in_band) {
    const bool corroborated = find_corroborator(world, band.paired_sensor_id);
    world.source.confirmation =
        corroborated ? Confirmation::Confirmed : Confirmation::Rejected;
  }

  if (point.sensor_id < kNumSensors) {
    std::deque<WorldPoint> &buffer = recent_by_sensor_[point.sensor_id];
    prune(buffer, point.timestamp);
    buffer.push_back(world);
  }

  return world;
}

std::vector<WorldPoint> Corroborator::update(const Sweep &completed_sweep) {
  std::vector<WorldPoint> out;
  out.reserve(completed_sweep.points.size());
  for (const RadarPoint &point : completed_sweep.points) {
    out.push_back(update(point));
  }
  return out;
}

void Corroborator::reset() {
  for (std::deque<WorldPoint> &buffer : recent_by_sensor_) {
    buffer.clear();
  }
}

} // namespace radar
