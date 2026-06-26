#include "radar/geometry.hpp"

#include <cmath>
#include <stdexcept>

namespace radar {
namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;
}

const SensorMount &SensorGeometry::mount_for(std::uint8_t sensor_id) const {
  for (const SensorMount &mount : mounts) {
    if (mount.sensor_id == sensor_id) {
      return mount;
    }
  }
  throw std::out_of_range("no SensorMount for sensor_id");
}

WorldPoint to_world(const RadarPoint &point, const SensorGeometry &geometry) {
  const SensorMount &mount = geometry.mount_for(point.sensor_id);

  const double world_angle_deg =
      mount.angle_sign * static_cast<double>(point.angle_deg) + mount.heading_offset_deg;
  const double world_angle_rad = world_angle_deg * kPi / 180.0;

  WorldPoint world;
  world.source = point;
  world.world_x_mm =
      mount.offset_x_mm + static_cast<double>(point.distance_mm) * std::cos(world_angle_rad);
  world.world_y_mm =
      mount.offset_y_mm + static_cast<double>(point.distance_mm) * std::sin(world_angle_rad);
  world.world_angle_deg = std::atan2(world.world_y_mm, world.world_x_mm) * 180.0 / kPi;
  return world;
}

std::vector<WorldPoint> to_world(const Sweep &sweep, const SensorGeometry &geometry) {
  std::vector<WorldPoint> out;
  out.reserve(sweep.points.size());
  for (const RadarPoint &point : sweep.points) {
    if (point.distance_mm != 0) {
      out.push_back(to_world(point, geometry));
    }
  }
  return out;
}

} // namespace radar
