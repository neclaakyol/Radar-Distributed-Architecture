#pragma once

#include "radar/sweep.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace radar {

// Mounting description for one sensor, expressed in a shared world frame
// whose origin and heading reference is the main sensor (sensor_id 0).
//
// World angle convention matches mti.hpp's polar_to_cartesian(): degrees,
// measured counter-clockwise from the world +X axis, with cartesian
// (x_mm, y_mm) sharing that same origin.
//
//   world_angle_deg = angle_sign * local_angle_deg + heading_offset_deg
//   x_mm = offset_x_mm + distance_mm * cos(world_angle_deg)
//   y_mm = offset_y_mm + distance_mm * sin(world_angle_deg)
//
// angle_sign exists because the two flank sensors are mounted mirrored
// relative to each other (left sweeps local angle increasing away from
// center one way, right sweeps the other way) -- a pure additive offset
// cannot express that on its own.
//
// PLACEHOLDER VALUES: nothing below is physically measured yet (no rig
// exists to measure). These encode only the qualitative layout described
// in plan.md: main sensor centered, local angle range [0,120], heading
// offset 30 deg so its wedge is symmetric about world "up" (90 deg); each
// flank sensor mounted at one boundary edge of that wedge, baseline
// distance kPlaceholderBaselineMm away, with local angle 0 aligned to the
// boundary edge (the "inward" overlap direction) and local angle
// increasing sweeping outward, away from the main wedge. Override via the
// --left-offset-x-mm / --left-heading-deg / etc. CLI flags in main.cpp
// once the rig is physically built and measured -- see plan.md's deferred
// EEPROM calibration step.
struct SensorMount {
  std::uint8_t sensor_id = 0;
  double offset_x_mm = 0.0;
  double offset_y_mm = 0.0;
  double heading_offset_deg = 0.0;
  double angle_sign = 1.0;
};

constexpr double kPlaceholderBaselineMm = 150.0;
constexpr double kMainHeadingOffsetDeg = 30.0;

// Left flank mount: world angle 150 deg (main's local-120 boundary edge),
// baseline kPlaceholderBaselineMm from the main sensor.
// offset_x_mm/offset_y_mm = baseline * (cos(150deg), sin(150deg))
inline const SensorMount kDefaultMainMount{0, 0.0, 0.0, kMainHeadingOffsetDeg, 1.0};
inline const SensorMount kDefaultLeftMount{1, -129.9038, 75.0, 150.0, 1.0};
// Right flank mount: world angle 30 deg (main's local-0 boundary edge),
// mirrored sweep direction (angle_sign = -1) so local angle 0 also sits at
// the inward/boundary edge with local angle increasing sweeping outward.
inline const SensorMount kDefaultRightMount{2, 129.9038, 75.0, 30.0, -1.0};

struct SensorGeometry {
  std::array<SensorMount, kNumSensors> mounts{kDefaultMainMount, kDefaultLeftMount,
                                              kDefaultRightMount};

  const SensorMount &mount_for(std::uint8_t sensor_id) const;
};

// A RadarPoint translated into the shared world frame. Retains the
// original local-frame point (sensor_id, confirmation, timestamp, etc.)
// alongside the derived world-frame polar/cartesian coordinates.
struct WorldPoint {
  RadarPoint source;
  double world_angle_deg = 0.0; // true bearing from the main sensor's origin
  double world_x_mm = 0.0;
  double world_y_mm = 0.0;
};

WorldPoint to_world(const RadarPoint &point, const SensorGeometry &geometry);
std::vector<WorldPoint> to_world(const Sweep &sweep, const SensorGeometry &geometry);

} // namespace radar
