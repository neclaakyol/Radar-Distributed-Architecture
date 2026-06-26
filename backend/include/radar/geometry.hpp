#pragma once

#include "radar/sweep.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace radar {

// Mounting description for one sensor, expressed in a shared world frame
// whose origin and heading reference is the main sensor (sensor_id 0, the
// HY-SRF05).
//
// World angle convention matches mti.hpp's polar_to_cartesian(): degrees,
// measured counter-clockwise from the world +X axis, with cartesian
// (x_mm, y_mm) sharing that same origin. World +Y points "into" the scene
// (the direction the main sensor looks); the main sensor sits at the origin.
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
// Physical layout (measured): the workspace is an 80 x 140 cm plane. The
// main HY-SRF05 sits at the origin and sweeps a 120 deg arc (local [0,120],
// heading offset 30 deg -> world wedge 30..150 deg, symmetric about world
// "up" at 90 deg), with an 80 cm max range. Each flank sensor is mounted on
// one boundary arm of that wedge, kFlankBaselineMm (40 cm) out from the
// origin, with local angle 0 aligned to that boundary edge -- so a flank
// local-0 reading lands on the same world ray the main sensor's matching
// boundary reading reaches (the overlap the corroborator checks). From that
// boundary edge each flank sweeps 180 deg *inward*, across the main
// sensor's workspace (angle_sign points the sweep into the scene, not out
// of it), so all three sensors look at the same space. The flanks have a
// 60 cm max range. Override any value via the --left-offset-x-mm /
// --left-heading-deg / --main-max-range-mm / etc. CLI flags in main.cpp.
struct SensorMount {
  std::uint8_t sensor_id = 0;
  double offset_x_mm = 0.0;
  double offset_y_mm = 0.0;
  double heading_offset_deg = 0.0;
  double angle_sign = 1.0;
  double max_range_mm = 800.0;   // readings beyond this are dropped (not shown on map)
  double sweep_min_deg = 0.0;    // local sweep extents (used to draw coverage on the map)
  double sweep_max_deg = 120.0;
};

// Workspace plane (top-down map extents). World x spans
// [-kPlaneWidthMm/2, +kPlaneWidthMm/2] (140 cm wide); world y spans
// [0, kPlaneDepthMm] (80 cm deep, the direction the main sensor looks).
constexpr double kPlaneWidthMm = 1400.0;
constexpr double kPlaneDepthMm = 800.0;

constexpr double kFlankBaselineMm = 400.0; // 40 cm along the wedge boundary arms
constexpr double kMainHeadingOffsetDeg = 30.0;
constexpr double kMainMaxRangeMm = 800.0;  // 80 cm
constexpr double kFlankMaxRangeMm = 600.0; // 60 cm

// Main HY-SRF05: origin, 120 deg sweep, 80 cm range.
inline const SensorMount kDefaultMainMount{0, 0.0, 0.0, kMainHeadingOffsetDeg, 1.0,
                                           kMainMaxRangeMm, 0.0, 120.0};
// Left flank: positioned on the main wedge's local-120 boundary arm (world
// 150 deg), kFlankBaselineMm out. offset = baseline * (cos150, sin150). In
// real life it is aimed at the main sensor, so its local-0 boresight points
// at the origin (world -30 deg); angle_sign = +1 sweeps the 180 deg arc from
// there (world -30 -> 150) up across the main sensor's workspace.
inline const SensorMount kDefaultLeftMount{1, -346.4101615, 200.0, -30.0, 1.0,
                                           kFlankMaxRangeMm, 0.0, 180.0};
// Right flank: mounted on the main wedge's local-0 boundary arm (world 30
// deg), mirrored. offset = baseline * (cos30, sin30). local-0 sits on that
// boundary edge; angle_sign = +1 sweeps the 180 deg arc inward (world
// 30 -> 210), across the main sensor's workspace.
inline const SensorMount kDefaultRightMount{2, 346.4101615, 200.0, 30.0, 1.0,
                                            kFlankMaxRangeMm, 0.0, 180.0};

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
