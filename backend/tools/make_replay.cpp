#include "radar/geometry.hpp"
#include "radar/protocol.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

std::uint16_t forward_distance(int angle, int cycle) {
  return static_cast<std::uint16_t>(700 + (angle % 23) + cycle * 10);
}

std::uint16_t return_distance(int angle, int cycle) {
  return static_cast<std::uint16_t>(760 + (angle % 23) + cycle * 10);
}

void write_frame(std::ofstream &output,
                 const std::array<radar::SensorReading, radar::kNumSensors> &readings) {
  const auto frame = radar::make_frame(readings);
  output.write(reinterpret_cast<const char *>(frame.data()),
               static_cast<std::streamsize>(frame.size()));
}

void write_uniform_frame(std::ofstream &output, std::uint8_t angle,
                         std::uint16_t distance) {
  // Replay generates identical readings across all three sensors for each step.
  write_frame(output, {{{0, angle, distance}, {1, angle, distance}, {2, angle, distance}}});
}

// Sweeps one sensor's angle back and forth within [min_angle, max_angle],
// mirroring the Arduino firmware's per-channel ScanChannel model -- each
// channel has its own bounds and its own forward/return state, independent
// of the other two.
struct ScanChannel {
  std::uint8_t sensor_id;
  int min_angle;
  int max_angle;
  int angle;
  int step = 1;
  bool forward = true;

  std::uint8_t advance() {
    const auto current = static_cast<std::uint8_t>(angle);
    if (forward) {
      angle += step;
      if (angle >= max_angle) {
        angle = max_angle;
        forward = false;
      }
    } else {
      angle -= step;
      if (angle <= min_angle) {
        angle = min_angle;
        forward = true;
      }
    }
    return current;
  }
};

// Solves for the distance a sensor must report, at a given local angle, to
// land on the same world-frame point as `reference`. Used to inject a
// geometrically-consistent cross-sensor match into the overlap scenario,
// computed through the same transform (radar::to_world) the backend
// applies on replay -- not a hand-picked constant.
std::uint16_t matching_distance(const radar::WorldPoint &reference,
                                std::uint8_t sensor_id, std::uint8_t local_angle_deg,
                                const radar::SensorGeometry &geometry) {
  const radar::SensorMount &mount = geometry.mount_for(sensor_id);
  const double world_angle_rad =
      (mount.angle_sign * static_cast<double>(local_angle_deg) + mount.heading_offset_deg) *
      kPi / 180.0;
  const double dx = reference.world_x_mm - mount.offset_x_mm;
  const double dy = reference.world_y_mm - mount.offset_y_mm;
  const double distance = dx * std::cos(world_angle_rad) + dy * std::sin(world_angle_rad);
  return static_cast<std::uint16_t>(std::lround(distance > 0.0 ? distance : 0.0));
}

void write_overlap_scenario(std::ofstream &output, int cycles) {
  const radar::SensorGeometry geometry;
  std::array<ScanChannel, radar::kNumSensors> channels{{
      {0, 0, 120, 0},
      {1, 0, 120, 0},
      {2, 0, 120, 0},
  }};

  // One round trip of a channel (all three share the 0-120 range and advance
  // together here) per "cycle" -- frames_per_cycle derives straight from the
  // channel range, so this stays correct regardless of the bound.
  const int frames_per_cycle = 2 * (channels[1].max_angle - channels[1].min_angle);
  const int total_frames = cycles * frames_per_cycle;

  for (int frame = 0; frame < total_frames; ++frame) {
    std::array<std::uint8_t, radar::kNumSensors> angles{};
    for (std::size_t i = 0; i < radar::kNumSensors; ++i) {
      angles[i] = channels[i].advance();
    }

    std::array<radar::SensorReading, radar::kNumSensors> readings{{
        {0, angles[0], forward_distance(angles[0], frame)},
        {1, angles[1], forward_distance(angles[1], frame)},
        {2, angles[2], forward_distance(angles[2], frame)},
    }};

    if (frame == 0) {
      // Both main (local angle 0, its boundary band) and the right flank
      // (local angle 0, its inward band) start their sweep here -- inject
      // a confirmed corroboration: the right flank reports a distance that
      // lands on the exact same world point as the main sensor.
      radar::RadarPoint main_point;
      main_point.angle_deg = angles[0];
      main_point.distance_mm = 1000;
      main_point.sensor_id = 0;
      const radar::WorldPoint main_world = radar::to_world(main_point, geometry);

      readings[0].distance_mm = main_point.distance_mm;
      readings[2].distance_mm = matching_distance(main_world, 2, angles[2], geometry);
    } else if (frame == 1) {
      // One step later, both are still within their overlap bands -- inject
      // a deliberate mismatch: the right flank reports a distance nowhere
      // near the main sensor's world point, so corroboration rejects it.
      readings[0].distance_mm = 1000;
      readings[2].distance_mm = 200;
    }

    write_frame(output, readings);
  }
}

} // namespace

int main(int argc, char **argv) {
  try {
    std::optional<std::filesystem::path> output_path;
    std::optional<int> cycles;
    std::string scenario = "uniform";

    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--scenario") {
        if (i + 1 >= argc) {
          throw std::invalid_argument("missing value for --scenario");
        }
        scenario = argv[++i];
      } else if (!output_path) {
        output_path = arg;
      } else if (!cycles) {
        cycles = std::stoi(arg);
      } else {
        throw std::invalid_argument("unexpected argument: " + arg);
      }
    }

    if (!output_path) {
      std::cerr << "Usage: " << argv[0]
                << " OUTPUT.bin [cycles] [--scenario uniform|overlap]\n";
      return 2;
    }
    if (scenario != "uniform" && scenario != "overlap") {
      throw std::invalid_argument("--scenario must be uniform or overlap");
    }

    const int cycle_count = cycles.value_or(2);
    if (cycle_count <= 0) {
      throw std::invalid_argument("cycles must be positive");
    }

    const auto parent = output_path->parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent);
    }

    std::ofstream output(*output_path, std::ios::binary);
    if (!output) {
      throw std::runtime_error("failed to open output: " + output_path->string());
    }

    if (scenario == "overlap") {
      write_overlap_scenario(output, cycle_count);
    } else {
      for (int cycle = 0; cycle < cycle_count; ++cycle) {
        for (int angle = 0; angle <= 120; ++angle) {
          write_uniform_frame(output, static_cast<std::uint8_t>(angle),
                              forward_distance(angle, cycle));
        }
        for (int angle = 119; angle >= 0; --angle) {
          write_uniform_frame(output, static_cast<std::uint8_t>(angle),
                              return_distance(angle, cycle));
        }
      }
    }

    std::cout << "Wrote replay fixture (" << scenario << "): " << output_path->string() << "\n";
    return 0;
  } catch (const std::exception &ex) {
    std::cerr << "Error: " << ex.what() << "\n";
    return 1;
  }
}
