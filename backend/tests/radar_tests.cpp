#include "radar/corroboration.hpp"
#include "radar/geometry.hpp"
#include "radar/io.hpp"
#include "radar/mti.hpp"
#include "radar/protocol.hpp"
#include "radar/raster.hpp"
#include "radar/sweep.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <utility>
#include <vector>

namespace {

using radar::RadarClock;

void test_protocol_round_trip() {
  const std::array<radar::SensorReading, radar::kNumSensors> readings = {{
      {0, 90, 400},
      {1, 45, 200},
      {2, 30, 300},
  }};
  const auto frame = radar::make_frame(readings);
  assert(frame[0] == radar::kSyncByte1);
  assert(frame[1] == radar::kSyncByte2);

  const auto decoded = radar::decode_frame(frame, 7);
  assert(decoded.valid);
  assert(decoded.points[0].sequence == 7);
  assert(decoded.points[0].sensor_id == 0);
  assert(decoded.points[0].angle_deg == 90);
  assert(decoded.points[0].distance_mm == 400);
  assert(decoded.points[1].sequence == 8);
  assert(decoded.points[1].sensor_id == 1);
  assert(decoded.points[1].angle_deg == 45);
  assert(decoded.points[1].distance_mm == 200);
  assert(decoded.points[2].sequence == 9);
  assert(decoded.points[2].sensor_id == 2);
  assert(decoded.points[2].angle_deg == 30);
  assert(decoded.points[2].distance_mm == 300);

  auto corrupt = frame;
  corrupt[4] ^= 0x55;
  const auto rejected = radar::decode_frame(corrupt, 8);
  assert(!rejected.valid);
}

void test_parser_resync() {
  radar::FrameParser parser;
  const std::vector<std::uint8_t> garbage{0x01, 0x02, 0xAA, 0x00, 0xAA};
  for (std::uint8_t byte : garbage) {
    assert(parser.ingest(byte).empty());
  }

  const std::array<radar::SensorReading, radar::kNumSensors> readings = {{
      {0, 45, 1234},
      {1, 20, 500},
      {2, 60, 600},
  }};
  const auto frame = radar::make_frame(readings);
  std::vector<radar::TelemetryPoint> points;
  for (std::size_t i = 1; i < frame.size(); ++i) {
    points = parser.ingest(frame[i]);
  }

  assert(!points.empty());
  assert(points[0].angle_deg == 45);
  assert(points[0].distance_mm == 1234);
  assert(parser.counters().valid_frames == 1);
  assert(parser.counters().sync_drops >= 3);

  // Sensor 0 timeout (distance == 0); sensors 1 and 2 non-zero.
  const std::array<radar::SensorReading, radar::kNumSensors> timeout_readings = {{
      {0, 46, 0},
      {1, 46, 200},
      {2, 46, 300},
  }};
  const auto timeout_frame = radar::make_frame(timeout_readings);
  for (std::uint8_t byte : timeout_frame) {
    points = parser.ingest(byte);
  }
  assert(!points.empty());
  assert(points[0].distance_mm == 0);
  assert(parser.counters().timeout_readings == 1);
}

void test_replay_source() {
  const auto temp = std::filesystem::temp_directory_path() / "radar_replay_test.bin";
  {
    std::ofstream output(temp, std::ios::binary);
    const std::array<radar::SensorReading, radar::kNumSensors> r_a = {{
        {0, 10, 100}, {1, 10, 110}, {2, 10, 120}
    }};
    const std::array<radar::SensorReading, radar::kNumSensors> r_b = {{
        {0, 20, 200}, {1, 20, 210}, {2, 20, 220}
    }};
    const auto a = radar::make_frame(r_a);
    const auto b = radar::make_frame(r_b);
    output.write(reinterpret_cast<const char *>(a.data()),
                 static_cast<std::streamsize>(a.size()));
    output.write(reinterpret_cast<const char *>(b.data()),
                 static_cast<std::streamsize>(b.size()));
  }

  int decoded_count = 0;
  {
    // Scoped so the ReplayByteSource's ifstream closes before we try to
    // delete the file below -- Windows refuses to remove a file that's
    // still open, and std::filesystem::remove throws on that failure.
    radar::ReplayByteSource replay(temp);
    radar::FrameParser parser;
    while (!replay.eof()) {
      const auto byte = replay.read_byte();
      if (!byte) {
        continue;
      }
      decoded_count += static_cast<int>(parser.ingest(*byte).size());
    }
  }
  assert(decoded_count == 6); // 2 frames × 3 sensors each
  std::filesystem::remove(temp);
}

radar::TelemetryPoint telemetry(std::uint8_t angle, std::uint16_t distance,
                                std::uint64_t sequence, std::uint8_t sensor_id = 0) {
  return radar::TelemetryPoint{angle, distance, sequence, sensor_id};
}

void test_sweep_builder() {
  radar::SweepBuilder builder;
  const auto base = RadarClock::now();

  assert(!builder.ingest(telemetry(0, 100, 0), base));
  assert(!builder.ingest(telemetry(0, 101, 1), base));
  assert(!builder.ingest(telemetry(1, 102, 2), base));
  assert(!builder.ingest(telemetry(2, 103, 3), base));

  auto forward = builder.ingest(telemetry(1, 104, 4), base);
  assert(forward);
  assert(forward->completed.direction == radar::SweepDirection::Forward);
  assert(!forward->completed_bidirectional_cycle);

  assert(!builder.ingest(telemetry(0, 105, 5), base));

  auto ret = builder.ingest(telemetry(1, 106, 6), base);
  assert(ret);
  assert(ret->completed.direction == radar::SweepDirection::Return);
  assert(ret->completed_bidirectional_cycle);
  assert(ret->previous_opposite);
}

void test_sweep_builder_sensor_id_propagation() {
  radar::SweepBuilder builder;
  const auto base = RadarClock::now();
  builder.ingest(telemetry(0, 100, 0, 7));
  auto forward = builder.ingest(telemetry(10, 101, 1, 7), base);
  assert(!forward); // direction not yet established
  assert(!builder.ingest(telemetry(20, 102, 2, 7)));
  auto event = builder.ingest(telemetry(10, 103, 3, 7));
  assert(event);
  for (const radar::RadarPoint &p : event->completed.points) {
    assert(p.sensor_id == 7);
  }
}

void test_sweep_builder_per_sensor_isolation() {
  // Sensor 0 sweeps an increasing angle sequence, sensor 1 sweeps a
  // decreasing one, interleaved in arrival order -- exactly what the 3
  // independently-paced sensor heads produce over UART.
  const std::vector<std::pair<std::uint8_t, std::uint8_t>> interleaved = {
      {0, 0}, {1, 170}, {0, 10}, {1, 160}, {0, 20}, {1, 150},
      {0, 30}, {1, 140}, {0, 40}, {1, 130}, {0, 50}, {1, 120},
  };

  // Routed through one builder per sensor_id, each builder only ever sees
  // its own monotonic sequence.
  std::array<radar::SweepBuilder, 2> per_sensor_builders;
  for (const auto &[sensor_id, angle] : interleaved) {
    per_sensor_builders[sensor_id].ingest(telemetry(angle, 1000, 0, sensor_id));
  }

  const auto &sensor0_sweep = per_sensor_builders[0].current_sweep();
  assert(sensor0_sweep);
  assert(sensor0_sweep->direction == radar::SweepDirection::Forward);
  assert(sensor0_sweep->points.size() == 6);

  const auto &sensor1_sweep = per_sensor_builders[1].current_sweep();
  assert(sensor1_sweep);
  assert(sensor1_sweep->direction == radar::SweepDirection::Return);
  assert(sensor1_sweep->points.size() == 6);

  // The same interleaved sequence fed into one shared builder corrupts
  // direction detection -- each cross-sensor jump (e.g. sensor0's 0 then
  // sensor1's 170) looks like a huge angle swing to the shared builder,
  // and a sweep ends up mixing points from both sensors.
  radar::SweepBuilder shared_builder;
  std::vector<radar::CompletedSweepEvent> shared_events;
  for (const auto &[sensor_id, angle] : interleaved) {
    auto event = shared_builder.ingest(telemetry(angle, 1000, 0, sensor_id));
    if (event) {
      shared_events.push_back(*event);
    }
  }
  assert(!shared_events.empty()); // direction flips on cross-sensor jumps
  bool any_mixed_sensor_sweep = false;
  for (const radar::CompletedSweepEvent &event : shared_events) {
    const std::uint8_t first_sensor = event.completed.points.front().sensor_id;
    for (const radar::RadarPoint &p : event.completed.points) {
      if (p.sensor_id != first_sensor) {
        any_mixed_sensor_sweep = true;
      }
    }
  }
  assert(any_mixed_sensor_sweep);
}

radar::Sweep make_sweep(std::vector<radar::RadarPoint> points,
                        radar::SweepDirection direction) {
  radar::Sweep sweep;
  sweep.direction = direction;
  sweep.points = std::move(points);
  return sweep;
}

radar::RadarPoint point(std::uint8_t angle, std::uint16_t distance,
                        std::uint64_t sequence, RadarClock::time_point time,
                        std::uint8_t sensor_id = 0) {
  radar::RadarPoint p;
  p.angle_deg = angle;
  p.distance_mm = distance;
  p.sequence = sequence;
  p.timestamp = time;
  p.sensor_id = sensor_id;
  return p;
}

void test_geometry_transform() {
  radar::SensorGeometry geometry; // default placeholder mounts

  // Main sensor: offset (0,0), heading 30 deg, angle_sign 1 -- local angle
  // 60 lands at world bearing 90 (straight up).
  {
    const auto world = radar::to_world(point(60, 1000, 0, RadarClock::now(), 0), geometry);
    assert(std::abs(world.world_x_mm - 0.0) < 1.0);
    assert(std::abs(world.world_y_mm - 1000.0) < 1.0);
    assert(std::abs(world.world_angle_deg - 90.0) < 0.5);
  }

  // Right flank mount is offset/heading-mirrored so that its local angle 0
  // lands exactly on the main sensor's local-angle-0 wedge boundary at the
  // same world point a matching main-sensor reading would reach.
  // (Flank baseline is 400 mm, so a flank local-0 reading at distance d lands
  // on the same world ray as a main local-0 reading at distance d + 400.)
  {
    const auto right = radar::to_world(point(0, 600, 0, RadarClock::now(), 2), geometry);
    const auto main_boundary =
        radar::to_world(point(0, 1000, 0, RadarClock::now(), 0), geometry);
    assert(std::abs(right.world_x_mm - main_boundary.world_x_mm) < 1.0);
    assert(std::abs(right.world_y_mm - main_boundary.world_y_mm) < 1.0);
  }

  // angle_sign mirrors the two flanks: left and right mounts are offset
  // and headed as mirror images of each other across the main sensor's
  // center bearing (world "up", x=0), so the same local angle/distance
  // lands at mirrored world coordinates -- x negates, y matches. (Their
  // world_angle_deg, the bearing *from the main sensor's origin*, isn't a
  // simple mirror once each mount's nonzero offset enters the atan2 -- the
  // x/y coordinates are the direct, offset-independent check on angle_sign
  // and heading_offset_deg.)
  {
    const auto left = radar::to_world(point(30, 100, 0, RadarClock::now(), 1), geometry);
    const auto right = radar::to_world(point(30, 100, 0, RadarClock::now(), 2), geometry);
    assert(std::abs(left.world_x_mm + right.world_x_mm) < 1.0);
    assert(std::abs(left.world_y_mm - right.world_y_mm) < 1.0);
  }

  // Sweep batch overload skips zero-distance (timeout) readings.
  {
    radar::Sweep sweep;
    sweep.points = {point(10, 0, 0, RadarClock::now(), 0),
                    point(20, 500, 1, RadarClock::now(), 0)};
    const auto world_points = radar::to_world(sweep, geometry);
    assert(world_points.size() == 1);
    assert(world_points[0].source.angle_deg == 20);
  }
}

void test_corroboration() {
  radar::SensorGeometry geometry;
  const auto t0 = RadarClock::now();

  // Main local angle 0 (within the 15 deg boundary band, paired with
  // sensor 2) and right-flank local angle 0 (within the 30 deg inward
  // band, paired with sensor 0) are mounted so that main distance 700 and
  // flank distance 300 land on the exact same world point (the 400 mm flank
  // baseline cancels out at heading 30 deg). Both are within their sensor's
  // max range (main 800, flank 600).
  const auto main_in_band = point(0, 700, 0, t0, 0);
  const auto right_in_band = point(0, 300, 0, t0, 2);
  const auto main_out_of_band = point(60, 700, 0, t0, 0); // mid-wedge, no band

  // Confirmed: paired sensor reported a geometrically consistent point
  // recently.
  {
    radar::Corroborator corroborator(geometry);
    corroborator.update(right_in_band);
    const auto world =
        corroborator.update(point(0, 700, 0, t0 + std::chrono::milliseconds(100), 0));
    assert(world.source.confirmation == radar::Confirmation::Confirmed);
  }

  // Rejected: in-band, but the paired sensor's buffer is empty.
  {
    radar::Corroborator corroborator(geometry);
    const auto world = corroborator.update(main_in_band);
    assert(world.source.confirmation == radar::Confirmation::Rejected);
  }

  // Unchecked: outside any overlap band, regardless of other sensors.
  {
    radar::Corroborator corroborator(geometry);
    corroborator.update(right_in_band);
    const auto world = corroborator.update(main_out_of_band);
    assert(world.source.confirmation == radar::Confirmation::Unchecked);
  }

  // Time-window expiry: a match exists but is older than the configured
  // window, so it can no longer corroborate.
  {
    radar::Corroborator corroborator(geometry);
    corroborator.update(right_in_band); // at t0
    const auto world = corroborator.update(
        point(0, 700, 0, t0 + std::chrono::milliseconds(600), 0)); // default window is 500ms
    assert(world.source.confirmation == radar::Confirmation::Rejected);
  }

  // Bad reading: a sanity-bound violation (or zero/timeout reading) is
  // never buffered, so it can't corroborate a later in-band detection.
  {
    radar::Corroborator corroborator(geometry);
    auto bad = right_in_band;
    bad.distance_mm = 9000; // beyond sensor 2's max range (600 mm)
    const auto bad_world = corroborator.update(bad);
    assert(bad_world.source.confirmation == radar::Confirmation::Unchecked);

    const auto world =
        corroborator.update(point(0, 700, 0, t0 + std::chrono::milliseconds(100), 0));
    assert(world.source.confirmation == radar::Confirmation::Rejected);
  }

  // The zero-distance (timeout) reading itself stays Unchecked rather than
  // being treated as in-band-but-no-match.
  {
    radar::Corroborator corroborator(geometry);
    const auto world = corroborator.update(point(0, 0, 0, t0, 0));
    assert(world.source.confirmation == radar::Confirmation::Unchecked);
  }
}

void test_mti() {
  const auto t0 = RadarClock::now();
  const auto t1 = t0 + std::chrono::seconds(1);

  for (std::uint16_t delta : {50, 100, 150}) {
    auto prev = make_sweep({point(0, 1000, 1, t0)}, radar::SweepDirection::Forward);
    auto cur = make_sweep({point(0, static_cast<std::uint16_t>(1000 + delta), 2, t1)},
                          radar::SweepDirection::Return);
    const auto vectors = radar::compute_motion_cpu(prev, cur, delta + 1.0);
    assert(vectors.size() == 1);
    assert(std::abs(vectors[0].displacement_mm - delta) < 0.001);
    assert(std::abs(vectors[0].speed_mm_s - delta) < 0.001);
  }

  auto prev = make_sweep({point(0, 1000, 1, t0), point(90, 1000, 2, t0)},
                         radar::SweepDirection::Forward);
  auto cur = make_sweep({point(0, 1050, 3, t1), point(90, 1030, 4, t1)},
                        radar::SweepDirection::Return);
  assert(radar::compute_motion_cpu(prev, cur, 80).size() == 2);

  auto rejected =
      radar::compute_motion_cpu(prev,
                                make_sweep({point(0, 1200, 5, t1)},
                                           radar::SweepDirection::Return),
                                80);
  assert(rejected.empty());

  auto duplicate = radar::compute_motion_cpu(
      make_sweep({point(0, 1000, 1, t0)}, radar::SweepDirection::Forward),
      make_sweep({point(0, 1040, 2, t1), point(0, 1020, 3, t1)},
                 radar::SweepDirection::Return),
      80);
  assert(duplicate.size() == 1);
  assert(std::abs(duplicate[0].displacement_mm - 20.0) < 0.001);
}

void test_pbm() {
  radar::RgbaFrame frame;
  frame.width = 8;
  frame.height = 2;
  frame.rgba.assign(8U * 2U * 4U, 0);
  frame.rgba[0] = 255;
  frame.rgba[1] = 255;
  frame.rgba[2] = 255;
  frame.rgba[3] = 255;

  const auto bits = radar::rgba_to_pbm_bits(frame, 128);
  assert(bits.size() == 2);
  assert(bits[0] == 0x80);
  assert(bits[1] == 0x00);

  const auto temp = std::filesystem::temp_directory_path() / "radar_test.pbm";
  radar::write_pbm(temp, frame, 128);
  std::string magic;
  {
    // Scoped so the ifstream closes before removing the file below --
    // Windows refuses to delete a file that's still open.
    std::ifstream input(temp, std::ios::binary);
    input >> magic;
  }
  assert(magic == "P4");
  std::filesystem::remove(temp);
}

} // namespace

int main() {
  test_protocol_round_trip();
  test_parser_resync();
  test_replay_source();
  test_sweep_builder();
  test_sweep_builder_sensor_id_propagation();
  test_sweep_builder_per_sensor_isolation();
  test_geometry_transform();
  test_corroboration();
  test_mti();
  test_pbm();
  std::cout << "radar_tests passed\n";
  return 0;
}
