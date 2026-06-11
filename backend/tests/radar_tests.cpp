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
#include <optional>
#include <utility>
#include <vector>

namespace {

using radar::RadarClock;

void test_protocol_round_trip() {
  const auto frame = radar::make_frame(90, 400);
  assert(frame[0] == radar::kSyncByte1);
  assert(frame[1] == radar::kSyncByte2);

  const auto decoded = radar::decode_frame(frame, 7);
  assert(decoded.valid);
  assert(decoded.point.sequence == 7);
  assert(decoded.point.angle_deg == 90);
  assert(decoded.point.distance_mm == 400);

  auto corrupt = frame;
  corrupt[4] ^= 0x55;
  const auto rejected = radar::decode_frame(corrupt, 8);
  assert(!rejected.valid);
}

void test_parser_resync() {
  radar::FrameParser parser;
  const std::vector<std::uint8_t> garbage{0x01, 0x02, 0xAA, 0x00, 0xAA};
  for (std::uint8_t byte : garbage) {
    assert(!parser.ingest(byte));
  }

  const auto frame = radar::make_frame(45, 1234);
  std::optional<radar::TelemetryPoint> point;
  for (std::size_t i = 1; i < frame.size(); ++i) {
    point = parser.ingest(frame[i]);
  }

  assert(point);
  assert(point->angle_deg == 45);
  assert(point->distance_mm == 1234);
  assert(parser.counters().valid_frames == 1);
  assert(parser.counters().sync_drops >= 3);

  const auto timeout_frame = radar::make_frame(46, 0);
  for (std::uint8_t byte : timeout_frame) {
    point = parser.ingest(byte);
  }
  assert(point);
  assert(point->distance_mm == 0);
  assert(parser.counters().timeout_readings == 1);
}

void test_replay_source() {
  const auto temp = std::filesystem::temp_directory_path() / "radar_replay_test.bin";
  {
    std::ofstream output(temp, std::ios::binary);
    const auto a = radar::make_frame(10, 100);
    const auto b = radar::make_frame(20, 200);
    output.write(reinterpret_cast<const char *>(a.data()),
                 static_cast<std::streamsize>(a.size()));
    output.write(reinterpret_cast<const char *>(b.data()),
                 static_cast<std::streamsize>(b.size()));
  }

  radar::ReplayByteSource replay(temp);
  radar::FrameParser parser;
  int decoded_count = 0;
  while (!replay.eof()) {
    const auto byte = replay.read_byte();
    if (!byte) {
      continue;
    }
    if (parser.ingest(*byte)) {
      ++decoded_count;
    }
  }
  assert(decoded_count == 2);
  std::filesystem::remove(temp);
}

radar::TelemetryPoint telemetry(std::uint8_t angle, std::uint16_t distance,
                                std::uint64_t sequence) {
  return radar::TelemetryPoint{angle, distance, sequence};
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

radar::Sweep make_sweep(std::vector<radar::RadarPoint> points,
                        radar::SweepDirection direction) {
  radar::Sweep sweep;
  sweep.direction = direction;
  sweep.points = std::move(points);
  return sweep;
}

radar::RadarPoint point(std::uint8_t angle, std::uint16_t distance,
                        std::uint64_t sequence, RadarClock::time_point time) {
  radar::RadarPoint p;
  p.angle_deg = angle;
  p.distance_mm = distance;
  p.sequence = sequence;
  p.timestamp = time;
  return p;
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
  std::ifstream input(temp, std::ios::binary);
  std::string magic;
  input >> magic;
  assert(magic == "P4");
  std::filesystem::remove(temp);
}

} // namespace

int main() {
  test_protocol_round_trip();
  test_parser_resync();
  test_replay_source();
  test_sweep_builder();
  test_mti();
  test_pbm();
  std::cout << "radar_tests passed\n";
  return 0;
}
