#pragma once

#include "radar/protocol.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

namespace radar {

using RadarClock = std::chrono::steady_clock;

enum class SweepDirection {
  Unknown,
  Forward,
  Return,
};

struct RadarPoint {
  std::uint8_t angle_deg = 0;
  std::uint16_t distance_mm = 0;
  RadarClock::time_point timestamp{};
  std::uint64_t sequence = 0;
};

struct Sweep {
  SweepDirection direction = SweepDirection::Unknown;
  std::vector<RadarPoint> points;
  RadarClock::time_point started_at{};
  RadarClock::time_point ended_at{};
  std::uint64_t id = 0;
};

struct CompletedSweepEvent {
  Sweep completed;
  std::optional<Sweep> previous_opposite;
  bool completed_bidirectional_cycle = false;
  std::uint64_t cycle_index = 0;
};

class SweepBuilder {
public:
  std::optional<CompletedSweepEvent>
  ingest(const TelemetryPoint &telemetry,
         RadarClock::time_point timestamp = RadarClock::now());

  void reset();
  const std::optional<Sweep> &current_sweep() const { return current_; }
  const std::optional<Sweep> &last_forward_sweep() const { return last_forward_; }
  const std::optional<Sweep> &last_return_sweep() const { return last_return_; }

private:
  std::optional<RadarPoint> last_point_;
  std::optional<Sweep> current_;
  std::optional<Sweep> last_forward_;
  std::optional<Sweep> last_return_;
  std::uint64_t next_sweep_id_ = 1;
  std::uint64_t next_cycle_index_ = 1;
};

const char *to_string(SweepDirection direction);

} // namespace radar
