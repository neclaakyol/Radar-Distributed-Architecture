#include "radar/sweep.hpp"

#include <utility>

namespace radar {
namespace {

Sweep make_sweep(SweepDirection direction, const RadarPoint &first_point,
                 std::uint64_t id) {
  Sweep sweep;
  sweep.direction = direction;
  sweep.started_at = first_point.timestamp;
  sweep.ended_at = first_point.timestamp;
  sweep.id = id;
  sweep.points.push_back(first_point);
  return sweep;
}

} // namespace

std::optional<CompletedSweepEvent>
SweepBuilder::ingest(const TelemetryPoint &telemetry,
                     RadarClock::time_point timestamp) {
  if (telemetry.distance_mm == 0) {
    return std::nullopt;
  }

  RadarPoint point;
  point.angle_deg = telemetry.angle_deg;
  point.distance_mm = telemetry.distance_mm;
  point.timestamp = timestamp;
  point.sequence = telemetry.sequence;

  if (!last_point_) {
    last_point_ = point;
    return std::nullopt;
  }

  if (point.angle_deg == last_point_->angle_deg) {
    if (current_) {
      current_->points.push_back(point);
      current_->ended_at = point.timestamp;
    } else {
      last_point_ = point;
    }
    return std::nullopt;
  }

  const SweepDirection detected =
      point.angle_deg > last_point_->angle_deg ? SweepDirection::Forward
                                               : SweepDirection::Return;

  if (!current_) {
    current_ = make_sweep(detected, *last_point_, next_sweep_id_++);
    current_->points.push_back(point);
    current_->ended_at = point.timestamp;
    last_point_ = point;
    return std::nullopt;
  }

  if (current_->direction != detected) {
    CompletedSweepEvent event;
    event.completed = std::move(*current_);

    if (event.completed.direction == SweepDirection::Forward) {
      event.previous_opposite = last_return_;
      last_forward_ = event.completed;
    } else {
      event.previous_opposite = last_forward_;
      last_return_ = event.completed;
    }

    event.completed_bidirectional_cycle = event.previous_opposite.has_value();
    if (event.completed_bidirectional_cycle) {
      event.cycle_index = next_cycle_index_++;
    }

    current_ = make_sweep(detected, point, next_sweep_id_++);
    last_point_ = point;
    return event;
  }

  current_->points.push_back(point);
  current_->ended_at = point.timestamp;
  last_point_ = point;
  return std::nullopt;
}

void SweepBuilder::reset() {
  last_point_.reset();
  current_.reset();
  last_forward_.reset();
  last_return_.reset();
  next_sweep_id_ = 1;
  next_cycle_index_ = 1;
}

const char *to_string(SweepDirection direction) {
  switch (direction) {
  case SweepDirection::Forward:
    return "forward";
  case SweepDirection::Return:
    return "return";
  case SweepDirection::Unknown:
    return "unknown";
  }
  return "unknown";
}

} // namespace radar
