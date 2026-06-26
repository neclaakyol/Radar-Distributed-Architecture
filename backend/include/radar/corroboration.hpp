#pragma once

#include "radar/geometry.hpp"
#include "radar/sweep.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <vector>

namespace radar {

// Configuration for cross-sensor corroboration in the wedge-boundary
// overlap bands. Angular bands are expressed in each sensor's OWN local
// angle frame (the frame sweep direction-detection naturally operates in,
// before any world-frame transform).
struct CorroborationConfig {
  // Main sensor (local range [0,120]): width, in degrees, of the band
  // counted as "near a boundary edge", applied inward from each end.
  double main_boundary_band_deg = 15.0;

  // Flank sensors (local range [0,120]): width, in degrees, of the band
  // nearest local angle 0 counted as the inward/overlap-facing portion of
  // their sweep (see geometry.hpp's mount placeholders -- local angle 0 is
  // mounted at the boundary edge for both flanks by construction).
  double flank_inward_band_deg = 30.0;

  double match_distance_tolerance_mm = 100.0;
  std::chrono::milliseconds match_time_window{500};
};

// Tracks each sensor's most recently seen world-frame points so that
// detections from independently-paced, asynchronously-sweeping sensors can
// still be cross-matched without waiting for simultaneous sweep
// completion.
class Corroborator {
public:
  explicit Corroborator(SensorGeometry geometry, CorroborationConfig config = {});

  // Feed one freshly-arrived local-frame RadarPoint (sensor_id already
  // set). Returns the world-frame transform of the point, with
  // source.confirmation set to:
  //   - Unchecked: distance failed the sanity bound, or the point is
  //     outside any overlap band (no corroboration possible/needed).
  //   - Confirmed: in an overlap band and the paired sensor's recent
  //     buffer has a world-position match within tolerance and the time
  //     window.
  //   - Rejected: in an overlap band but no such match was found.
  WorldPoint update(const RadarPoint &point);

  // Convenience: process every point of a freshly-completed sweep, in
  // order (order matters for the recent-points time-window logic).
  std::vector<WorldPoint> update(const Sweep &completed_sweep);

  void reset();

private:
  struct BandMatch {
    bool in_band = false;
    std::uint8_t paired_sensor_id = 0;
  };

  BandMatch overlap_band(const RadarPoint &point) const;
  bool find_corroborator(const WorldPoint &candidate, std::uint8_t paired_sensor_id) const;
  void prune(std::deque<WorldPoint> &buffer, RadarClock::time_point now) const;

  SensorGeometry geometry_;
  CorroborationConfig config_;
  std::array<std::deque<WorldPoint>, kNumSensors> recent_by_sensor_;
};

} // namespace radar
