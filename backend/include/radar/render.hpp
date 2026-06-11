#pragma once

#include "radar/mti.hpp"
#include "radar/protocol.hpp"
#include "radar/raster.hpp"
#include "radar/sweep.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace radar {

struct RenderStats {
  ProtocolCounters protocol;
  std::uint64_t completed_sweeps = 0;
  std::uint64_t completed_cycles = 0;
  std::uint64_t vector_count = 0;
};

struct RenderState {
  std::vector<RadarPoint> current_points;
  std::vector<RadarPoint> history_points;
  std::vector<MotionVector> vectors;
  std::uint8_t sweep_angle_deg = 0;
  RenderStats stats;
  std::string source_name;
};

class SoftwareRenderer {
public:
  SoftwareRenderer(std::uint32_t width = 800, std::uint32_t height = 480);

  RgbaFrame render(const RenderState &state) const;

private:
  std::uint32_t width_;
  std::uint32_t height_;
};

} // namespace radar
