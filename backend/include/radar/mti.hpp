#pragma once

#include "radar/sweep.hpp"

#include <string>
#include <vector>

namespace radar {

struct CartesianPoint {
  double x_mm = 0.0;
  double y_mm = 0.0;
  RadarPoint source;
};

struct MotionVector {
  CartesianPoint previous;
  CartesianPoint current;
  double dx_mm = 0.0;
  double dy_mm = 0.0;
  double displacement_mm = 0.0;
  double heading_deg = 0.0;
  double speed_mm_s = 0.0;
};

CartesianPoint polar_to_cartesian(const RadarPoint &point);
std::vector<MotionVector> compute_motion_cpu(const Sweep &previous,
                                             const Sweep &current,
                                             double tau_mm);

class GpuMti {
public:
  bool available() const;
  std::string backend_name() const;

  std::vector<MotionVector> compute(const Sweep &previous, const Sweep &current,
                                    double tau_mm);
};

} // namespace radar
