#include "radar/mti.hpp"

namespace radar {

bool GpuMti::available() const {
  return false;
}

std::string GpuMti::backend_name() const {
#if RADAR_HAS_VULKAN
  return "vulkan-compute-contract-cpu-reference";
#else
  return "cpu-reference";
#endif
}

std::vector<MotionVector> GpuMti::compute(const Sweep &previous,
                                          const Sweep &current,
                                          double tau_mm) {
  return compute_motion_cpu(previous, current, tau_mm);
}

} // namespace radar
