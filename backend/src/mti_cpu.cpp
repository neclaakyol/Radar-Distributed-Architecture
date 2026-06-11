#include "radar/mti.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace radar {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

double distance_between(const CartesianPoint &a, const CartesianPoint &b) {
  const double dx = b.x_mm - a.x_mm;
  const double dy = b.y_mm - a.y_mm;
  return std::sqrt(dx * dx + dy * dy);
}

double seconds_between(const RadarPoint &previous, const RadarPoint &current) {
  const auto delta = current.timestamp - previous.timestamp;
  const double seconds = std::chrono::duration<double>(delta).count();
  return seconds > 0.0 ? seconds : 0.0;
}

} // namespace

CartesianPoint polar_to_cartesian(const RadarPoint &point) {
  const double radians = static_cast<double>(point.angle_deg) * kPi / 180.0;
  CartesianPoint cartesian;
  cartesian.x_mm = static_cast<double>(point.distance_mm) * std::cos(radians);
  cartesian.y_mm = static_cast<double>(point.distance_mm) * std::sin(radians);
  cartesian.source = point;
  return cartesian;
}

std::vector<MotionVector> compute_motion_cpu(const Sweep &previous,
                                             const Sweep &current,
                                             double tau_mm) {
  struct Candidate {
    std::size_t previous_index = 0;
    std::size_t current_index = 0;
    double distance_mm = 0.0;
  };

  std::vector<CartesianPoint> previous_points;
  previous_points.reserve(previous.points.size());
  for (const RadarPoint &point : previous.points) {
    if (point.distance_mm != 0) {
      previous_points.push_back(polar_to_cartesian(point));
    }
  }

  std::vector<CartesianPoint> current_points;
  current_points.reserve(current.points.size());
  for (const RadarPoint &point : current.points) {
    if (point.distance_mm != 0) {
      current_points.push_back(polar_to_cartesian(point));
    }
  }

  std::vector<std::optional<Candidate>> best_by_previous(previous_points.size());

  for (std::size_t current_index = 0; current_index < current_points.size();
       ++current_index) {
    double best_distance = std::numeric_limits<double>::infinity();
    std::optional<std::size_t> best_previous_index;

    for (std::size_t previous_index = 0;
         previous_index < previous_points.size(); ++previous_index) {
      const double distance =
          distance_between(previous_points[previous_index],
                           current_points[current_index]);
      if (distance <= tau_mm && distance < best_distance) {
        best_distance = distance;
        best_previous_index = previous_index;
      }
    }

    if (!best_previous_index) {
      continue;
    }

    Candidate candidate{*best_previous_index, current_index, best_distance};
    auto &slot = best_by_previous[*best_previous_index];
    if (!slot || candidate.distance_mm < slot->distance_mm ||
        (candidate.distance_mm == slot->distance_mm &&
         candidate.current_index < slot->current_index)) {
      slot = candidate;
    }
  }

  std::vector<Candidate> accepted;
  for (const auto &candidate : best_by_previous) {
    if (candidate) {
      accepted.push_back(*candidate);
    }
  }

  std::sort(accepted.begin(), accepted.end(),
            [&](const Candidate &lhs, const Candidate &rhs) {
              const auto lhs_seq =
                  current_points[lhs.current_index].source.sequence;
              const auto rhs_seq =
                  current_points[rhs.current_index].source.sequence;
              if (lhs_seq != rhs_seq) {
                return lhs_seq < rhs_seq;
              }
              return lhs.previous_index < rhs.previous_index;
            });

  std::vector<MotionVector> vectors;
  vectors.reserve(accepted.size());

  for (const Candidate &candidate : accepted) {
    const CartesianPoint &previous_cart =
        previous_points[candidate.previous_index];
    const CartesianPoint &current_cart = current_points[candidate.current_index];

    MotionVector vector;
    vector.previous = previous_cart;
    vector.current = current_cart;
    vector.dx_mm = current_cart.x_mm - previous_cart.x_mm;
    vector.dy_mm = current_cart.y_mm - previous_cart.y_mm;
    vector.displacement_mm =
        std::sqrt(vector.dx_mm * vector.dx_mm + vector.dy_mm * vector.dy_mm);
    vector.heading_deg = std::atan2(vector.dy_mm, vector.dx_mm) * 180.0 / kPi;

    const double seconds =
        seconds_between(previous_cart.source, current_cart.source);
    vector.speed_mm_s = seconds > 0.0 ? vector.displacement_mm / seconds : 0.0;
    vectors.push_back(vector);
  }

  return vectors;
}

} // namespace radar
