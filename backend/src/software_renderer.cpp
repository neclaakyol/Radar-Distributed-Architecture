#include "radar/geometry.hpp"
#include "radar/render.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>

namespace radar {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

struct Color {
  std::uint8_t r = 0;
  std::uint8_t g = 0;
  std::uint8_t b = 0;
  std::uint8_t a = 255;
};

struct Point2 {
  int x = 0;
  int y = 0;
};

// --- Palette (dark radar background; everything else pops on top) ---
constexpr Color kBackground{4, 6, 10, 255};
constexpr Color kBorder{150, 152, 165, 255};   // coverage shield outline
constexpr Color kSensor{235, 60, 60, 255};      // sensor markers (red)
constexpr Color kSensorRing{255, 120, 120, 255};
constexpr Color kRay{236, 238, 248, 255};       // scan ray (white)
constexpr Color kDot{60, 230, 95, 255};         // detected object (green)
constexpr Color kDotConfirmed{255, 255, 255, 255};
constexpr Color kArrow{170, 240, 120, 255};     // MTI motion arrow
constexpr Color kLabel{150, 158, 165, 255};
constexpr Color kHud{120, 150, 130, 255};

// On-screen placement + 120 deg fan for one sensor, matching the user's
// sketch: HY-SRF05 bottom-center sweeping up, two HC-SR04s on the left/right
// sweeping inward. Screen angle convention: degrees CCW from +x, with +deg
// pointing up (screen y grows downward, handled in dir_of()).
//   screen_deg(local) = base_deg + sign * local_angle   (local in [0,120])
struct SensorView {
  Point2 anchor;
  double base_deg;
  double sign;
  const char *label;
};

constexpr int kRayLength = 340; // screen px for a full-range (max_range) reading

// Indexed by sensor_id (0,1 = HC-SR04 flanks, 2 = HY-SRF05 base).
const std::array<SensorView, kNumSensors> kLayout{{
    {{110, 250}, -54.0, 1.0, "HC-SR04"},   // sensor 0: left, fan opens right
    {{690, 250}, 114.0, 1.0, "HC-SR04"},   // sensor 1: right, fan opens left
    {{400, 440}, 30.0, 1.0, "HY-SRF05"},   // sensor 2: bottom, fan opens up
}};

// Coverage-shield corners above the flank sensors (border drawing only).
constexpr Point2 kCornerLeft{150, 70};
constexpr Point2 kCornerRight{650, 70};
constexpr Point2 kArcControl{400, 10}; // pulls the top edge into an arc

std::pair<double, double> dir_of(double screen_deg) {
  const double rad = screen_deg * kPi / 180.0;
  return {std::cos(rad), -std::sin(rad)};
}

void put_pixel(RgbaFrame &frame, int x, int y, Color color) {
  if (x < 0 || y < 0 || x >= static_cast<int>(frame.width) ||
      y >= static_cast<int>(frame.height)) {
    return;
  }
  const std::size_t offset =
      (static_cast<std::size_t>(y) * frame.width + static_cast<std::size_t>(x)) * 4U;
  // max-blend: bright marks win over the dark background and each other.
  frame.rgba[offset + 0] = std::max(frame.rgba[offset + 0], color.r);
  frame.rgba[offset + 1] = std::max(frame.rgba[offset + 1], color.g);
  frame.rgba[offset + 2] = std::max(frame.rgba[offset + 2], color.b);
  frame.rgba[offset + 3] = color.a;
}

void draw_line(RgbaFrame &frame, Point2 a, Point2 b, Color color) {
  int dx = std::abs(b.x - a.x);
  int sx = a.x < b.x ? 1 : -1;
  int dy = -std::abs(b.y - a.y);
  int sy = a.y < b.y ? 1 : -1;
  int err = dx + dy;
  for (;;) {
    put_pixel(frame, a.x, a.y, color);
    if (a.x == b.x && a.y == b.y) {
      break;
    }
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      a.x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      a.y += sy;
    }
  }
}

// 2 px line so rays / the border read clearly against the dark field.
void draw_line_thick(RgbaFrame &frame, Point2 a, Point2 b, Color color) {
  draw_line(frame, a, b, color);
  draw_line(frame, {a.x + 1, a.y}, {b.x + 1, b.y}, color);
  draw_line(frame, {a.x, a.y + 1}, {b.x, b.y + 1}, color);
}

void draw_filled_circle(RgbaFrame &frame, Point2 center, int radius, Color color) {
  for (int y = -radius; y <= radius; ++y) {
    for (int x = -radius; x <= radius; ++x) {
      if (x * x + y * y <= radius * radius) {
        put_pixel(frame, center.x + x, center.y + y, color);
      }
    }
  }
}

void draw_ring(RgbaFrame &frame, Point2 center, int radius, Color color) {
  const int outer = radius * radius;
  const int inner = (radius - 1) * (radius - 1);
  for (int y = -radius; y <= radius; ++y) {
    for (int x = -radius; x <= radius; ++x) {
      const int d = x * x + y * y;
      if (d <= outer && d >= inner) {
        put_pixel(frame, center.x + x, center.y + y, color);
      }
    }
  }
}

// Quadratic Bezier, used for the rounded top edge of the coverage shield.
void draw_bezier(RgbaFrame &frame, Point2 p0, Point2 ctrl, Point2 p1, Color color) {
  Point2 prev = p0;
  constexpr int steps = 48;
  for (int i = 1; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    const double u = 1.0 - t;
    const double x = u * u * p0.x + 2 * u * t * ctrl.x + t * t * p1.x;
    const double y = u * u * p0.y + 2 * u * t * ctrl.y + t * t * p1.y;
    const Point2 cur{static_cast<int>(std::round(x)), static_cast<int>(std::round(y))};
    draw_line(frame, prev, cur, color);
    prev = cur;
  }
}

void draw_arrow(RgbaFrame &frame, Point2 from, Point2 to, Color color) {
  draw_line(frame, from, to, color);
  const double angle = std::atan2(static_cast<double>(to.y - from.y),
                                  static_cast<double>(to.x - from.x));
  constexpr double wing = 0.55;
  constexpr int length = 8;
  Point2 left{to.x - static_cast<int>(std::round(std::cos(angle - wing) * length)),
              to.y - static_cast<int>(std::round(std::sin(angle - wing) * length))};
  Point2 right{to.x - static_cast<int>(std::round(std::cos(angle + wing) * length)),
               to.y - static_cast<int>(std::round(std::sin(angle + wing) * length))};
  draw_line(frame, to, left, color);
  draw_line(frame, to, right, color);
}

std::array<std::uint8_t, 7> glyph(char c) {
  switch (c) {
  case '0': return {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E};
  case '1': return {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E};
  case '2': return {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F};
  case '3': return {0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E};
  case '4': return {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02};
  case '5': return {0x1F, 0x10, 0x10, 0x1E, 0x01, 0x01, 0x1E};
  case '6': return {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E};
  case '7': return {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08};
  case '8': return {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E};
  case '9': return {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C};
  case 'A': return {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11};
  case 'B': return {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E};
  case 'C': return {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E};
  case 'D': return {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E};
  case 'E': return {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F};
  case 'F': return {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10};
  case 'G': return {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F};
  case 'H': return {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11};
  case 'I': return {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E};
  case 'J': return {0x07, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0C};
  case 'K': return {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11};
  case 'L': return {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F};
  case 'M': return {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11};
  case 'N': return {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11};
  case 'O': return {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E};
  case 'P': return {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10};
  case 'Q': return {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D};
  case 'R': return {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11};
  case 'S': return {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E};
  case 'T': return {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04};
  case 'U': return {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E};
  case 'V': return {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04};
  case 'W': return {0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11};
  case 'X': return {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11};
  case 'Y': return {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04};
  case 'Z': return {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F};
  case ':': return {0x00, 0x04, 0x04, 0x00, 0x04, 0x04, 0x00};
  case '-': return {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00};
  case '.': return {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C};
  case '/': return {0x01, 0x01, 0x02, 0x04, 0x08, 0x10, 0x10};
  case ' ': return {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  default: return {0x1F, 0x11, 0x15, 0x15, 0x15, 0x11, 0x1F};
  }
}

int text_width(const std::string &text, int scale) {
  return static_cast<int>(text.size()) * 6 * scale;
}

void draw_text(RgbaFrame &frame, int x, int y, std::string text, Color color, int scale = 1) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
    return static_cast<char>(std::toupper(ch));
  });
  int cursor_x = x;
  for (char c : text) {
    const auto rows = glyph(c);
    for (int row = 0; row < 7; ++row) {
      for (int col = 0; col < 5; ++col) {
        if ((rows[row] & (0x10 >> col)) == 0) {
          continue;
        }
        for (int sy = 0; sy < scale; ++sy) {
          for (int sx = 0; sx < scale; ++sx) {
            put_pixel(frame, cursor_x + col * scale + sx, y + row * scale + sy, color);
          }
        }
      }
    }
    cursor_x += 6 * scale;
  }
}

double max_display_range(const RenderState &state) {
  std::uint16_t max_distance = 1000;
  auto scan = [&](const std::vector<RadarPoint> &points) {
    for (const RadarPoint &point : points) {
      max_distance = std::max(max_distance, point.distance_mm);
    }
  };
  scan(state.history_points);
  scan(state.current_points);
  for (const MotionVector &vector : state.vectors) {
    max_distance = std::max(max_distance, vector.current.source.distance_mm);
    max_distance = std::max(max_distance, vector.previous.source.distance_mm);
  }
  return static_cast<double>(max_distance) * 1.1;
}

// Projects a (sensor, local angle, distance) reading into screen space along
// that sensor's on-screen fan.
Point2 plot(const SensorView &view, double local_angle, std::uint16_t distance_mm,
            double max_range) {
  const double frac = std::clamp(static_cast<double>(distance_mm) / max_range, 0.0, 1.0);
  const auto [dx, dy] = dir_of(view.base_deg + view.sign * local_angle);
  const double len = frac * kRayLength;
  return Point2{view.anchor.x + static_cast<int>(std::round(dx * len)),
                view.anchor.y + static_cast<int>(std::round(dy * len))};
}

} // namespace

SoftwareRenderer::SoftwareRenderer(std::uint32_t width, std::uint32_t height)
    : width_(width), height_(height) {}

RgbaFrame SoftwareRenderer::render(const RenderState &state) const {
  RgbaFrame frame;
  frame.width = width_;
  frame.height = height_;
  frame.rgba.assign(static_cast<std::size_t>(width_) * height_ * 4U, 255);
  for (std::size_t i = 0; i + 3 < frame.rgba.size(); i += 4) {
    frame.rgba[i + 0] = kBackground.r;
    frame.rgba[i + 1] = kBackground.g;
    frame.rgba[i + 2] = kBackground.b;
    frame.rgba[i + 3] = 255;
  }

  const double max_range = max_display_range(state);

  // --- Coverage border (shield through the 3 sensors + arched top) ---
  draw_line_thick(frame, kCornerLeft, kLayout[0].anchor, kBorder);
  draw_line_thick(frame, kLayout[0].anchor, kLayout[2].anchor, kBorder);
  draw_line_thick(frame, kLayout[2].anchor, kLayout[1].anchor, kBorder);
  draw_line_thick(frame, kLayout[1].anchor, kCornerRight, kBorder);
  draw_bezier(frame, kCornerLeft, kArcControl, kCornerRight, kBorder);

  // --- Per-sensor live scan ray (white), at each sensor's current angle ---
  for (std::size_t sid = 0; sid < kLayout.size(); ++sid) {
    const SensorView &view = kLayout[sid];
    const auto [dx, dy] = dir_of(view.base_deg + view.sign * state.sweep_angle_by_sensor[sid]);
    const Point2 tip{view.anchor.x + static_cast<int>(std::round(dx * kRayLength)),
                     view.anchor.y + static_cast<int>(std::round(dy * kRayLength))};
    draw_line_thick(frame, view.anchor, tip, kRay);
  }

  // --- Detected objects (green dots); confirmed get a white ring ---
  for (const RadarPoint &point : state.current_points) {
    if (point.distance_mm == 0 || point.sensor_id >= kLayout.size()) {
      continue;
    }
    const Point2 p = plot(kLayout[point.sensor_id], point.angle_deg, point.distance_mm, max_range);
    if (point.confirmation == Confirmation::Confirmed) {
      draw_ring(frame, p, 5, kDotConfirmed);
    }
    draw_filled_circle(frame, p, 3, kDot);
  }

  // --- MTI motion arrows (object movement between sweeps) ---
  for (const MotionVector &vector : state.vectors) {
    const std::uint8_t sid = vector.current.source.sensor_id;
    if (sid >= kLayout.size()) {
      continue;
    }
    const Point2 from = plot(kLayout[sid], vector.previous.source.angle_deg,
                             vector.previous.source.distance_mm, max_range);
    const Point2 to = plot(kLayout[sid], vector.current.source.angle_deg,
                           vector.current.source.distance_mm, max_range);
    draw_arrow(frame, from, to, kArrow);
  }

  // --- Sensor markers (red) on top, with labels ---
  for (const SensorView &view : kLayout) {
    draw_filled_circle(frame, view.anchor, 5, kSensor);
    draw_ring(frame, view.anchor, 8, kSensorRing);
    const std::string name = view.label;
    draw_text(frame, view.anchor.x - text_width(name, 1) / 2, view.anchor.y + 14, name, kLabel, 1);
    draw_text(frame, view.anchor.x - text_width("120 DEG", 1) / 2, view.anchor.y + 26,
              "120 DEG", kLabel, 1);
  }

  // --- Compact HUD ---
  std::ostringstream line1;
  line1 << "FRM:" << state.stats.protocol.valid_frames
        << " CRC:" << state.stats.protocol.crc_drops
        << " SYNC:" << state.stats.protocol.sync_drops;
  std::ostringstream line2;
  line2 << "ANG0:" << static_cast<int>(state.sweep_angle_by_sensor[0])
        << " ANG1:" << static_cast<int>(state.sweep_angle_by_sensor[1])
        << " ANG2:" << static_cast<int>(state.sweep_angle_by_sensor[2])
        << " CYCLE:" << state.stats.completed_cycles;
  draw_text(frame, 12, 12, line1.str(), kHud, 1);
  draw_text(frame, 12, 24, line2.str(), kHud, 1);

  return frame;
}

} // namespace radar
