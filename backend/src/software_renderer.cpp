#include "radar/render.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
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

Color phosphor(std::uint8_t intensity) {
  return Color{static_cast<std::uint8_t>(intensity / 4U), intensity,
               static_cast<std::uint8_t>(intensity / 3U), 255};
}

void put_pixel(RgbaFrame &frame, int x, int y, Color color) {
  if (x < 0 || y < 0 || x >= static_cast<int>(frame.width) ||
      y >= static_cast<int>(frame.height)) {
    return;
  }

  const std::size_t offset =
      (static_cast<std::size_t>(y) * frame.width + static_cast<std::size_t>(x)) *
      4U;
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

void draw_filled_circle(RgbaFrame &frame, Point2 center, int radius,
                        Color color) {
  for (int y = -radius; y <= radius; ++y) {
    for (int x = -radius; x <= radius; ++x) {
      if (x * x + y * y <= radius * radius) {
        put_pixel(frame, center.x + x, center.y + y, color);
      }
    }
  }
}

void draw_arc(RgbaFrame &frame, Point2 center, int radius, Color color) {
  for (int angle = 0; angle <= 180; ++angle) {
    const double radians = static_cast<double>(angle) * kPi / 180.0;
    const int x = center.x + static_cast<int>(std::round(std::cos(radians) * radius));
    const int y = center.y - static_cast<int>(std::round(std::sin(radians) * radius));
    put_pixel(frame, x, y, color);
  }
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

void draw_text(RgbaFrame &frame, int x, int y, std::string text, Color color,
               int scale = 1) {
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
            put_pixel(frame, cursor_x + col * scale + sx,
                      y + row * scale + sy, color);
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
    max_distance =
        std::max(max_distance, vector.current.source.distance_mm);
    max_distance =
        std::max(max_distance, vector.previous.source.distance_mm);
  }
  return static_cast<double>(max_distance) * 1.1;
}

Point2 project(Point2 center, int radius, const RadarPoint &point,
               double max_range_mm) {
  const double range_fraction =
      std::clamp(static_cast<double>(point.distance_mm) / max_range_mm, 0.0, 1.0);
  const double radians = static_cast<double>(point.angle_deg) * kPi / 180.0;
  const double screen_radius = range_fraction * static_cast<double>(radius);
  return Point2{center.x + static_cast<int>(std::round(std::cos(radians) * screen_radius)),
                center.y - static_cast<int>(std::round(std::sin(radians) * screen_radius))};
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

} // namespace

SoftwareRenderer::SoftwareRenderer(std::uint32_t width, std::uint32_t height)
    : width_(width), height_(height) {}

RgbaFrame SoftwareRenderer::render(const RenderState &state) const {
  RgbaFrame frame;
  frame.width = width_;
  frame.height = height_;
  frame.rgba.assign(static_cast<std::size_t>(width_) * height_ * 4U, 255);

  for (std::uint32_t y = 0; y < height_; ++y) {
    for (std::uint32_t x = 0; x < width_; ++x) {
      const std::size_t offset =
          (static_cast<std::size_t>(y) * width_ + x) * 4U;
      frame.rgba[offset + 0] = 0;
      frame.rgba[offset + 1] = static_cast<std::uint8_t>(6 + (y % 5));
      frame.rgba[offset + 2] = 4;
      frame.rgba[offset + 3] = 255;
    }
  }

  const Point2 center{static_cast<int>(width_ / 2U),
                      static_cast<int>(height_) - 28};
  const int radius =
      static_cast<int>(std::min(width_ / 2U - 24U, height_ - 72U));
  const double max_range = max_display_range(state);

  for (int ring = 1; ring <= 4; ++ring) {
    draw_arc(frame, center, radius * ring / 4, phosphor(45));
  }
  draw_line(frame, Point2{center.x - radius, center.y},
            Point2{center.x + radius, center.y}, phosphor(40));

  for (int angle = 0; angle <= 180; angle += 30) {
    const double radians = static_cast<double>(angle) * kPi / 180.0;
    const Point2 end{
        center.x + static_cast<int>(std::round(std::cos(radians) * radius)),
        center.y - static_cast<int>(std::round(std::sin(radians) * radius))};
    draw_line(frame, center, end, phosphor(38));
  }

  for (const RadarPoint &point : state.history_points) {
    if (point.distance_mm == 0) {
      continue;
    }
    draw_filled_circle(frame, project(center, radius, point, max_range), 2,
                       phosphor(70));
  }

  for (const RadarPoint &point : state.current_points) {
    if (point.distance_mm == 0) {
      continue;
    }
    Point2 p = project(center, radius, point, max_range);
    draw_filled_circle(frame, p, 3, phosphor(210));
    draw_filled_circle(frame, p, 1, phosphor(255));
  }

  for (const MotionVector &vector : state.vectors) {
    const Point2 from =
        project(center, radius, vector.previous.source, max_range);
    const Point2 to = project(center, radius, vector.current.source, max_range);
    draw_arrow(frame, from, to, phosphor(230));
  }

  RadarPoint sweep_point;
  sweep_point.angle_deg = state.sweep_angle_deg;
  sweep_point.distance_mm = static_cast<std::uint16_t>(max_range);
  draw_line(frame, center, project(center, radius, sweep_point, max_range),
            phosphor(160));

  std::ostringstream line1;
  line1 << "SRC:" << state.source_name;
  std::ostringstream line2;
  line2 << "FRM:" << state.stats.protocol.valid_frames
        << " CRC:" << state.stats.protocol.crc_drops
        << " SYNC:" << state.stats.protocol.sync_drops;
  std::ostringstream line3;
  line3 << "TIMEOUT:" << state.stats.protocol.timeout_readings
        << " SWEEP:" << state.stats.completed_sweeps
        << " CYCLE:" << state.stats.completed_cycles;
  std::ostringstream line4;
  line4 << "VEC:" << state.stats.vector_count
        << " ANG:" << static_cast<int>(state.sweep_angle_deg);

  draw_text(frame, 12, 12, line1.str(), phosphor(135), 1);
  draw_text(frame, 12, 24, line2.str(), phosphor(135), 1);
  draw_text(frame, 12, 36, line3.str(), phosphor(135), 1);
  draw_text(frame, 12, 48, line4.str(), phosphor(135), 1);

  return frame;
}

} // namespace radar
