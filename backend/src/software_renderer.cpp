#include "radar/geometry.hpp"
#include "radar/render.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace radar {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

// How long a detection lingers on the map before fading to nothing. Tuned to
// roughly one sweep so the live arc leaves a fading trail behind it.
constexpr double kFadeWindowMs = 3000.0;

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
constexpr Color kPlaneBorder{170, 174, 190, 255};  // 80x140 cm workspace outline
constexpr Color kGrid{20, 30, 26, 255};            // 10 cm reference grid
constexpr Color kSensor{235, 60, 60, 255};         // sensor markers (red)
constexpr Color kSensorRing{255, 120, 120, 255};
constexpr Color kCoverage{34, 52, 60, 255};        // per-sensor coverage wedge
constexpr Color kRay{236, 238, 248, 255};          // live scan ray (white)
constexpr Color kDotConfirmed{255, 255, 255, 255}; // confirmed-detection ring
constexpr Color kArrow{170, 240, 120, 255};        // MTI motion arrow
constexpr Color kLabel{150, 158, 165, 255};
constexpr Color kHud{120, 150, 130, 255};
constexpr Color kDistanceLabel{230, 222, 140, 255}; // per-object distance-to-main (cm)

// Per-sensor detection colors, indexed by sensor_id
// (0 = main HY-SRF05, 1 = left flank, 2 = right flank).
constexpr std::array<Color, kNumSensors> kSensorColor{{
    {60, 230, 95, 255},   // main: green
    {70, 200, 235, 255},  // left flank: cyan
    {235, 120, 235, 255}, // right flank: magenta
}};

const std::array<const char *, kNumSensors> kSensorLabel{{
    "S0 HY-SRF05",
    "S1 HC-SR04",
    "S2 HC-SR04",
}};

Color dim(Color c, double f) {
  return Color{static_cast<std::uint8_t>(c.r * f), static_cast<std::uint8_t>(c.g * f),
               static_cast<std::uint8_t>(c.b * f), c.a};
}

// Maps world millimetres (origin = main sensor, +y into the scene) onto the
// framebuffer with a single uniform scale (so the map is to-scale, no
// distortion). The drawn world window covers the 80x140 cm plane plus a small
// margin so the sensor row near y=0 isn't flush against the edge.
struct Viewport {
  double scale = 1.0;   // px per mm
  double origin_sx = 0; // screen x of world (0,0)
  double origin_sy = 0; // screen y of world (0,0)

  Point2 to_screen(double wx, double wy) const {
    return Point2{static_cast<int>(std::lround(origin_sx + wx * scale)),
                  static_cast<int>(std::lround(origin_sy - wy * scale))};
  }
};

Viewport make_viewport(std::uint32_t w, std::uint32_t h) {
  const double world_x_min = -kPlaneWidthMm / 2.0 - 80.0;
  const double world_x_max = kPlaneWidthMm / 2.0 + 80.0;
  const double world_y_min = -140.0;             // a little below the sensor row
  const double world_y_max = kPlaneDepthMm + 90.0;
  const double margin = 14.0;

  const double world_w = world_x_max - world_x_min;
  const double world_h = world_y_max - world_y_min;
  const double scale =
      std::min((w - 2.0 * margin) / world_w, (h - 2.0 * margin) / world_h);

  // Center the used area in the frame.
  const double off_x = (w - world_w * scale) / 2.0;
  const double off_y = (h - world_h * scale) / 2.0;

  Viewport vp;
  vp.scale = scale;
  vp.origin_sx = off_x + (0.0 - world_x_min) * scale;
  vp.origin_sy = off_y + (world_y_max - 0.0) * scale;
  return vp;
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

// World direction (unit vector) a sensor points at for a given local angle.
std::pair<double, double> world_dir(const SensorMount &mount, double local_angle_deg) {
  const double rad =
      (mount.angle_sign * local_angle_deg + mount.heading_offset_deg) * kPi / 180.0;
  return {std::cos(rad), std::sin(rad)};
}

// Faint outline of a sensor's coverage wedge: the two boundary rays plus a
// sampled arc at max range, so the operator can see where each sensor looks.
void draw_coverage(RgbaFrame &frame, const Viewport &vp, const SensorMount &mount) {
  const auto edge = [&](double local) {
    const auto [dx, dy] = world_dir(mount, local);
    return vp.to_screen(mount.offset_x_mm + mount.max_range_mm * dx,
                        mount.offset_y_mm + mount.max_range_mm * dy);
  };
  const Point2 origin = vp.to_screen(mount.offset_x_mm, mount.offset_y_mm);
  draw_line(frame, origin, edge(mount.sweep_min_deg), kCoverage);
  draw_line(frame, origin, edge(mount.sweep_max_deg), kCoverage);

  constexpr int steps = 48;
  Point2 prev = edge(mount.sweep_min_deg);
  for (int i = 1; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    const Point2 cur =
        edge(mount.sweep_min_deg + t * (mount.sweep_max_deg - mount.sweep_min_deg));
    draw_line(frame, prev, cur, kCoverage);
    prev = cur;
  }
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

  const Viewport vp = make_viewport(width_, height_);
  const SensorGeometry &geometry = state.geometry;
  const double half_w = kPlaneWidthMm / 2.0;

  // --- 10 cm reference grid inside the plane ---
  for (double x = -half_w; x <= half_w + 1.0; x += 100.0) {
    draw_line(frame, vp.to_screen(x, 0.0), vp.to_screen(x, kPlaneDepthMm), kGrid);
  }
  for (double y = 0.0; y <= kPlaneDepthMm + 1.0; y += 100.0) {
    draw_line(frame, vp.to_screen(-half_w, y), vp.to_screen(half_w, y), kGrid);
  }

  // --- 80 x 140 cm workspace outline ---
  const Point2 bl = vp.to_screen(-half_w, 0.0);
  const Point2 br = vp.to_screen(half_w, 0.0);
  const Point2 tl = vp.to_screen(-half_w, kPlaneDepthMm);
  const Point2 tr = vp.to_screen(half_w, kPlaneDepthMm);
  draw_line_thick(frame, bl, br, kPlaneBorder);
  draw_line_thick(frame, br, tr, kPlaneBorder);
  draw_line_thick(frame, tr, tl, kPlaneBorder);
  draw_line_thick(frame, tl, bl, kPlaneBorder);

  // --- Per-sensor coverage wedges (drawn first, faint, behind everything) ---
  for (std::uint8_t sid = 0; sid < kNumSensors; ++sid) {
    draw_coverage(frame, vp, geometry.mount_for(sid));
  }

  // --- Live scan ray (white) at each sensor's current angle ---
  for (std::uint8_t sid = 0; sid < kNumSensors; ++sid) {
    const SensorMount &mount = geometry.mount_for(sid);
    const auto [dx, dy] = world_dir(mount, state.sweep_angle_by_sensor[sid]);
    const Point2 origin = vp.to_screen(mount.offset_x_mm, mount.offset_y_mm);
    const Point2 tip = vp.to_screen(mount.offset_x_mm + mount.max_range_mm * dx,
                                    mount.offset_y_mm + mount.max_range_mm * dy);
    draw_line_thick(frame, origin, tip, kRay);
  }

  // --- Detections, with a time-based fade trail (newest bright, older fade
  //     out to nothing) and a per-object distance-to-main readout. ---
  const auto now = RadarClock::now();
  auto fade_for = [&](RadarClock::time_point ts) {
    const double age_ms = std::chrono::duration<double, std::milli>(now - ts).count();
    return std::clamp(1.0 - age_ms / kFadeWindowMs, 0.0, 1.0);
  };

  // Draws one detection at its world position; brightness falls off purely
  // with age, so a blip is brightest when freshly seen and fades to nothing.
  // Returns the fade factor (or -1 if the point was skipped).
  auto draw_detection = [&](const RadarPoint &point) -> double {
    if (point.distance_mm == 0 || point.sensor_id >= kNumSensors) {
      return -1.0;
    }
    const double f = fade_for(point.timestamp);
    if (f <= 0.02) {
      return -1.0;
    }
    const WorldPoint w = to_world(point, geometry);
    const Point2 p = vp.to_screen(w.world_x_mm, w.world_y_mm);
    if (point.confirmation == Confirmation::Confirmed) {
      draw_ring(frame, p, 5, dim(kDotConfirmed, f));
    }
    draw_filled_circle(frame, p, f > 0.5 ? 3 : 2, dim(kSensorColor[point.sensor_id], f));
    return f;
  };

  for (const RadarPoint &point : state.history_points) {
    draw_detection(point);
  }

  // Live sweep also feeds the distance-to-main readout: each still-bright
  // object gets a distance label (cm). Labels are decimated by screen distance
  // so a cluster reads as a single number; the closest shows in the HUD too.
  std::vector<Point2> labeled;
  double nearest_to_main_mm = -1.0;
  auto labeled_near = [&](Point2 p) {
    for (const Point2 &q : labeled) {
      const int dx = p.x - q.x;
      const int dy = p.y - q.y;
      if (dx * dx + dy * dy < 34 * 34) {
        return true;
      }
    }
    return false;
  };

  for (const RadarPoint &point : state.current_points) {
    const double f = draw_detection(point);
    if (f < 0.0) {
      continue;
    }
    // Main sensor is the world origin, so distance-to-main is the world radius.
    const WorldPoint w = to_world(point, geometry);
    const double dist_to_main_mm = std::hypot(w.world_x_mm, w.world_y_mm);
    if (nearest_to_main_mm < 0.0 || dist_to_main_mm < nearest_to_main_mm) {
      nearest_to_main_mm = dist_to_main_mm;
    }
    if (f < 0.4) { // too faded to label clearly
      continue;
    }
    const Point2 p = vp.to_screen(w.world_x_mm, w.world_y_mm);
    if (!labeled_near(p)) {
      labeled.push_back(p);
      draw_text(frame, p.x + 6, p.y - 3,
                std::to_string(std::lround(dist_to_main_mm / 10.0)), kDistanceLabel, 1);
    }
  }

  // --- MTI motion arrows (object movement between sweeps) ---
  for (const MotionVector &vector : state.vectors) {
    if (vector.current.source.sensor_id >= kNumSensors) {
      continue;
    }
    const WorldPoint from_w = to_world(vector.previous.source, geometry);
    const WorldPoint to_w = to_world(vector.current.source, geometry);
    draw_arrow(frame, vp.to_screen(from_w.world_x_mm, from_w.world_y_mm),
               vp.to_screen(to_w.world_x_mm, to_w.world_y_mm), kArrow);
  }

  // --- Sensor markers (red) on top, with labels ---
  for (std::uint8_t sid = 0; sid < kNumSensors; ++sid) {
    const SensorMount &mount = geometry.mount_for(sid);
    const Point2 marker = vp.to_screen(mount.offset_x_mm, mount.offset_y_mm);
    draw_filled_circle(frame, marker, 5, kSensor);
    draw_ring(frame, marker, 8, kSensorRing);
    const std::string name = kSensorLabel[sid];
    draw_text(frame, marker.x - text_width(name, 1) / 2, marker.y + 14, name, kLabel, 1);
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
  if (nearest_to_main_mm >= 0.0) {
    std::ostringstream line3;
    line3 << "NEAREST TO MAIN: " << std::lround(nearest_to_main_mm / 10.0) << " CM";
    draw_text(frame, 12, 36, line3.str(), kDistanceLabel, 1);
  }
  draw_text(frame, 12, height_ - 16, "PLANE 80X140 CM  GRID 10 CM", kLabel, 1);

  return frame;
}

} // namespace radar
