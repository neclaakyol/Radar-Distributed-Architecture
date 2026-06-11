#include "radar/raster.hpp"

#include <fstream>
#include <stdexcept>

namespace radar {

std::vector<std::uint8_t> rgba_to_pbm_bits(const RgbaFrame &frame,
                                           std::uint8_t threshold) {
  if (frame.width == 0 || frame.height == 0 ||
      frame.rgba.size() < static_cast<std::size_t>(frame.width) * frame.height * 4U) {
    throw std::invalid_argument("invalid RGBA frame");
  }

  const std::uint32_t row_bytes = (frame.width + 7U) / 8U;
  std::vector<std::uint8_t> bits(static_cast<std::size_t>(row_bytes) *
                                 frame.height);

  for (std::uint32_t y = 0; y < frame.height; ++y) {
    for (std::uint32_t x = 0; x < frame.width; ++x) {
      const std::size_t pixel_index =
          (static_cast<std::size_t>(y) * frame.width + x) * 4U;
      const std::uint8_t r = frame.rgba[pixel_index + 0];
      const std::uint8_t g = frame.rgba[pixel_index + 1];
      const std::uint8_t b = frame.rgba[pixel_index + 2];
      const std::uint16_t luminance =
          static_cast<std::uint16_t>((77U * r + 150U * g + 29U * b) >> 8U);

      if (luminance >= threshold) {
        const std::size_t out_index =
            static_cast<std::size_t>(y) * row_bytes + (x / 8U);
        bits[out_index] |= static_cast<std::uint8_t>(0x80U >> (x % 8U));
      }
    }
  }

  return bits;
}

void write_pbm(const std::filesystem::path &path, const RgbaFrame &frame,
               std::uint8_t threshold) {
  const auto parent = path.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }

  std::ofstream output(path, std::ios::binary);
  if (!output) {
    throw std::runtime_error("failed to open PBM output: " + path.string());
  }

  const std::vector<std::uint8_t> bits = rgba_to_pbm_bits(frame, threshold);
  output << "P4\n" << frame.width << ' ' << frame.height << "\n";
  output.write(reinterpret_cast<const char *>(bits.data()),
               static_cast<std::streamsize>(bits.size()));
}

} // namespace radar
