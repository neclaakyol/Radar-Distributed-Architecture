#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

namespace radar {

struct RgbaFrame {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::vector<std::uint8_t> rgba;
};

std::vector<std::uint8_t> rgba_to_pbm_bits(const RgbaFrame &frame,
                                           std::uint8_t threshold);
void write_pbm(const std::filesystem::path &path, const RgbaFrame &frame,
               std::uint8_t threshold);

} // namespace radar
