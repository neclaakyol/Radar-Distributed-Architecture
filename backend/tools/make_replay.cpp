#include "radar/protocol.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

std::uint16_t forward_distance(int angle, int cycle) {
  return static_cast<std::uint16_t>(700 + (angle % 23) + cycle * 10);
}

std::uint16_t return_distance(int angle, int cycle) {
  return static_cast<std::uint16_t>(760 + (angle % 23) + cycle * 10);
}

void write_frame(std::ofstream &output, std::uint8_t angle,
                 std::uint16_t distance) {
  const auto frame = radar::make_frame(angle, distance);
  output.write(reinterpret_cast<const char *>(frame.data()),
               static_cast<std::streamsize>(frame.size()));
}

} // namespace

int main(int argc, char **argv) {
  try {
    if (argc < 2 || argc > 3) {
      std::cerr << "Usage: " << argv[0] << " OUTPUT.bin [cycles]\n";
      return 2;
    }

    const std::filesystem::path output_path = argv[1];
    const int cycles = argc == 3 ? std::stoi(argv[2]) : 2;
    if (cycles <= 0) {
      throw std::invalid_argument("cycles must be positive");
    }

    const auto parent = output_path.parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent);
    }

    std::ofstream output(output_path, std::ios::binary);
    if (!output) {
      throw std::runtime_error("failed to open output: " + output_path.string());
    }

    for (int cycle = 0; cycle < cycles; ++cycle) {
      for (int angle = 0; angle <= 180; ++angle) {
        write_frame(output, static_cast<std::uint8_t>(angle),
                    forward_distance(angle, cycle));
      }
      for (int angle = 179; angle >= 0; --angle) {
        write_frame(output, static_cast<std::uint8_t>(angle),
                    return_distance(angle, cycle));
      }
    }

    std::cout << "Wrote replay fixture: " << output_path.string() << "\n";
    return 0;
  } catch (const std::exception &ex) {
    std::cerr << "Error: " << ex.what() << "\n";
    return 1;
  }
}
