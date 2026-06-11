#include "radar/io.hpp"

#include <stdexcept>

namespace radar {

ReplayByteSource::ReplayByteSource(std::filesystem::path path)
    : path_(std::move(path)), input_(path_, std::ios::binary) {
  if (!input_) {
    throw std::runtime_error("failed to open replay file: " + path_.string());
  }
}

std::optional<std::uint8_t> ReplayByteSource::read_byte() {
  char byte = 0;
  if (!input_.get(byte)) {
    eof_ = true;
    return std::nullopt;
  }
  return static_cast<std::uint8_t>(static_cast<unsigned char>(byte));
}

} // namespace radar
