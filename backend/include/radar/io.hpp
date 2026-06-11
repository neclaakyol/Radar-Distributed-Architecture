#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>

namespace radar {

class ByteSource {
public:
  virtual ~ByteSource() = default;
  virtual std::optional<std::uint8_t> read_byte() = 0;
  virtual bool eof() const = 0;
  virtual std::string description() const = 0;
};

class ReplayByteSource final : public ByteSource {
public:
  explicit ReplayByteSource(std::filesystem::path path);

  std::optional<std::uint8_t> read_byte() override;
  bool eof() const override { return eof_; }
  std::string description() const override { return path_.string(); }

private:
  std::filesystem::path path_;
  std::ifstream input_;
  bool eof_ = false;
};

struct SerialByteSourceImpl;

class SerialByteSource final : public ByteSource {
public:
  SerialByteSource(std::string port_name, int baud_rate);
  ~SerialByteSource() override;

  SerialByteSource(const SerialByteSource &) = delete;
  SerialByteSource &operator=(const SerialByteSource &) = delete;

  std::optional<std::uint8_t> read_byte() override;
  bool eof() const override { return false; }
  std::string description() const override { return port_name_; }

private:
  std::string port_name_;
  int baud_rate_ = 115200;
  std::unique_ptr<SerialByteSourceImpl> impl_;
};

} // namespace radar
