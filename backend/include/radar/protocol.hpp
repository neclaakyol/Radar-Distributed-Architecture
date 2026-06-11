#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace radar {

constexpr std::uint8_t kSyncByte1 = 0xAA;
constexpr std::uint8_t kSyncByte2 = 0x55;
constexpr std::size_t kFrameSize = 12;
constexpr std::size_t kEncryptedPayloadSize = 8;

struct TelemetryPoint {
  std::uint8_t angle_deg = 0;
  std::uint16_t distance_mm = 0;
  std::uint64_t sequence = 0;
};

struct ProtocolCounters {
  std::uint64_t valid_frames = 0;
  std::uint64_t crc_drops = 0;
  std::uint64_t sync_drops = 0;
  std::uint64_t timeout_readings = 0;
};

struct FrameDecodeResult {
  bool valid = false;
  TelemetryPoint point;
  std::uint16_t calculated_crc = 0;
  std::uint16_t received_crc = 0;
};

std::uint16_t crc16_ccitt(std::span<const std::uint8_t> data);
void xtea_encrypt_block(std::uint32_t block[2]);
void xtea_decrypt_block(std::uint32_t block[2]);

std::array<std::uint8_t, kFrameSize> make_frame(std::uint8_t angle_deg,
                                                std::uint16_t distance_mm);
FrameDecodeResult decode_frame(const std::array<std::uint8_t, kFrameSize> &frame,
                               std::uint64_t sequence);

class FrameParser {
public:
  std::optional<TelemetryPoint> ingest(std::uint8_t byte);
  const ProtocolCounters &counters() const { return counters_; }
  void reset();

private:
  enum class State {
    WaitSync1,
    WaitSync2,
    ReadFrame,
  };

  State state_ = State::WaitSync1;
  std::array<std::uint8_t, kFrameSize> buffer_{};
  std::size_t bytes_read_ = 0;
  std::uint64_t next_sequence_ = 0;
  ProtocolCounters counters_;
};

} // namespace radar
