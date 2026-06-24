#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace radar {

constexpr std::uint8_t kSyncByte1 = 0xAA;
constexpr std::uint8_t kSyncByte2 = 0x55;
constexpr std::size_t kFrameSize = 20;            // 2 sync + 16 encrypted + 2 CRC
constexpr std::size_t kEncryptedPayloadSize = 16; // two 8-byte XTEA blocks
constexpr std::size_t kNumSensors = 3;

// Input descriptor for one sensor reading; used to build a frame.
struct SensorReading {
  std::uint8_t sensor_id = 0;
  std::uint8_t angle_deg = 0;
  std::uint16_t distance_mm = 0;
};

struct TelemetryPoint {
  std::uint8_t angle_deg = 0;
  std::uint16_t distance_mm = 0;
  std::uint64_t sequence = 0;
  std::uint8_t sensor_id = 0; // appended last to keep existing aggregate-init sites valid
};

struct ProtocolCounters {
  std::uint64_t valid_frames = 0;
  std::uint64_t crc_drops = 0;
  std::uint64_t sync_drops = 0;
  std::uint64_t timeout_readings = 0;
};

struct FrameDecodeResult {
  bool valid = false;
  std::array<TelemetryPoint, kNumSensors> points{};
  std::uint16_t calculated_crc = 0;
  std::uint16_t received_crc = 0;
};

std::uint16_t crc16_ccitt(std::span<const std::uint8_t> data);
void xtea_encrypt_block(std::uint32_t block[2]);
void xtea_decrypt_block(std::uint32_t block[2]);

std::array<std::uint8_t, kFrameSize> make_frame(
    const std::array<SensorReading, kNumSensors> &readings,
    std::uint8_t cycle_counter = 0,
    std::uint8_t status_flags = 0);

FrameDecodeResult decode_frame(const std::array<std::uint8_t, kFrameSize> &frame,
                               std::uint64_t base_sequence);

class FrameParser {
public:
  std::vector<TelemetryPoint> ingest(std::uint8_t byte);
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
