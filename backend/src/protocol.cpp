#include "radar/protocol.hpp"

#include <algorithm>
#include <cstring>

namespace radar {
namespace {

constexpr std::uint32_t kXteaKey[4] = {
    0x12345678U,
    0x9ABCDEF0U,
    0x11223344U,
    0x55667788U,
};

std::uint32_t load_le32(const std::uint8_t *bytes) {
  return static_cast<std::uint32_t>(bytes[0]) |
         (static_cast<std::uint32_t>(bytes[1]) << 8) |
         (static_cast<std::uint32_t>(bytes[2]) << 16) |
         (static_cast<std::uint32_t>(bytes[3]) << 24);
}

void store_le32(std::uint8_t *bytes, std::uint32_t value) {
  bytes[0] = static_cast<std::uint8_t>(value & 0xFFU);
  bytes[1] = static_cast<std::uint8_t>((value >> 8) & 0xFFU);
  bytes[2] = static_cast<std::uint8_t>((value >> 16) & 0xFFU);
  bytes[3] = static_cast<std::uint8_t>((value >> 24) & 0xFFU);
}

} // namespace

std::uint16_t crc16_ccitt(std::span<const std::uint8_t> data) {
  std::uint16_t crc = 0xFFFFU;
  for (std::uint8_t byte : data) {
    crc ^= static_cast<std::uint16_t>(byte) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000U) != 0) {
        crc = static_cast<std::uint16_t>((crc << 1) ^ 0x1021U);
      } else {
        crc = static_cast<std::uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

void xtea_encrypt_block(std::uint32_t block[2]) {
  std::uint32_t v0 = block[0];
  std::uint32_t v1 = block[1];
  std::uint32_t sum = 0;
  constexpr std::uint32_t delta = 0x9E3779B9U;

  for (int round = 0; round < 32; ++round) {
    v0 += (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + kXteaKey[sum & 3U]);
    sum += delta;
    v1 += (((v0 << 4) ^ (v0 >> 5)) + v0) ^
          (sum + kXteaKey[(sum >> 11) & 3U]);
  }

  block[0] = v0;
  block[1] = v1;
}

void xtea_decrypt_block(std::uint32_t block[2]) {
  std::uint32_t v0 = block[0];
  std::uint32_t v1 = block[1];
  constexpr std::uint32_t delta = 0x9E3779B9U;
  std::uint32_t sum = delta * 32U;

  for (int round = 0; round < 32; ++round) {
    v1 -= (((v0 << 4) ^ (v0 >> 5)) + v0) ^
          (sum + kXteaKey[(sum >> 11) & 3U]);
    sum -= delta;
    v0 -= (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + kXteaKey[sum & 3U]);
  }

  block[0] = v0;
  block[1] = v1;
}

// Plaintext payload layout (16 bytes):
//   [0]  sensor_id 0   [1] angle 0    [2-3]  dist 0 (hi,lo)
//   [4]  sensor_id 1   [5] angle 1    [6-7]  dist 1 (hi,lo)
//   [8]  sensor_id 2   [9] angle 2    [10-11] dist 2 (hi,lo)
//   [12] cycle_counter [13] status_flags  [14-15] reserved
std::array<std::uint8_t, kFrameSize> make_frame(
    const std::array<SensorReading, kNumSensors> &readings,
    std::uint8_t cycle_counter,
    std::uint8_t status_flags) {

  std::array<std::uint8_t, kFrameSize> frame{};
  std::array<std::uint8_t, kEncryptedPayloadSize> payload{};

  for (std::size_t i = 0; i < kNumSensors; ++i) {
    const std::size_t off = i * 4;
    payload[off]     = readings[i].sensor_id;
    payload[off + 1] = readings[i].angle_deg;
    payload[off + 2] = static_cast<std::uint8_t>((readings[i].distance_mm >> 8) & 0xFFU);
    payload[off + 3] = static_cast<std::uint8_t>(readings[i].distance_mm & 0xFFU);
  }
  payload[12] = cycle_counter;
  payload[13] = status_flags;
  // payload[14] and payload[15] stay zero (reserved)

  // Encrypt two consecutive 8-byte XTEA blocks (ECB, same key)
  std::uint32_t block1[2] = {load_le32(payload.data()),     load_le32(payload.data() + 4)};
  xtea_encrypt_block(block1);
  store_le32(payload.data(),     block1[0]);
  store_le32(payload.data() + 4, block1[1]);

  std::uint32_t block2[2] = {load_le32(payload.data() + 8), load_le32(payload.data() + 12)};
  xtea_encrypt_block(block2);
  store_le32(payload.data() + 8,  block2[0]);
  store_le32(payload.data() + 12, block2[1]);

  frame[0] = kSyncByte1;
  frame[1] = kSyncByte2;
  std::copy(payload.begin(), payload.end(), frame.begin() + 2);

  const std::uint16_t crc =
      crc16_ccitt(std::span<const std::uint8_t>(frame.data(), kFrameSize - 2));
  frame[kFrameSize - 2] = static_cast<std::uint8_t>((crc >> 8) & 0xFFU);
  frame[kFrameSize - 1] = static_cast<std::uint8_t>(crc & 0xFFU);
  return frame;
}

FrameDecodeResult decode_frame(const std::array<std::uint8_t, kFrameSize> &frame,
                               std::uint64_t base_sequence) {
  FrameDecodeResult result;
  result.calculated_crc =
      crc16_ccitt(std::span<const std::uint8_t>(frame.data(), kFrameSize - 2));
  result.received_crc =
      static_cast<std::uint16_t>(
          (static_cast<std::uint16_t>(frame[kFrameSize - 2]) << 8) | frame[kFrameSize - 1]);

  if (frame[0] != kSyncByte1 || frame[1] != kSyncByte2 ||
      result.calculated_crc != result.received_crc) {
    return result;
  }

  std::array<std::uint8_t, kEncryptedPayloadSize> payload{};
  std::copy(frame.begin() + 2, frame.begin() + 2 + kEncryptedPayloadSize, payload.begin());

  std::uint32_t block1[2] = {load_le32(payload.data()),     load_le32(payload.data() + 4)};
  xtea_decrypt_block(block1);
  store_le32(payload.data(),     block1[0]);
  store_le32(payload.data() + 4, block1[1]);

  std::uint32_t block2[2] = {load_le32(payload.data() + 8), load_le32(payload.data() + 12)};
  xtea_decrypt_block(block2);
  store_le32(payload.data() + 8,  block2[0]);
  store_le32(payload.data() + 12, block2[1]);

  for (std::size_t i = 0; i < kNumSensors; ++i) {
    const std::size_t off = i * 4;
    result.points[i].sensor_id   = payload[off];
    result.points[i].angle_deg   = payload[off + 1];
    result.points[i].distance_mm = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(payload[off + 2]) << 8) | payload[off + 3]);
    result.points[i].sequence = base_sequence + i;
  }

  result.valid = true;
  return result;
}

std::vector<TelemetryPoint> FrameParser::ingest(std::uint8_t byte) {
  switch (state_) {
  case State::WaitSync1:
    if (byte == kSyncByte1) {
      buffer_[0] = byte;
      bytes_read_ = 1;
      state_ = State::WaitSync2;
    } else {
      ++counters_.sync_drops;
    }
    return {};

  case State::WaitSync2:
    if (byte == kSyncByte2) {
      buffer_[1] = byte;
      bytes_read_ = 2;
      state_ = State::ReadFrame;
    } else {
      ++counters_.sync_drops;
      if (byte == kSyncByte1) {
        buffer_[0] = byte;
        bytes_read_ = 1;
      } else {
        bytes_read_ = 0;
        state_ = State::WaitSync1;
      }
    }
    return {};

  case State::ReadFrame:
    buffer_[bytes_read_++] = byte;
    if (bytes_read_ < kFrameSize) {
      return {};
    }

    state_ = State::WaitSync1;
    bytes_read_ = 0;

    {
      FrameDecodeResult decoded = decode_frame(buffer_, next_sequence_);
      if (!decoded.valid) {
        ++counters_.crc_drops;
        return {};
      }

      next_sequence_ += kNumSensors;
      ++counters_.valid_frames;
      for (const auto &p : decoded.points) {
        if (p.distance_mm == 0) {
          ++counters_.timeout_readings;
        }
      }
      return std::vector<TelemetryPoint>(decoded.points.begin(), decoded.points.end());
    }
  }

  return {};
}

void FrameParser::reset() {
  state_ = State::WaitSync1;
  buffer_.fill(0);
  bytes_read_ = 0;
  next_sequence_ = 0;
  counters_ = {};
}

} // namespace radar
