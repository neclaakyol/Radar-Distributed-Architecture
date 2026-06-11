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

std::array<std::uint8_t, kFrameSize> make_frame(std::uint8_t angle_deg,
                                                std::uint16_t distance_mm) {
  std::array<std::uint8_t, kFrameSize> frame{};
  std::array<std::uint8_t, kEncryptedPayloadSize> payload{};

  payload[0] = angle_deg;
  payload[1] = static_cast<std::uint8_t>((distance_mm >> 8) & 0xFFU);
  payload[2] = static_cast<std::uint8_t>(distance_mm & 0xFFU);

  std::uint32_t block[2] = {
      load_le32(payload.data()),
      load_le32(payload.data() + 4),
  };
  xtea_encrypt_block(block);
  store_le32(payload.data(), block[0]);
  store_le32(payload.data() + 4, block[1]);

  frame[0] = kSyncByte1;
  frame[1] = kSyncByte2;
  std::copy(payload.begin(), payload.end(), frame.begin() + 2);

  const std::uint16_t crc = crc16_ccitt(std::span<const std::uint8_t>(frame.data(), 10));
  frame[10] = static_cast<std::uint8_t>((crc >> 8) & 0xFFU);
  frame[11] = static_cast<std::uint8_t>(crc & 0xFFU);
  return frame;
}

FrameDecodeResult decode_frame(const std::array<std::uint8_t, kFrameSize> &frame,
                               std::uint64_t sequence) {
  FrameDecodeResult result;
  result.calculated_crc =
      crc16_ccitt(std::span<const std::uint8_t>(frame.data(), 10));
  result.received_crc =
      static_cast<std::uint16_t>((static_cast<std::uint16_t>(frame[10]) << 8) |
                                 frame[11]);

  if (frame[0] != kSyncByte1 || frame[1] != kSyncByte2 ||
      result.calculated_crc != result.received_crc) {
    return result;
  }

  std::array<std::uint8_t, kEncryptedPayloadSize> payload{};
  std::copy(frame.begin() + 2, frame.begin() + 10, payload.begin());

  std::uint32_t block[2] = {
      load_le32(payload.data()),
      load_le32(payload.data() + 4),
  };
  xtea_decrypt_block(block);
  store_le32(payload.data(), block[0]);
  store_le32(payload.data() + 4, block[1]);

  result.valid = true;
  result.point.angle_deg = payload[0];
  result.point.distance_mm =
      static_cast<std::uint16_t>((static_cast<std::uint16_t>(payload[1]) << 8) |
                                 payload[2]);
  result.point.sequence = sequence;
  return result;
}

std::optional<TelemetryPoint> FrameParser::ingest(std::uint8_t byte) {
  switch (state_) {
  case State::WaitSync1:
    if (byte == kSyncByte1) {
      buffer_[0] = byte;
      bytes_read_ = 1;
      state_ = State::WaitSync2;
    } else {
      ++counters_.sync_drops;
    }
    return std::nullopt;

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
    return std::nullopt;

  case State::ReadFrame:
    buffer_[bytes_read_++] = byte;
    if (bytes_read_ < kFrameSize) {
      return std::nullopt;
    }

    state_ = State::WaitSync1;
    bytes_read_ = 0;

    {
      FrameDecodeResult decoded = decode_frame(buffer_, next_sequence_);
      if (!decoded.valid) {
        ++counters_.crc_drops;
        return std::nullopt;
      }

      ++next_sequence_;
      ++counters_.valid_frames;
      if (decoded.point.distance_mm == 0) {
        ++counters_.timeout_readings;
      }
      return decoded.point;
    }
  }

  return std::nullopt;
}

void FrameParser::reset() {
  state_ = State::WaitSync1;
  buffer_.fill(0);
  bytes_read_ = 0;
  next_sequence_ = 0;
  counters_ = {};
}

} // namespace radar
