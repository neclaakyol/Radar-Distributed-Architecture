#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <termios.h>
#include <unistd.h>

// --- Security Keys ---
// Must match the Arduino perfectly
const uint32_t xtea_key[4] = {0x12345678, 0x9ABCDEF0, 0x11223344, 0x55667788};

// --- Helper Functions: XTEA Decrypt & CRC-16 ---
void xtea_decrypt(uint32_t v[2], uint32_t const key[4]) {
  unsigned int i;
  uint32_t v0 = v[0], v1 = v[1], delta = 0x9E3779B9, sum = delta * 32;
  for (i = 0; i < 32; i++) {
    v1 -= (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + key[(sum >> 11) & 3]);
    sum -= delta;
    v0 -= (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + key[sum & 3]);
  }
  v[0] = v0;
  v[1] = v1;
}

uint16_t crc16_ccitt(const uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// --- Serial Port Setup (POSIX termios) ---
int setup_serial(const char *port_name) {
  int fd = open(port_name, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    std::cerr << "Error opening serial port: " << port_name << "\n";
    return -1;
  }

  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    std::cerr << "Error from tcgetattr\n";
    return -1;
  }

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
  tty.c_iflag &= ~IGNBRK;                     // disable break processing
  tty.c_lflag = 0;                            // no signaling chars, no echo,
  tty.c_oflag = 0;                            // no remapping, no delays
  tty.c_cc[VMIN] = 1;                         // read blocks
  tty.c_cc[VTIME] = 5;                        // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem controls, enable reading
  tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS; // Disable hardware flow control

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    std::cerr << "Error from tcsetattr\n";
    return -1;
  }
  return fd;
}

// --- Main Application ---
int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <serial_port> (e.g., /dev/cu.usbmodem14101)\n";
    return 1;
  }

  int serial_fd = setup_serial(argv[1]);
  if (serial_fd < 0)
    return 1;

  std::cout << "Listening on " << argv[1] << " at 115200 baud...\n";
  std::cout << "Waiting for secure telemetry frames...\n";
  std::cout << "--------------------------------------------------\n";

  uint8_t buffer[12];
  int state = 0;
  int bytes_read = 0;

  // Telemetry Statistics
  uint32_t valid_frames = 0;
  uint32_t dropped_frames = 0;

  while (true) {
    uint8_t byte;
    int n = read(serial_fd, &byte, 1);

    if (n > 0) {
      // State Machine for Frame Synchronization
      if (state == 0) {
        if (byte == 0xAA) {
          buffer[0] = byte;
          state = 1;
        }
      } else if (state == 1) {
        if (byte == 0x55) {
          buffer[1] = byte;
          state = 2;
          bytes_read = 2;
        } else {
          state = 0; // Lost sync
        }
      } else if (state == 2) {
        buffer[bytes_read++] = byte;
        if (bytes_read == 12) {
          // We have a full 12-byte frame!

          // 1. Verify CRC-16 on the first 10 bytes
          uint16_t calculated_crc = crc16_ccitt(buffer, 10);
          uint16_t received_crc = (buffer[10] << 8) | buffer[11];

          if (calculated_crc == received_crc) {
            valid_frames++;

            // 2. Extract encrypted payload (Bytes 2 through 9)
            uint32_t payloadBlock[2];
            std::memcpy(payloadBlock, &buffer[2], 8);

            // 3. Decrypt the payload
            xtea_decrypt(payloadBlock, xtea_key);
            uint8_t *decryptedBytes = (uint8_t *)payloadBlock;

            // 4. Parse the original data
            uint8_t angle = decryptedBytes[0];
            uint16_t distance = (decryptedBytes[1] << 8) | decryptedBytes[2];

            std::cout << "[VALID] Angle: " << std::setw(3) << (int)angle
                      << " deg | Distance: " << std::setw(4) << distance
                      << " mm | Acceptance Rate: " << std::fixed
                      << std::setprecision(2)
                      << ((float)valid_frames /
                          (valid_frames + dropped_frames)) *
                             100.0
                      << "%\n";

            // Stage 3 Integration Point:
            // Here is where you will push 'angle' and 'distance' into your
            // std::vector sweep buffers for the Nearest-Neighbor algorithm.

          } else {
            dropped_frames++;
            std::cerr << "[DROP] CRC Failure. Expected: 0x" << std::hex
                      << calculated_crc << " Got: 0x" << received_crc
                      << std::dec << "\n";
          }

          // Reset state for the next frame
          state = 0;
        }
      }
    }
  }

  close(serial_fd);
  return 0;
}