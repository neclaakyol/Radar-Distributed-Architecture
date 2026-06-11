#include "radar/io.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#ifdef _WIN32
#include <windows.h>
#else
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace radar {

#ifdef _WIN32

struct SerialByteSourceImpl {
  HANDLE handle = INVALID_HANDLE_VALUE;
};

namespace {

std::string normalize_windows_port_name(const std::string &port_name) {
  if (port_name.rfind("\\\\.\\", 0) == 0) {
    return port_name;
  }
  if (port_name.rfind("COM", 0) == 0 && port_name.size() > 4) {
    return "\\\\.\\" + port_name;
  }
  return port_name;
}

} // namespace

SerialByteSource::SerialByteSource(std::string port_name, int baud_rate)
    : port_name_(std::move(port_name)), baud_rate_(baud_rate),
      impl_(std::make_unique<SerialByteSourceImpl>()) {
  const std::string device = normalize_windows_port_name(port_name_);
  impl_->handle = CreateFileA(device.c_str(), GENERIC_READ, 0, nullptr,
                              OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (impl_->handle == INVALID_HANDLE_VALUE) {
    throw std::runtime_error("failed to open serial port: " + port_name_);
  }

  DCB dcb{};
  dcb.DCBlength = sizeof(dcb);
  if (!GetCommState(impl_->handle, &dcb)) {
    throw std::runtime_error("GetCommState failed for: " + port_name_);
  }

  dcb.BaudRate = static_cast<DWORD>(baud_rate_);
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fBinary = TRUE;
  dcb.fDtrControl = DTR_CONTROL_ENABLE;
  dcb.fRtsControl = RTS_CONTROL_ENABLE;

  if (!SetCommState(impl_->handle, &dcb)) {
    throw std::runtime_error("SetCommState failed for: " + port_name_);
  }

  COMMTIMEOUTS timeouts{};
  timeouts.ReadIntervalTimeout = MAXDWORD;
  timeouts.ReadTotalTimeoutConstant = 20;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  if (!SetCommTimeouts(impl_->handle, &timeouts)) {
    throw std::runtime_error("SetCommTimeouts failed for: " + port_name_);
  }
}

SerialByteSource::~SerialByteSource() {
  if (impl_ && impl_->handle != INVALID_HANDLE_VALUE) {
    CloseHandle(impl_->handle);
    impl_->handle = INVALID_HANDLE_VALUE;
  }
}

std::optional<std::uint8_t> SerialByteSource::read_byte() {
  std::uint8_t byte = 0;
  DWORD bytes_read = 0;
  if (!ReadFile(impl_->handle, &byte, 1, &bytes_read, nullptr)) {
    throw std::runtime_error("serial read failed: " + port_name_);
  }
  if (bytes_read == 0) {
    return std::nullopt;
  }
  return byte;
}

#else

struct SerialByteSourceImpl {
  int fd = -1;
};

namespace {

speed_t baud_to_speed(int baud_rate) {
  switch (baud_rate) {
  case 9600:
    return B9600;
  case 19200:
    return B19200;
  case 38400:
    return B38400;
  case 57600:
    return B57600;
  case 115200:
    return B115200;
  default:
    throw std::invalid_argument("unsupported baud rate: " +
                                std::to_string(baud_rate));
  }
}

} // namespace

SerialByteSource::SerialByteSource(std::string port_name, int baud_rate)
    : port_name_(std::move(port_name)), baud_rate_(baud_rate),
      impl_(std::make_unique<SerialByteSourceImpl>()) {
  impl_->fd = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (impl_->fd < 0) {
    throw std::runtime_error("failed to open serial port: " + port_name_ +
                             ": " + std::strerror(errno));
  }

  termios tty{};
  if (tcgetattr(impl_->fd, &tty) != 0) {
    throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(errno)));
  }

  const speed_t speed = baud_to_speed(baud_rate_);
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
  tty.c_cflag &= ~CRTSCTS;
#endif

  if (tcsetattr(impl_->fd, TCSANOW, &tty) != 0) {
    throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(errno)));
  }
}

SerialByteSource::~SerialByteSource() {
  if (impl_ && impl_->fd >= 0) {
    close(impl_->fd);
    impl_->fd = -1;
  }
}

std::optional<std::uint8_t> SerialByteSource::read_byte() {
  std::uint8_t byte = 0;
  const ssize_t n = read(impl_->fd, &byte, 1);
  if (n == 1) {
    return byte;
  }
  if (n == 0 || (n < 0 && errno == EINTR)) {
    return std::nullopt;
  }
  if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    return std::nullopt;
  }
  throw std::runtime_error("serial read failed: " + std::string(std::strerror(errno)));
}

#endif

} // namespace radar
