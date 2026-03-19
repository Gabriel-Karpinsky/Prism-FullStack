#include "serial_transport.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <thread>

#ifdef __linux__
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace edge {
namespace {

#ifdef __linux__
speed_t SelectBaud(int baud) {
  switch (baud) {
    case 115200:
      return B115200;
    case 57600:
      return B57600;
    case 38400:
      return B38400;
    default:
      return B9600;
  }
}
#endif

}  // namespace

SerialTransport::SerialTransport(std::string port, int baud)
    : port_(std::move(port)), baud_(baud) {}

SerialTransport::~SerialTransport() { Close(); }

bool SerialTransport::Open() {
#ifdef __linux__
  fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) {
    last_error_ = std::strerror(errno);
    is_open_ = false;
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    last_error_ = std::strerror(errno);
    Close();
    return false;
  }

  const speed_t speed = SelectBaud(baud_);
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
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    last_error_ = std::strerror(errno);
    Close();
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  is_open_ = true;
  last_error_.clear();
  return true;
#else
  last_error_ = "serial transport is only implemented for Linux hosts";
  is_open_ = false;
  return false;
#endif
}

void SerialTransport::Close() {
#ifdef __linux__
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
#endif
  is_open_ = false;
}

void SerialTransport::DrainInput() {
#ifdef __linux__
  if (fd_ >= 0) {
    tcflush(fd_, TCIFLUSH);
  }
#endif
}

bool SerialTransport::SendLine(const std::string& line) {
#ifdef __linux__
  if (!is_open_) {
    last_error_ = "serial port is not open";
    return false;
  }

  const std::string payload = line + "\n";
  const ssize_t written = write(fd_, payload.data(), payload.size());
  if (written < 0 || static_cast<size_t>(written) != payload.size()) {
    last_error_ = std::strerror(errno);
    return false;
  }

  tcdrain(fd_);
  last_error_.clear();
  return true;
#else
  (void)line;
  last_error_ = "serial transport is only implemented for Linux hosts";
  return false;
#endif
}

bool SerialTransport::ReadLine(std::string& line, int timeoutMs) {
#ifdef __linux__
  if (!is_open_) {
    last_error_ = "serial port is not open";
    return false;
  }

  line.clear();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
  while (std::chrono::steady_clock::now() < deadline) {
    char ch = '\0';
    const ssize_t count = read(fd_, &ch, 1);
    if (count < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      last_error_ = std::strerror(errno);
      return false;
    }

    if (count == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }

    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      if (!line.empty()) {
        last_error_.clear();
        return true;
      }
      continue;
    }
    line.push_back(ch);
  }

  last_error_ = "timed out waiting for serial response";
  return false;
#else
  (void)line;
  (void)timeoutMs;
  last_error_ = "serial transport is only implemented for Linux hosts";
  return false;
#endif
}

bool SerialTransport::SendCommand(const std::string& line, std::string& response, int timeoutMs) {
  DrainInput();
  if (!SendLine(line)) {
    return false;
  }
  return ReadLine(response, timeoutMs);
}

}  // namespace edge
