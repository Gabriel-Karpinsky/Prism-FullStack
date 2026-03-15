#include "serial_transport.hpp"

#include <cerrno>
#include <cstring>

#ifdef __linux__
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace edge {

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

  const speed_t speed = baud_ == 115200 ? B115200 : B9600;
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

  last_error_.clear();
  return true;
#else
  (void)line;
  last_error_ = "serial transport is only implemented for Linux hosts";
  return false;
#endif
}

}  // namespace edge
