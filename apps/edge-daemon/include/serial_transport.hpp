#pragma once

#include <string>

namespace edge {

class SerialTransport {
 public:
  SerialTransport(std::string port, int baud);
  ~SerialTransport();

  bool Open();
  void Close();
  bool SendLine(const std::string& line);
  bool IsOpen() const { return is_open_; }
  const std::string& last_error() const { return last_error_; }

 private:
  std::string port_;
  int baud_;
  bool is_open_ = false;
  std::string last_error_;
#ifdef __linux__
  int fd_ = -1;
#endif
};

}  // namespace edge
