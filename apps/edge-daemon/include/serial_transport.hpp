#pragma once

#include <string>

namespace edge {

class SerialTransport {
 public:
  SerialTransport(std::string port, int baud);
  ~SerialTransport();

  bool Open();
  void Close();
  void DrainInput();
  bool SendLine(const std::string& line);
  bool ReadLine(std::string& line, int timeoutMs);
  bool SendCommand(const std::string& line, std::string& response, int timeoutMs);
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
