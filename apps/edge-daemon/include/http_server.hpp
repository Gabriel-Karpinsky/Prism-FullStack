#pragma once

#include <atomic>

#include "edge_daemon.hpp"
#include "hardware_config.hpp"

namespace edge {

class HttpServer {
 public:
  HttpServer(ServiceConfig service, EdgeDaemon& daemon);

  // Runs the accept loop on the calling thread. Returns non-zero on listen/bind failure.
  int Run();

  // Signals Run() to return at the next accept timeout.
  void RequestShutdown();

 private:
  ServiceConfig service_;
  EdgeDaemon& daemon_;
  std::atomic<bool> shutdown_{false};
};

}  // namespace edge
