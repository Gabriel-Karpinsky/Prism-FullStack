#pragma once

#include "config.hpp"
#include "edge_daemon.hpp"

namespace edge {

class HttpServer {
 public:
  HttpServer(Config config, EdgeDaemon& daemon);
  int Run();

 private:
  Config config_;
  EdgeDaemon& daemon_;
};

}  // namespace edge
