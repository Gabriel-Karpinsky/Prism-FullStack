#include <exception>
#include <iostream>

#include "config.hpp"
#include "edge_daemon.hpp"
#include "http_server.hpp"

int main() {
  try {
    edge::Config config = edge::LoadConfigFromEnv();
    edge::EdgeDaemon daemon(config);
    daemon.Start();

    std::cout << "Edge daemon listening on http://" << config.bind_host << ":" << config.bind_port << std::endl;

    edge::HttpServer server(config, daemon);
    const int rc = server.Run();
    daemon.Stop();
    return rc;
  } catch (const std::exception& ex) {
    std::cerr << "edge daemon fatal error: " << ex.what() << std::endl;
    return 1;
  }
}
