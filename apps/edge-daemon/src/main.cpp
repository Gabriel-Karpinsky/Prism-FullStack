// Edge-daemon entry point. Loads hardware config, brings up the daemon, and
// serves HTTP requests on a single thread. SIGINT/SIGTERM route through a
// shutdown flag so the accept loop and worker threads unwind cleanly.

#include <atomic>
#include <csignal>
#include <exception>
#include <iostream>

#include "edge_daemon.hpp"
#include "hardware_config.hpp"
#include "http_server.hpp"

namespace {

edge::HttpServer* g_server = nullptr;
std::atomic<int> g_signal{0};

void HandleSignal(int sig) {
  g_signal.store(sig);
  if (g_server) g_server->RequestShutdown();
}

}  // namespace

int main() {
  try {
    const std::string config_path = edge::ResolveConfigPath();
    edge::Config config = edge::LoadConfigFromFile(config_path);

    edge::EdgeDaemon daemon(config);
    if (!daemon.Start()) {
      std::cerr << "edge daemon: startup failed\n";
      return 1;
    }

    edge::HttpServer server(config.service, daemon);
    g_server = &server;
    std::signal(SIGINT,  HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    std::cout << "edge daemon listening on http://"
              << config.service.bind_host << ':' << config.service.bind_port
              << " (config: " << config_path << ")" << std::endl;

    const int rc = server.Run();
    g_server = nullptr;
    daemon.Stop();
    return rc;
  } catch (const std::exception& ex) {
    std::cerr << "edge daemon fatal error: " << ex.what() << std::endl;
    return 1;
  }
}
