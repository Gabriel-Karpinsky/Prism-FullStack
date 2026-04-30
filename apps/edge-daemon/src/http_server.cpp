// Minimal single-threaded HTTP/1.1 server for the edge-daemon control plane.
// Shared with the Go control-api over localhost. Requests are small JSON
// payloads; we trade throughput for zero external dependencies beyond
// nlohmann-json (already required by hardware_config).
//
// Endpoints:
//   GET  /health                -> {"ok":true}
//   GET  /api/hardware/state    -> Snapshot JSON (camelCase, matches Go client)
//   POST /api/hardware/command  -> {command, axis?, delta?, resolution?} -> {ok, state, error?}
//   GET  /api/config            -> full effective Config (snake_case)
//   GET  /api/config/motion     -> {yaw, pitch} motion envelope
//   PUT  /api/config/motion     -> accepts {yaw?, pitch?}; validates + persists to disk

#include "http_server.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <regex>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>

#include "edge_daemon.hpp"

#ifdef __linux__
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace edge {
namespace {

using json = nlohmann::json;

struct Request {
  std::string method;
  std::string path;
  std::string body;
};

json AxisToJson(const AxisMotion& a) {
  return json{
      {"min_deg", a.min_deg},
      {"max_deg", a.max_deg},
      {"max_speed_deg_s", a.max_speed_deg_s},
      {"accel_deg_s2", a.accel_deg_s2},
  };
}

bool ReadAxis(const json& node, AxisMotion& out) {
  if (!node.is_object()) return false;
  try {
    if (node.contains("min_deg"))         out.min_deg         = node.at("min_deg").get<double>();
    if (node.contains("max_deg"))         out.max_deg         = node.at("max_deg").get<double>();
    if (node.contains("max_speed_deg_s")) out.max_speed_deg_s = node.at("max_speed_deg_s").get<double>();
    if (node.contains("accel_deg_s2"))    out.accel_deg_s2    = node.at("accel_deg_s2").get<double>();
    return true;
  } catch (...) {
    return false;
  }
}

json ActivityToJson(const ActivityEntry& e) {
  return json{
      {"source", e.source},
      {"ts", e.ts},
      {"message", e.message},
      {"level", e.level},
  };
}

json SnapshotToJson(const Snapshot& s) {
  json activity = json::array();
  for (const auto& a : s.activity) activity.push_back(ActivityToJson(a));

  json root = {
      {"connected", s.connected},
      {"mode", s.mode},
      {"controlOwner", s.control_owner},
      {"yaw", s.yaw},
      {"pitch", s.pitch},
      {"coverage", s.coverage},
      {"scanProgress", s.scan_progress},
      {"scanDurationSeconds", s.scan_duration_seconds},
      {"scanSettings", {
          {"yawMin",             s.scan_settings.yaw_min},
          {"yawMax",             s.scan_settings.yaw_max},
          {"pitchMin",           s.scan_settings.pitch_min},
          {"pitchMax",           s.scan_settings.pitch_max},
          {"sweepSpeedDegPerSec", s.scan_settings.sweep_speed_deg_per_sec},
          {"resolution",         s.scan_settings.resolution},
      }},
      {"metrics", {
          {"motorTempC",     s.metrics.motor_temp_c},
          {"motorCurrentA",  s.metrics.motor_current_a},
          {"lidarFps",       s.metrics.lidar_fps},
          {"radarFps",       s.metrics.radar_fps},
          {"latencyMs",      s.metrics.latency_ms},
          {"packetsDropped", s.metrics.packets_dropped},
      }},
      {"faults", s.faults},
      {"activity", activity},
      {"grid", s.grid},
  };
  root["lastCompletedScanAt"]   = s.last_completed_scan_at.has_value()
                                      ? json(*s.last_completed_scan_at) : json(nullptr);
  root["controlLeaseExpiresAt"] = s.control_lease_expires_at.has_value()
                                      ? json(*s.control_lease_expires_at) : json(nullptr);
  return root;
}

Request ParseRequest(const std::string& raw) {
  Request req;
  const auto line_end = raw.find("\r\n");
  if (line_end == std::string::npos) return req;

  std::istringstream line_stream(raw.substr(0, line_end));
  line_stream >> req.method >> req.path;

  const auto header_end = raw.find("\r\n\r\n");
  if (header_end != std::string::npos) req.body = raw.substr(header_end + 4);
  return req;
}

std::string MakeResponse(int code, const std::string& status_text, const std::string& body) {
  std::ostringstream out;
  out << "HTTP/1.1 " << code << ' ' << status_text << "\r\n"
      << "Content-Type: application/json; charset=utf-8\r\n"
      << "Cache-Control: no-store\r\n"
      << "Content-Length: " << body.size() << "\r\n"
      << "Connection: close\r\n\r\n"
      << body;
  return out.str();
}

}  // namespace

HttpServer::HttpServer(ServiceConfig service, EdgeDaemon& daemon)
    : service_(std::move(service)), daemon_(daemon) {}

void HttpServer::RequestShutdown() { shutdown_.store(true); }

int HttpServer::Run() {
#ifdef __linux__
  const int server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd < 0) return 1;

  int opt = 1;
  ::setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<std::uint16_t>(service_.bind_port));
  if (::inet_pton(AF_INET, service_.bind_host.c_str(), &addr.sin_addr) <= 0) {
    ::close(server_fd);
    return 1;
  }
  if (::bind(server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(server_fd);
    return 1;
  }
  if (::listen(server_fd, 16) < 0) {
    ::close(server_fd);
    return 1;
  }

  while (!shutdown_.load()) {
    pollfd pfd{server_fd, POLLIN, 0};
    const int ready = ::poll(&pfd, 1, 500);  // periodic wakeup to observe shutdown_
    if (ready <= 0) continue;

    const int client_fd = ::accept(server_fd, nullptr, nullptr);
    if (client_fd < 0) continue;

    std::string raw;
    char buffer[4096];
    while (true) {
      const ssize_t n = ::recv(client_fd, buffer, sizeof(buffer), 0);
      if (n <= 0) break;
      raw.append(buffer, static_cast<std::size_t>(n));
      const auto hend = raw.find("\r\n\r\n");
      if (hend == std::string::npos) continue;
      std::smatch m;
      std::regex cl_re("Content-Length:\\s*([0-9]+)", std::regex_constants::icase);
      std::size_t expected = 0;
      const std::string headers = raw.substr(0, hend);
      if (std::regex_search(headers, m, cl_re)) expected = std::stoul(m[1].str());
      if (raw.size() >= hend + 4 + expected) break;
    }

    const Request req = ParseRequest(raw);
    int code = 200;
    std::string status_text = "OK";
    std::string body;

    try {
      if (req.method == "GET" && req.path == "/health") {
        body = json{{"ok", true}}.dump();

      } else if (req.method == "GET" && req.path == "/api/hardware/state") {
        body = SnapshotToJson(daemon_.GetSnapshot()).dump();

      } else if (req.method == "POST" && req.path == "/api/hardware/command") {
        CommandRequest command;
        const auto payload = req.body.empty() ? json::object() : json::parse(req.body);
        command.command    = payload.value("command",    std::string{});
        command.axis       = payload.value("axis",       std::string{});
        command.delta      = payload.value("delta",      0.0);
        command.resolution = payload.value("resolution", std::string{});

        std::string err;
        const Snapshot snap = daemon_.ExecuteCommand(command, err);
        json env = {{"ok", err.empty()}, {"state", SnapshotToJson(snap)}};
        if (!err.empty()) {
          env["error"] = err;
          code = 409;
          status_text = "Conflict";
        }
        body = env.dump();

      } else if (req.method == "GET" && req.path == "/api/config") {
        const Config c = daemon_.GetConfig();
        body = json{
            {"motion", {
                {"yaw",   AxisToJson(c.motion.yaw)},
                {"pitch", AxisToJson(c.motion.pitch)},
            }},
            {"mechanics", {
                {"full_steps_per_rev", c.mechanics.full_steps_per_rev},
                {"microsteps",         c.mechanics.microsteps},
                {"yaw_gear_ratio",     c.mechanics.yaw_gear_ratio},
                {"pitch_gear_ratio",   c.mechanics.pitch_gear_ratio},
            }},
            {"service", {
                {"bind_host",   c.service.bind_host},
                {"bind_port",   c.service.bind_port},
                {"grid_width",  c.service.grid_width},
                {"grid_height", c.service.grid_height},
            }},
            {"simulate_hardware", c.simulate_hardware},
            {"config_file_path",  c.config_file_path},
        }.dump();

      } else if (req.method == "GET" && req.path == "/api/config/motion") {
        const MotionConfig m = daemon_.GetMotionConfig();
        body = json{{"yaw", AxisToJson(m.yaw)}, {"pitch", AxisToJson(m.pitch)}}.dump();

      } else if (req.method == "PUT" && req.path == "/api/config/motion") {
        MotionConfig proposed = daemon_.GetMotionConfig();
        const auto payload = req.body.empty() ? json::object() : json::parse(req.body);
        if (payload.contains("yaw")   && !ReadAxis(payload["yaw"],   proposed.yaw))
          throw std::runtime_error("yaw payload invalid");
        if (payload.contains("pitch") && !ReadAxis(payload["pitch"], proposed.pitch))
          throw std::runtime_error("pitch payload invalid");

        const std::string err = daemon_.UpdateMotionConfig(proposed);
        if (!err.empty()) {
          code = 400; status_text = "Bad Request";
          body = json{{"ok", false}, {"error", err}}.dump();
        } else {
          const MotionConfig applied = daemon_.GetMotionConfig();
          body = json{
              {"ok", true},
              {"motion", {{"yaw", AxisToJson(applied.yaw)}, {"pitch", AxisToJson(applied.pitch)}}},
          }.dump();
        }

      } else {
        code = 404; status_text = "Not Found";
        body = json{{"error", "Not found."}}.dump();
      }
    } catch (const std::exception& e) {
      code = 400; status_text = "Bad Request";
      body = json{{"error", std::string("bad request: ") + e.what()}}.dump();
    }

    const std::string response = MakeResponse(code, status_text, body);
    ::send(client_fd, response.data(), response.size(), MSG_NOSIGNAL);
    ::close(client_fd);
  }

  ::close(server_fd);
  return 0;
#else
  return 1;
#endif
}

}  // namespace edge
