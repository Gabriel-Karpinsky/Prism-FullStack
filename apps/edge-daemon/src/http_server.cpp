#include "http_server.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#ifdef __linux__
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace edge {
namespace {

struct Request {
  std::string method;
  std::string path;
  std::string body;
};

std::string EscapeJson(const std::string& value) {
  std::string out;
  out.reserve(value.size() + 8);
  for (char c : value) {
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default: out += c; break;
    }
  }
  return out;
}

std::string FormatDouble(double value) {
  std::ostringstream out;
  out.setf(std::ios::fixed, std::ios::floatfield);
  out.precision(4);
  out << value;
  std::string text = out.str();
  while (!text.empty() && text.back() == '0') text.pop_back();
  if (!text.empty() && text.back() == '.') text.pop_back();
  if (text.empty()) return "0";
  return text;
}

std::string ActivityToJson(const ActivityEntry& entry) {
  std::ostringstream out;
  out << "{"
      << "\"source\":\"" << EscapeJson(entry.source) << "\"," 
      << "\"ts\":\"" << EscapeJson(entry.ts) << "\"," 
      << "\"message\":\"" << EscapeJson(entry.message) << "\"," 
      << "\"level\":\"" << EscapeJson(entry.level) << "\""
      << "}";
  return out.str();
}

std::string SnapshotToJson(const Snapshot& snapshot) {
  std::ostringstream out;
  out << "{";
  out << "\"connected\":" << (snapshot.connected ? "true" : "false") << ",";
  out << "\"mode\":\"" << EscapeJson(snapshot.mode) << "\",";
  out << "\"controlOwner\":\"\",";
  out << "\"controlLeaseExpiresAt\":null,";
  out << "\"yaw\":" << FormatDouble(snapshot.yaw) << ",";
  out << "\"pitch\":" << FormatDouble(snapshot.pitch) << ",";
  out << "\"coverage\":" << FormatDouble(snapshot.coverage) << ",";
  out << "\"scanProgress\":" << FormatDouble(snapshot.scan_progress) << ",";
  out << "\"scanDurationSeconds\":" << FormatDouble(snapshot.scan_duration_seconds) << ",";
  if (snapshot.last_completed_scan_at.has_value()) {
    out << "\"lastCompletedScanAt\":\"" << EscapeJson(*snapshot.last_completed_scan_at) << "\",";
  } else {
    out << "\"lastCompletedScanAt\":null,";
  }
  out << "\"scanSettings\":{";
  out << "\"yawMin\":" << FormatDouble(snapshot.scan_settings.yaw_min) << ",";
  out << "\"yawMax\":" << FormatDouble(snapshot.scan_settings.yaw_max) << ",";
  out << "\"pitchMin\":" << FormatDouble(snapshot.scan_settings.pitch_min) << ",";
  out << "\"pitchMax\":" << FormatDouble(snapshot.scan_settings.pitch_max) << ",";
  out << "\"sweepSpeedDegPerSec\":" << FormatDouble(snapshot.scan_settings.sweep_speed_deg_per_sec) << ",";
  out << "\"resolution\":\"" << EscapeJson(snapshot.scan_settings.resolution) << "\"";
  out << "},";
  out << "\"metrics\":{";
  out << "\"motorTempC\":" << FormatDouble(snapshot.metrics.motor_temp_c) << ",";
  out << "\"motorCurrentA\":" << FormatDouble(snapshot.metrics.motor_current_a) << ",";
  out << "\"lidarFps\":" << snapshot.metrics.lidar_fps << ",";
  out << "\"radarFps\":" << snapshot.metrics.radar_fps << ",";
  out << "\"latencyMs\":" << snapshot.metrics.latency_ms << ",";
  out << "\"packetsDropped\":" << snapshot.metrics.packets_dropped;
  out << "},";
  out << "\"faults\":[";
  for (size_t i = 0; i < snapshot.faults.size(); ++i) {
    if (i > 0) out << ",";
    out << "\"" << EscapeJson(snapshot.faults[i]) << "\"";
  }
  out << "],";
  out << "\"activity\":[";
  for (size_t i = 0; i < snapshot.activity.size(); ++i) {
    if (i > 0) out << ",";
    out << ActivityToJson(snapshot.activity[i]);
  }
  out << "],";
  out << "\"grid\":[";
  for (size_t y = 0; y < snapshot.grid.size(); ++y) {
    if (y > 0) out << ",";
    out << "[";
    for (size_t x = 0; x < snapshot.grid[y].size(); ++x) {
      if (x > 0) out << ",";
      out << FormatDouble(snapshot.grid[y][x]);
    }
    out << "]";
  }
  out << "]";
  out << "}";
  return out.str();
}

std::string Envelope(bool ok, const Snapshot& snapshot, const std::string& error) {
  std::ostringstream out;
  out << "{";
  if (!error.empty()) {
    out << "\"error\":\"" << EscapeJson(error) << "\",";
  }
  out << "\"ok\":" << (ok ? "true" : "false") << ",";
  out << "\"state\":" << SnapshotToJson(snapshot);
  out << "}";
  return out.str();
}

std::optional<std::string> ExtractString(const std::string& json, const std::string& key) {
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
  std::smatch match;
  if (std::regex_search(json, match, pattern) && match.size() > 1) {
    return match[1].str();
  }
  return std::nullopt;
}

std::optional<double> ExtractNumber(const std::string& json, const std::string& key) {
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
  std::smatch match;
  if (std::regex_search(json, match, pattern) && match.size() > 1) {
    return std::stod(match[1].str());
  }
  return std::nullopt;
}

Request ParseRequest(const std::string& raw) {
  Request request;
  const auto line_end = raw.find("\r\n");
  if (line_end == std::string::npos) {
    return request;
  }

  const std::string request_line = raw.substr(0, line_end);
  std::istringstream line_stream(request_line);
  line_stream >> request.method >> request.path;

  const auto header_end = raw.find("\r\n\r\n");
  if (header_end != std::string::npos) {
    request.body = raw.substr(header_end + 4);
  }
  return request;
}

std::string MakeHttpResponse(int status_code, const std::string& status_text, const std::string& body) {
  std::ostringstream out;
  out << "HTTP/1.1 " << status_code << " " << status_text << "\r\n";
  out << "Content-Type: application/json; charset=utf-8\r\n";
  out << "Cache-Control: no-store\r\n";
  out << "Content-Length: " << body.size() << "\r\n";
  out << "Connection: close\r\n\r\n";
  out << body;
  return out.str();
}

}  // namespace

HttpServer::HttpServer(Config config, EdgeDaemon& daemon)
    : config_(std::move(config)), daemon_(daemon) {}

int HttpServer::Run() {
#ifdef __linux__
  const int server_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd < 0) {
    return 1;
  }

  int opt = 1;
  setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(config_.bind_port));
  if (inet_pton(AF_INET, config_.bind_host.c_str(), &addr.sin_addr) <= 0) {
    close(server_fd);
    return 1;
  }

  if (bind(server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    close(server_fd);
    return 1;
  }

  if (listen(server_fd, 16) < 0) {
    close(server_fd);
    return 1;
  }

  while (true) {
    const int client_fd = accept(server_fd, nullptr, nullptr);
    if (client_fd < 0) {
      continue;
    }

    std::string raw;
    char buffer[4096];
    while (true) {
      const ssize_t count = recv(client_fd, buffer, sizeof(buffer), 0);
      if (count <= 0) {
        break;
      }
      raw.append(buffer, static_cast<size_t>(count));
      if (raw.find("\r\n\r\n") != std::string::npos) {
        const auto header_end = raw.find("\r\n\r\n");
        const std::string headers = raw.substr(0, header_end);
        std::smatch match;
        std::regex length_pattern("Content-Length:\\s*([0-9]+)", std::regex_constants::icase);
        size_t expected_body = 0;
        if (std::regex_search(headers, match, length_pattern) && match.size() > 1) {
          expected_body = static_cast<size_t>(std::stoul(match[1].str()));
        }
        if (raw.size() >= header_end + 4 + expected_body) {
          break;
        }
      }
    }

    const Request request = ParseRequest(raw);
    std::string body;
    int status_code = 200;
    std::string status_text = "OK";

    if (request.method == "GET" && request.path == "/health") {
      body = "{\"ok\":true}";
    } else if (request.method == "GET" && request.path == "/api/hardware/state") {
      body = SnapshotToJson(daemon_.GetSnapshot());
    } else if (request.method == "POST" && request.path == "/api/hardware/command") {
      CommandRequest command;
      command.command = ExtractString(request.body, "command").value_or("");
      command.axis = ExtractString(request.body, "axis").value_or("");
      command.delta = ExtractNumber(request.body, "delta").value_or(0.0);
      command.resolution = ExtractString(request.body, "resolution").value_or("");

      std::string error_message;
      const Snapshot snapshot = daemon_.ExecuteCommand(command, error_message);
      if (!error_message.empty()) {
        status_code = 409;
        status_text = "Conflict";
        body = Envelope(false, snapshot, error_message);
      } else {
        body = Envelope(true, snapshot, "");
      }
    } else {
      status_code = 404;
      status_text = "Not Found";
      body = "{\"error\":\"Not found.\"}";
    }

    const std::string response = MakeHttpResponse(status_code, status_text, body);
    send(client_fd, response.data(), response.size(), 0);
    close(client_fd);
  }

  close(server_fd);
  return 0;
#else
  return 1;
#endif
}

}  // namespace edge
