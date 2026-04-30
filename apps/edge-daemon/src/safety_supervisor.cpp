#include "safety_supervisor.hpp"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

#include "motion_controller.hpp"

#ifdef __linux__
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#endif

namespace edge {

const char* FaultCodeName(FaultCode code) {
  switch (code) {
    case FaultCode::None:         return "none";
    case FaultCode::EStop:        return "estop";
    case FaultCode::HostWatchdog: return "host_watchdog";
    case FaultCode::MotionAbort:  return "motion_abort";
    case FaultCode::LidarFault:   return "lidar_fault";
  }
  return "unknown";
}

SafetySupervisor::SafetySupervisor(const Config& config, MotionController& motion)
    : config_(config), motion_(motion), last_heartbeat_(std::chrono::steady_clock::now()) {}

SafetySupervisor::~SafetySupervisor() { Stop(); }

void SafetySupervisor::Start() {
  if (running_.exchange(true)) return;
  last_heartbeat_ = std::chrono::steady_clock::now();
  thread_ = std::thread([this] { RunLoop(); });
}

void SafetySupervisor::Stop() {
  if (!running_.exchange(false)) return;
  if (thread_.joinable()) thread_.join();
}

void SafetySupervisor::Heartbeat() {
  std::lock_guard<std::mutex> lock(mutex_);
  last_heartbeat_ = std::chrono::steady_clock::now();
}

void SafetySupervisor::TriggerFault(FaultCode code, std::string reason) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fault_code_ != FaultCode::None) return;  // first fault wins (preserves root cause).
    fault_code_ = code;
    fault_reason_ = std::move(reason);
  }
  motion_.AbortMotion();
  motion_.SetEnabled(false);
}

void SafetySupervisor::TriggerEStop(std::string reason) {
  TriggerFault(FaultCode::EStop, std::move(reason));
}

bool SafetySupervisor::ClearFault() {
  std::lock_guard<std::mutex> lock(mutex_);
  fault_code_ = FaultCode::None;
  fault_reason_.clear();
  last_heartbeat_ = std::chrono::steady_clock::now();
  return true;
}

FaultCode SafetySupervisor::fault_code() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return fault_code_;
}

std::string SafetySupervisor::fault_reason() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return fault_reason_;
}

void SafetySupervisor::RunLoop() {
  const auto tick = std::chrono::milliseconds(100);
  const auto watchdog_ms = std::chrono::milliseconds(config_.safety.host_watchdog_ms);

  while (running_.load()) {
    std::this_thread::sleep_for(tick);
    NotifySystemdWatchdog();

    std::chrono::steady_clock::time_point last;
    FaultCode current;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last = last_heartbeat_;
      current = fault_code_;
    }
    if (current != FaultCode::None) continue;

    const auto since = std::chrono::steady_clock::now() - last;
    if (since > watchdog_ms) {
      TriggerFault(FaultCode::HostWatchdog,
                   "no host heartbeat in " + std::to_string(config_.safety.host_watchdog_ms) + "ms");
    }
  }
}

void SafetySupervisor::NotifySystemdWatchdog() const {
#ifdef __linux__
  const char* socket_path = std::getenv("NOTIFY_SOCKET");
  if (socket_path == nullptr || *socket_path == '\0') return;

  const int fd = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd < 0) return;

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (socket_path[0] == '@') {
    // Abstract socket namespace.
    addr.sun_path[0] = '\0';
    std::strncpy(addr.sun_path + 1, socket_path + 1, sizeof(addr.sun_path) - 2);
  } else {
    std::strncpy(addr.sun_path, socket_path, sizeof(addr.sun_path) - 1);
  }

  const char* msg = "WATCHDOG=1";
  ::sendto(fd, msg, std::strlen(msg), MSG_NOSIGNAL,
           reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
  ::close(fd);
#endif
}

}  // namespace edge
