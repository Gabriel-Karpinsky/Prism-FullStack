#include "safety_supervisor.hpp"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

#include "motion_controller.hpp"
#include "systemd_notify.hpp"

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
  SystemdNotify("WATCHDOG=1");
}

}  // namespace edge
