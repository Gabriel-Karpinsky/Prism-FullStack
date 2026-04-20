#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include "hardware_config.hpp"

namespace edge {

class MotionController;

enum class FaultCode {
  None,
  EStop,
  HostWatchdog,
  MotionAbort,
  LidarFault,
};

class SafetySupervisor {
 public:
  SafetySupervisor(const Config& config, MotionController& motion);
  ~SafetySupervisor();

  void Start();
  void Stop();

  // Called by the HTTP layer on any operator interaction; resets the watchdog.
  void Heartbeat();

  void TriggerFault(FaultCode code, std::string reason);
  void TriggerEStop(std::string reason);
  bool ClearFault();

  FaultCode fault_code() const;
  std::string fault_reason() const;
  bool faulted() const { return fault_code() != FaultCode::None; }

 private:
  void RunLoop();
  void NotifySystemdWatchdog() const;  // sends WATCHDOG=1 to $NOTIFY_SOCKET if set.

  Config config_;
  MotionController& motion_;
  std::atomic<bool> running_{false};
  std::thread thread_;

  mutable std::mutex mutex_;
  FaultCode fault_code_ = FaultCode::None;
  std::string fault_reason_;
  std::chrono::steady_clock::time_point last_heartbeat_;
};

const char* FaultCodeName(FaultCode code);

}  // namespace edge
