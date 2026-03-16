#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "ScannerController.h"

namespace {

scanner::ScannerController g_controller;
constexpr size_t kLineBufferSize = 96;
char g_line_buffer[kLineBufferSize];
size_t g_line_length = 0;

const char* ModeName(scanner::MotionMode mode) {
  switch (mode) {
    case scanner::MotionMode::kIdle:
      return "idle";
    case scanner::MotionMode::kManual:
      return "manual";
    case scanner::MotionMode::kScanning:
      return "scanning";
    case scanner::MotionMode::kPaused:
      return "paused";
    case scanner::MotionMode::kFault:
      return "fault";
  }
  return "unknown";
}

void SendOk(const char* label) {
  Serial.print(F("OK "));
  Serial.println(label);
}

void SendError(const char* message) {
  Serial.print(F("ERR "));
  Serial.println(message);
}

void SendStatus() {
  const scanner::ControllerStatus status = g_controller.status();
  Serial.print(F("STATUS mode="));
  Serial.print(ModeName(status.mode));
  Serial.print(F(" moving="));
  Serial.print(status.moving ? 1 : 0);
  Serial.print(F(" yaw="));
  Serial.print(status.yaw_deg, 2);
  Serial.print(F(" pitch="));
  Serial.print(status.pitch_deg, 2);
  Serial.print(F(" targetYaw="));
  Serial.print(status.target_yaw_deg, 2);
  Serial.print(F(" targetPitch="));
  Serial.print(status.target_pitch_deg, 2);
  Serial.print(F(" fault="));
  Serial.print(status.fault_mask);
  Serial.print(F(" trigger="));
  Serial.println(status.trigger_count);
}

bool ParseFloatToken(char* token, float& value) {
  if (token == nullptr) {
    return false;
  }
  value = static_cast<float>(atof(token));
  return true;
}

void HandleCommand(char* line) {
  char* save_ptr = nullptr;
  char* command = strtok_r(line, " ", &save_ptr);
  if (command == nullptr) {
    return;
  }

  g_controller.Heartbeat();

  if (strcmp(command, "PING") == 0) {
    Serial.println(F("OK PONG"));
    return;
  }

  if (strcmp(command, "STATUS") == 0) {
    SendStatus();
    return;
  }

  if (strcmp(command, "HEARTBEAT") == 0) {
    SendOk("HEARTBEAT");
    return;
  }

  if (strcmp(command, "HOME") == 0) {
    if (!g_controller.Home()) {
      SendError("home_rejected");
      return;
    }
    SendOk("HOME");
    return;
  }

  if (strcmp(command, "MOVE") == 0) {
    float yaw = 0.0f;
    float pitch = 0.0f;
    if (!ParseFloatToken(strtok_r(nullptr, " ", &save_ptr), yaw) ||
        !ParseFloatToken(strtok_r(nullptr, " ", &save_ptr), pitch)) {
      SendError("move_requires_yaw_pitch");
      return;
    }
    if (!g_controller.MoveTo(yaw, pitch)) {
      SendError("move_rejected");
      return;
    }
    SendOk("MOVE");
    return;
  }

  if (strcmp(command, "JOG") == 0) {
    char* axis = strtok_r(nullptr, " ", &save_ptr);
    float delta = 0.0f;
    if (axis == nullptr || !ParseFloatToken(strtok_r(nullptr, " ", &save_ptr), delta)) {
      SendError("jog_requires_axis_delta");
      return;
    }

    bool accepted = false;
    if (strcmp(axis, "yaw") == 0) {
      accepted = g_controller.JogYaw(delta);
    } else if (strcmp(axis, "pitch") == 0) {
      accepted = g_controller.JogPitch(delta);
    } else {
      SendError("unknown_axis");
      return;
    }

    if (!accepted) {
      SendError("jog_rejected");
      return;
    }
    SendOk("JOG");
    return;
  }

  if (strcmp(command, "START_SCAN") == 0 || strcmp(command, "START") == 0) {
    if (!g_controller.StartScan()) {
      SendError("start_rejected");
      return;
    }
    SendOk("START_SCAN");
    return;
  }

  if (strcmp(command, "PAUSE_SCAN") == 0 || strcmp(command, "PAUSE") == 0) {
    if (!g_controller.PauseScan()) {
      SendError("pause_rejected");
      return;
    }
    SendOk("PAUSE_SCAN");
    return;
  }

  if (strcmp(command, "RESUME_SCAN") == 0 || strcmp(command, "RESUME") == 0) {
    if (!g_controller.ResumeScan()) {
      SendError("resume_rejected");
      return;
    }
    SendOk("RESUME_SCAN");
    return;
  }

  if (strcmp(command, "STOP_SCAN") == 0 || strcmp(command, "STOP") == 0) {
    if (!g_controller.StopScan()) {
      SendError("stop_rejected");
      return;
    }
    SendOk("STOP_SCAN");
    return;
  }

  if (strcmp(command, "TRIGGER") == 0) {
    if (!g_controller.Trigger()) {
      SendError("trigger_rejected");
      return;
    }
    SendOk("TRIGGER");
    return;
  }

  if (strcmp(command, "ESTOP") == 0) {
    g_controller.EStop();
    SendOk("ESTOP");
    return;
  }

  if (strcmp(command, "CLEAR_FAULT") == 0) {
    if (!g_controller.ClearFault()) {
      SendError("clear_fault_rejected");
      return;
    }
    SendOk("CLEAR_FAULT");
    return;
  }

  if (strcmp(command, "SET_RESOLUTION") == 0) {
    SendOk("SET_RESOLUTION");
    return;
  }

  if (strcmp(command, "JOG_YAW_POS") == 0) {
    if (!g_controller.JogYaw(5.0f)) {
      SendError("jog_rejected");
      return;
    }
    SendOk("JOG_YAW_POS");
    return;
  }

  if (strcmp(command, "JOG_YAW_NEG") == 0) {
    if (!g_controller.JogYaw(-5.0f)) {
      SendError("jog_rejected");
      return;
    }
    SendOk("JOG_YAW_NEG");
    return;
  }

  if (strcmp(command, "JOG_PITCH_POS") == 0) {
    if (!g_controller.JogPitch(3.0f)) {
      SendError("jog_rejected");
      return;
    }
    SendOk("JOG_PITCH_POS");
    return;
  }

  if (strcmp(command, "JOG_PITCH_NEG") == 0) {
    if (!g_controller.JogPitch(-3.0f)) {
      SendError("jog_rejected");
      return;
    }
    SendOk("JOG_PITCH_NEG");
    return;
  }

  SendError("unknown_command");
}

void ProcessSerial() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      g_line_buffer[g_line_length] = '\0';
      if (g_line_length > 0) {
        HandleCommand(g_line_buffer);
      }
      g_line_length = 0;
      continue;
    }

    if (g_line_length + 1 >= kLineBufferSize) {
      g_line_length = 0;
      SendError("line_too_long");
      continue;
    }

    g_line_buffer[g_line_length++] = ch;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  g_controller.Begin();
  Serial.println(F("OK READY"));
}

void loop() {
  g_controller.Update();
  ProcessSerial();
}
