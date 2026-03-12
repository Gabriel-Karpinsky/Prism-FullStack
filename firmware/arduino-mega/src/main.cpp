#include <Arduino.h>

#include "Protocol.h"
#include "ScannerController.h"

namespace {

scanner::ScannerController g_controller;
uint32_t g_lastTelemetryMs = 0;

void SendTelemetry() {
  scanner::TelemetryPayload payload{};
  payload.yawDegX10 = static_cast<int16_t>(g_controller.yaw_deg() * 10.0f);
  payload.pitchDegX10 = static_cast<int16_t>(g_controller.pitch_deg() * 10.0f);
  payload.mode = static_cast<uint8_t>(g_controller.mode());
  payload.faults = g_controller.fault_mask();
  payload.limitMask = 0;

  Serial.write(reinterpret_cast<const uint8_t*>(&payload), sizeof(payload));
}

void HandleAsciiPrototypeCommand(const String& line) {
  // This ASCII parser is only for early bench testing.
  // Replace it with the binary framed protocol for the real integration.
  if (line == "HOME") {
    g_controller.Home();
  } else if (line == "START") {
    g_controller.StartScan();
  } else if (line == "PAUSE") {
    g_controller.PauseScan();
  } else if (line == "STOP") {
    g_controller.StopScan();
  } else if (line == "ESTOP") {
    g_controller.EStop();
  } else if (line == "CLEAR_FAULT") {
    g_controller.ClearFault();
  } else if (line == "JOG_YAW_POS") {
    g_controller.Jog(5.0f, 0.0f);
  } else if (line == "JOG_YAW_NEG") {
    g_controller.Jog(-5.0f, 0.0f);
  } else if (line == "JOG_PITCH_POS") {
    g_controller.Jog(0.0f, 3.0f);
  } else if (line == "JOG_PITCH_NEG") {
    g_controller.Jog(0.0f, -3.0f);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  g_controller.Begin();
}

void loop() {
  g_controller.Update();

  if (Serial.available()) {
    const String line = Serial.readStringUntil('\n');
    HandleAsciiPrototypeCommand(line);
  }

  const uint32_t now = millis();
  if (now - g_lastTelemetryMs >= 200) {
    g_lastTelemetryMs = now;
    SendTelemetry();
  }
}
