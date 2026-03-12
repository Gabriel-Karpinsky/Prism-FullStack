#pragma once

#include <Arduino.h>

namespace scanner {

enum class MessageType : uint8_t {
  kConnect = 0x01,
  kHeartbeat = 0x02,
  kHome = 0x10,
  kJog = 0x11,
  kMoveAbsolute = 0x12,
  kSetResolution = 0x13,
  kStartScan = 0x20,
  kPauseScan = 0x21,
  kStopScan = 0x22,
  kEStop = 0x30,
  kClearFault = 0x31,
  kTelemetry = 0x80,
  kAck = 0x81,
  kNack = 0x82,
};

struct FrameHeader {
  uint8_t magic0;
  uint8_t magic1;
  uint8_t version;
  uint8_t messageType;
  uint16_t sequence;
  uint16_t payloadLength;
};

struct JogPayload {
  int16_t yawDeltaDegX10;
  int16_t pitchDeltaDegX10;
};

struct TelemetryPayload {
  int16_t yawDegX10;
  int16_t pitchDegX10;
  uint8_t mode;
  uint8_t faults;
  uint8_t limitMask;
};

constexpr uint8_t kMagic0 = 0xAA;
constexpr uint8_t kMagic1 = 0x55;
constexpr uint8_t kProtocolVersion = 1;

uint16_t ComputeCrc16(const uint8_t* data, size_t size);

}  // namespace scanner
