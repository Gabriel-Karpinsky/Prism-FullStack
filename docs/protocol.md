# Serial Protocol Sketch

The Arduino Mega should expose a compact binary protocol over USB serial.

## Frame layout

```text
0xAA 0x55 | version | message_type | sequence | payload_length | payload | crc16
```

## Recommended commands

- `CONNECT`
- `HOME`
- `JOG`
- `MOVE_ABSOLUTE`
- `SET_SCAN_WINDOW`
- `SET_RESOLUTION`
- `START_SCAN`
- `PAUSE_SCAN`
- `STOP_SCAN`
- `ESTOP`
- `CLEAR_FAULT`
- `HEARTBEAT`

## Recommended telemetry

- current yaw / pitch
- current motion mode
- limit switch state
- fault flags
- heartbeat counter
- motor current / temperature if available

## Reliability rules

- every command carries a sequence number
- the host expects an ACK / NACK
- the Arduino validates CRC before acting
- the host treats missed heartbeats as a fault
- the Arduino should fail safe on host disconnect
