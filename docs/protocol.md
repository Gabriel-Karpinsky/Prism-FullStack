# MVP Serial Protocol

The current Arduino-to-Pi protocol is a line-oriented ASCII protocol over USB serial at `115200` baud.

This is the protocol actually used by the MVP firmware and edge daemon today.

## Command format

Each command is one UTF-8/ASCII line terminated by `\n`.

Examples:

```text
HEARTBEAT
STATUS
HOME
MOVE -12.50 18.00
JOG yaw 2.50
START_SCAN
PAUSE_SCAN
RESUME_SCAN
STOP_SCAN
TRIGGER
ESTOP
CLEAR_FAULT
```

## Responses

The Arduino responds with one line per command.

Success:

```text
OK <label>
```

Errors:

```text
ERR <reason>
```

Status snapshots:

```text
STATUS mode=scanning moving=1 yaw=-12.50 pitch=18.00 targetYaw=-12.50 targetPitch=18.00 fault=0 trigger=14
```

## Supported commands

- `PING`
- `HEARTBEAT`
- `STATUS`
- `HOME`
- `MOVE <yawDeg> <pitchDeg>`
- `JOG yaw <deltaDeg>`
- `JOG pitch <deltaDeg>`
- `START_SCAN`
- `PAUSE_SCAN`
- `RESUME_SCAN`
- `STOP_SCAN`
- `TRIGGER`
- `ESTOP`
- `CLEAR_FAULT`
- `SET_RESOLUTION` currently ACKs for compatibility and is handled on the Pi side

Legacy jog aliases are still accepted for bench compatibility:

- `JOG_YAW_POS`
- `JOG_YAW_NEG`
- `JOG_PITCH_POS`
- `JOG_PITCH_NEG`
- `START`
- `PAUSE`
- `STOP`

## Firmware behavior

- yaw and pitch are driven as independent step/dir axes
- the firmware assumes the external drivers are already configured for `128` microsteps
- the Arduino clamps commands to the configured software travel limits
- the `HEARTBEAT` command refreshes a watchdog timer
- if the watchdog expires during active control, the Arduino enters fault mode
- `TRIGGER` emits a short pulse on the configured Garmin trigger pin

## Future upgrade path

A framed binary protocol is still a sensible future step once the MVP is proven in the lab.

That future protocol should add:

- explicit framing and CRC
- sequence numbers and ACK/NACK handling
- richer telemetry
- a cleaner path to higher-rate streaming
