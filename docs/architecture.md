# Architecture Notes

## Process split

The scanner runs as three processes on one Raspberry Pi 4B:

1. **`apps/edge-daemon`** — C++ service owning all hardware. Drives steppers
   via pigpio DMA waveforms, reads the Garmin LIDAR-Lite v3HP over I²C, and
   exposes a localhost JSON API. This is the only process that touches GPIO.
2. **`apps/control-api`** — Go backend. Serves the web UI, enforces the
   single-operator control lease, and proxies calls to the edge daemon.
3. **`apps/web-ui`** — Static browser dashboard.

Optionally, `tailscaled` on the Pi publishes the Go backend over the tailnet
when the lab LAN can't reach the scanner directly.

```
Browser ─(HTTPS/tailnet)─► control-api (:8080) ─(HTTP/127.0.0.1)─► edge-daemon (:9090)
                                                                         │
                                                                         ├── pigpio DMA ── TB6600 × 2 (yaw, pitch)
                                                                         └── I²C bus 1   ── LIDAR-Lite v3HP
```

## Why the edge daemon exists

The edge daemon centralises real-time hardware timing and fault handling so
the web stack never has to worry about either:

- owns all GPIO (step/dir pulses, enable, LIDAR trigger, status LED)
- reads LIDAR over I²C
- runs the scan raster state machine on a dedicated worker thread
- runs a SafetySupervisor that drops ENABLE if the host stops heart-beating
  and that pings the systemd watchdog
- persists the motion envelope to `/etc/prism-scanner/hardware.json`

The Go backend talks to the edge daemon over localhost JSON. If the daemon
crashes, systemd restarts it; pigpio is torn down on process exit, which
floats the TB6600 ENABLE line and fail-safes the drivers OFF.

## Scan loop

```
start_scan ──► ScanWorker thread ──► for each cell (boustrophedon order):
                                        1. motion_->MoveTo(yaw, pitch)     // blocks on DMA waveform
                                        2. gpio_->PulseTrigger(25 µs)
                                        3. distance = lidar_->ReadDistanceMeters(...)
                                        4. state_.grid[y][x] = normalise(distance)
                                     on fault: SafetySupervisor::TriggerFault → AbortMotion, ENABLE off
```

Pause is cooperative (worker parks on a condition variable after the current
cell); stop_scan and estop call `motion_->AbortMotion()` which halts the
pigpio waveform immediately and marks axis positions as unknown (operator
must HOME before the next move).

## Motion planning

Trapezoidal velocity profile per axis (see `stepper_axis.cpp`):

- Compute `N_accel = min(vmax² / (2·a), N/2)` microsteps.
- Accel phase: `t_i = √(2i/a)`.
- Cruise phase: linear at `vmax`.
- Decel phase: mirrored accel around the move midpoint.

Yaw and pitch plan independently, then the two step-time lists are merged
into one `WaveformPlan` so simultaneous moves run on a single pigpio DMA
waveform. Coincident-microsecond pulses OR together into a single
`gpioPulse_t` mask (one DMA entry, both axes stepping).

## Safety model

- **Control lease** — Go API grants one operator at a time; other browsers see
  read-only state.
- **Host watchdog** — SafetySupervisor faults after `host_watchdog_ms` without
  a Heartbeat() call from the HTTP layer. Any command counts as a heartbeat.
- **Systemd watchdog** — Same thread pings `WATCHDOG=1` to `$NOTIFY_SOCKET`
  every 100 ms; systemd `WatchdogSec=2` restarts the service if it stops.
- **First-cause latching** — The first fault wins; later faults are swallowed
  so the root cause isn't overwritten.
- **Fail-safe ENABLE** — TB6600 ENABLE is wired common-anode; Pi sinks to
  assert. Process exit → pigpio terminates → GPIO floats → driver OFF.

## Config layering

- `/etc/prism-scanner/hardware.json` — loaded at boot. Contains the motion
  envelope, GPIO pin map, mechanics, safety timings, LIDAR I²C address, and
  bind host/port. See `deploy/pi/hardware.json.example`.
- Motion limits are hot-swappable via `PUT /api/config/motion` and persist
  back to the same JSON file. Pin maps and mechanics require a restart.
- `PRISM_HARDWARE_CONFIG` env var overrides the config path (useful in tests).
