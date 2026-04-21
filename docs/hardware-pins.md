# Hardware Wiring (Pi 4B → TB6600 × 2 + LIDAR-Lite v3HP)

The edge daemon drives two TB6600 (TB67S109A-based) stepper drivers and the
Garmin LIDAR-Lite v3HP directly from the Pi. No microcontroller sits between
the Pi and the drivers.

## Default GPIO map

All GPIO numbers are BCM (what `pigpio` and `gpio readall` use).

| Signal         | BCM | Physical | Direction | Target            |
|----------------|-----|----------|-----------|-------------------|
| yaw_step       | 17  | 11       | OUT       | TB6600 yaw PUL−   |
| yaw_dir        | 27  | 13       | OUT       | TB6600 yaw DIR−   |
| pitch_step     | 22  | 15       | OUT       | TB6600 pitch PUL− |
| pitch_dir      | 23  | 16       | OUT       | TB6600 pitch DIR− |
| enable         | 24  | 18       | OUT       | Both drivers ENA− |
| lidar_trigger  | 25  | 22       | OUT       | LIDAR TRIG (J1-4) |
| status_led     | 18  | 12       | OUT       | On-board status   |
| I²C SDA        | 2   | 3        | I/O       | LIDAR SDA (J1-5)  |
| I²C SCL        | 3   | 5        | I/O       | LIDAR SCL (J1-6)  |

Every pin is configurable in `/etc/prism-scanner/hardware.json`; the above is
what `DefaultConfig()` ships.

## TB6600 wiring (common-anode, Pi-sinks)

The TB6600 opto-isolators need ~8–15 mA through the LED. At 3.3 V with a
common-cathode wire (Pi sources), that's barely 2 mA through the internal
~330 Ω — marginal. Common-anode wiring (Pi sinks) is the robust choice:

```
               +5 V ───┬──── PUL+ (and DIR+, ENA+)
                       │
                       └──── 220 Ω (optional — TB6600 already has internal R)
                             │
                             └──► PUL− ── GPIO (Pi)

Pi GPIO LOW  = current flows = opto ON  = driver sees "asserted"
Pi GPIO HIGH = current blocked = opto OFF = driver sees "idle"
```

That "Pi LOW ⇒ asserted" is why `step_active_low=true` and
`enable_active_low=true` in the default config. `dir_active_low=false` because
direction is logically a level (not a pulse) and flipping it matches the
mechanical convention on the gantry.

## LIDAR-Lite v3HP (I²C @ 0x62)

- SDA / SCL go to BCM 2/3 (physical 3/5) — standard I²C bus `/dev/i2c-1`.
- 5 V power, **not** 3.3 V. The SDA/SCL lines are 3.3 V tolerant.
- TRIG (J1-4) is optional and wired to BCM 25. The daemon pulses it for
  `safety.lidar_trigger_pulse_us` (default 25 µs) before each read.
- Enable `i2c-1` in `/boot/config.txt`:
  ```
  dtparam=i2c_arm=on
  ```
- The daemon opens `/dev/i2c-1` directly; the `i2c` group is in the systemd
  unit's SupplementaryGroups so it's reachable if we later drop root.

## Fail-safe behaviour

- On clean exit, pigpio releases all GPIOs. Un-driven TB6600 ENA− inputs are
  pulled to ENA+ through the opto, which reads as "disabled" — motors freewheel
  safely.
- The SafetySupervisor thread pings `WATCHDOG=1` at 10 Hz; systemd restarts
  the service if it stops (`WatchdogSec=2` in the unit file).
- Any fault (e-stop, host watchdog, motion abort, lidar fault) calls
  `motion_->AbortMotion()` which invokes `gpioWaveTxStop()` — the DMA buffer
  is cleared in microseconds, then `ENABLE` is deasserted.

## Electrical cross-checks before first power-on

1. Confirm TB6600 supply is 9–42 V DC, current limit set for your motor.
2. Probe PUL−/DIR−/ENA− with the daemon running in simulate mode
   (`simulate_hardware: true`) — they should sit HIGH (idle) at 3.3 V.
3. Jog yaw +1° in simulate mode; the level should toggle with the expected
   `step_pulse_us` (default 4 µs). A scope or logic analyser confirms the
   waveform cadence matches `v_max_deg_s`.
4. Flip `simulate_hardware: false` and repeat the jog with the motor coupled.
