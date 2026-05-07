# Hardware Wiring (Pi 4B → TB6600 × 2 + LIDAR-Lite v3HP)

The edge daemon drives two TB6600 stepper drivers and the Garmin LIDAR-Lite
v3HP directly from the Pi. No microcontroller sits in the motion or sensor
path.

---

## Default GPIO map

All GPIO numbers are BCM (what `pigpio` and `gpio readall` use).

| Signal         | BCM | Physical | Direction | Target                     |
|----------------|-----|----------|-----------|----------------------------|
| yaw_step       | 17  | 11       | OUT       | TB6600 yaw  PUL−           |
| yaw_dir        | 27  | 13       | OUT       | TB6600 yaw  DIR−           |
| pitch_step     | 22  | 15       | OUT       | TB6600 pitch PUL−          |
| pitch_dir      | 23  | 16       | OUT       | TB6600 pitch DIR−          |
| enable         | 24  | 18       | OUT       | Both drivers ENA− (shared) |
| lidar_trigger  | 25  | 22       | OUT       | LIDAR-Lite v3HP MODE/TRIG  |
| status_led     | 18  | 12       | OUT       | External status LED (opt.) |
| I²C SDA        | 2   | 3        | I/O       | LIDAR-Lite v3HP SDA        |
| I²C SCL        | 3   | 5        | I/O       | LIDAR-Lite v3HP SCL        |

All pins are configurable in `/etc/prism-scanner/hardware.json`; the above
is what `DefaultConfig()` ships. The Pi also supplies **3.3 V** to the I²C
pull-up resistors (already on-board the Pi) and **5 V** to the LIDAR.

---

## TB6600 wiring (common-anode, Pi-sinks)

The TB6600 opto-isolators need ~8–15 mA through the LED. At 3.3 V with a
common-cathode wire (Pi sources current), that's barely 2 mA through the
internal ~330 Ω — too marginal to guarantee the opto fires reliably.
Common-anode wiring (Pi sinks current) is the robust choice:

```
               +5 V ─────┬──── PUL+  (and DIR+, ENA+, tied together)
                         │
                         └──── 220 Ω  (optional — TB6600 already has
                               │        internal current-limiting R)
                               └──► PUL−  ── BCM GPIO (Pi output)

Pi GPIO LOW  → current flows through opto → driver sees "asserted"
Pi GPIO HIGH → current blocked            → driver sees "idle"
```

That inversion ("Pi LOW = asserted") is why:
- `step_active_low = true`  — a step pulse is a LOW pulse.
- `enable_active_low = true` — holding ENA− LOW energises the motor.
- `dir_active_low = false`  — direction is a level, not a pulse; no
  inversion keeps the mechanical convention of the gantry consistent.

---

## LIDAR-Lite v3HP wiring

### Connector

The LIDAR-Lite v3HP has a **6-pin JST PH 2.0 mm** connector (J1).  
Garmin supplies a short pigtail harness; solder or crimp your own extensions.

| J1 pin | Wire colour | Signal  | Connect to                                | Notes                             |
|--------|-------------|---------|-------------------------------------------|-----------------------------------|
| 1      | Red         | 5 V     | Pi 5 V rail (physical pin 2 or 4)        | 4.5–5.5 V, up to 85 mA peak      |
| 2      | Black       | GND     | Pi GND (physical pin 6, 9, 14 …)        | Shared ground with Pi             |
| 3      | Orange      | PWR_EN  | Leave unconnected (float)                 | See PWR_EN note below             |
| 4      | Yellow      | MODE    | BCM 25 / physical 22                     | Trigger input; see TRIG note below|
| 5      | Blue        | SDA     | BCM 2 / physical 3 (I²C bus 1)          | 3.3 V logic; pull-up on Pi board  |
| 6      | Green       | SCL     | BCM 3 / physical 5 (I²C bus 1)          | 3.3 V logic; pull-up on Pi board  |

**Important: add a 680 µF (or 1000 µF) electrolytic capacitor between
the 5 V and GND wires as close to the LIDAR as possible.** The sensor
draws up to 85 mA in short bursts during laser fire; without the cap the
current spike causes a voltage droop that can reset or brown-out the Pi.

### PWR_EN (orange, J1-3)

An internal 10 kΩ pull-up holds PWR_EN HIGH. As long as the pin is
floating or HIGH the sensor is fully powered. Driving it LOW shuts down
the laser and most internal circuitry — useful for low-power sequencing.

We do **not** connect PWR_EN to a GPIO. The LIDAR stays always-on
whenever the scanner is running. If you add power sequencing later, wire
PWR_EN to a spare GPIO and add its pin number to `hardware.json`.

### MODE / TRIG (yellow, J1-4)

This pin has two personalities depending on how you talk to it:

**As a trigger input (what we use):**  
Drive it HIGH for ≥ 1 µs and the sensor immediately starts a ranging
measurement — no I²C write needed. The Garmin datasheet specifies ≥ 1 µs;
we default to **25 µs** (`lidar_trigger_pulse_us` in `hardware.json`) for
margin against wiring capacitance.

**As a busy/status output:**  
After triggering (either by TRIG pulse or the I²C command), the pin goes
HIGH during the measurement and returns LOW when the result is ready. You
could poll it instead of reading the status register over I²C, saving one
I²C round-trip. We currently do not use this output direction.

### I²C address

Factory default: **0x62** (decimal 98). Configurable in the LIDAR's own
registers if you need multiple sensors on the same bus.  
`hardware.json` key: `"i2c_address": 98` — the daemon talks to this address
on `/dev/i2c-1`.

---

## How the code uses the LIDAR (trigger + I²C explained)

The full per-point measurement sequence in `EdgeDaemon::ScanWorker()`:

```
1.  MotionController::MoveTo(yaw, pitch)    — blocks until DMA waveform done
2.  IGpioBackend::PulseTrigger(25 µs)       — hardware TRIG pulse on BCM 25
3.  LidarSensor::ReadDistanceMeters()       — I²C transaction sequence
```

### Step 2 — the TRIG pulse (`PulseTrigger`)

```cpp
// pigpio_gpio_backend.cpp
void PulseTrigger(std::uint32_t microseconds) override {
    gpioTrigger(config_.gpio.lidar_trigger, microseconds, 1);
}
```

`gpioTrigger(pin, us, level)` is a pigpio call that:
1. Drives BCM 25 HIGH for exactly `us` microseconds using hardware timing.
2. Returns the pin to LOW.

This goes down J1-4 (yellow wire) to the LIDAR's MODE pin. The rising
edge starts an internal measurement immediately, with no I²C overhead.
The purpose is to pre-warm the laser while the I²C transaction is setting
up — at 25 µs the laser fires almost in parallel with the I²C start
command, saving a few milliseconds per grid point at the cost of one extra
GPIO toggle.

### Step 3 — the I²C sequence (`ReadDistanceMeters`)

```cpp
// lidar_sensor.cpp — GarminLidarLiteV3HPSensor::ReadDistanceMeters()

WriteRegister(0x00, 0x01);  // ①
WaitForReady();             // ②
ReadRegister16(0x0f, distance_cm);  // ③
return distance_cm / 100.0;  // ④ convert cm → m
```

① **Write 0x01 to register 0x00 — "acquire distance, no DC correction"**  
This is the I²C command to start a new measurement (alternative to the
hardware TRIG pulse; both start the same measurement cycle; the second
one is redundant but harmless). The I²C transaction is:

```
START | 0x62 W | 0x00 | 0x01 | STOP    (select register 0x00, write 0x01)
```

② **Poll register 0x01 (status) until bit 0 == 0**  
Bit 0 of register 0x01 is the *busy flag*. The sensor sets it HIGH during
laser emission and processing (~20 ms typical at 1 cm resolution). We poll
every 2 ms, timeout at 120 ms.

```
START | 0x62 W | 0x01 | REPEATED-START | 0x62 R | [status byte] | STOP
                           … repeat until bit 0 == 0 …
```

③ **Read registers 0x0f–0x10 — the distance in cm**  
Registers 0x0f (high byte) and 0x10 (low byte) give a 16-bit big-endian
count in centimetres. A single `read(fd, bytes, 2)` auto-increments the
register pointer:

```
START | 0x62 W | 0x0f | REPEATED-START | 0x62 R | [high] [low] | STOP
```

```cpp
value = (uint16_t(bytes[0]) << 8) | bytes[1];  // big-endian → host
```

④ **Convert cm to metres** and return.

The whole sequence takes 20–30 ms per point at default settings (the laser
takes ~20 ms internally). That's why `per_point_ms = 80 ms` is the budget
used to estimate scan duration — move + settle + trigger + read.

---

## Status LED (BCM 18)

`status_led` (BCM 18, physical pin 12) is reserved in the GPIO map and the
backend has the `SetStatusLed(bool)` method wired up, but **it is not
currently driven by the daemon**. No code path calls `SetStatusLed()` yet.

The pin is set as OUTPUT and initialised LOW (off) on startup. What you
connect to it is up to your enclosure design — a simple resistor + LED to
3.3 V or 5 V works. Planned use: blink during scan, solid during fault,
off when idle. Set `"status_led": 0` in `hardware.json` to disable the
pin entirely (the backend skips its `gpioSetMode` when the value is 0).

---

## TB6600 fail-safe on power loss / daemon exit

- On clean daemon exit, pigpio releases all GPIOs (reverts to inputs).
  Un-driven TB6600 ENA− inputs float HIGH through the PUL+ tie resistor,
  which the driver reads as "disabled" — motors freewheel without braking.
- On a crash/OOM kill, the hardware watchdog (`WatchdogSec=2` in the
  systemd unit) restarts the daemon within 2 s, which re-asserts ENA.
- Any latched fault (e-stop, host watchdog timeout, motion abort) calls
  `AbortMotion()` → `gpioWaveTxStop()`. The DMA engine clears in < 1 µs;
  then `SetEnabled(false)` de-energises the motors.

---

## Electrical checks before first power-on

1. **Power the LIDAR from 5 V**, not 3.3 V. Confirm with a multimeter
   before connecting (the 5 V rail on the Pi's physical pin 2/4 is
   unfused and always live).
2. **Fit the 680–1000 µF cap** between LIDAR 5 V and GND before first
   measurement — brown-out symptoms are intermittent I²C errors, not
   obvious until under load.
3. **Confirm I²C address** with `i2cdetect -y 1` — expect `0x62` (62 in
   hex column). If you see `UU`, a kernel driver claimed the address;
   unload it.
4. **Probe TB6600 control lines** with the daemon running in
   `simulate_hardware: true` — PUL−/DIR−/ENA− should sit at 3.3 V (HIGH,
   idle, no current through opto). A logic analyser or fast oscilloscope
   lets you verify step-pulse width (default 4 µs) and DMA timing.
5. **Jog one axis** +1° after switching to `simulate_hardware: false` with
   the motor mechanically decoupled. Listen for the characteristic stepper
   "tick"; watch `journalctl -u cliffscanner-edge -f` for motion log lines.
