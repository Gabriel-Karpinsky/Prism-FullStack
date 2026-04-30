# Raspberry Pi Deployment

Pi-native bring-up. There is no microcontroller in the motion path — the Pi
drives the TB6600 drivers directly via `pigpio` DMA waveforms and reads the
LIDAR over I²C.

## Two paths

1. **Flash the pre-baked image** (recommended). Zero on-device setup beyond
   wiring. Skip to ["Image flash path"](#image-flash-path).
2. **Build from source on a running Pi.** Useful when iterating on the daemon
   itself. See ["Build-from-source path"](#build-from-source-path).

## Image flash path

1. Grab the latest release `.img.xz` from
   <https://github.com/Gabriel-Karpinsky/Prism-FullStack/releases>
   (file: `cliffscanner-pi-aarch64-vX.Y.Z.img.xz`).
2. (Optional, for remote access) Generate a Tailscale auth key at
   <https://login.tailscale.com/admin/settings/keys>. A reusable, ephemeral,
   pre-authorised key is the safest choice — it auto-expires and can't be
   replayed off the SD card.
3. Flash with **Raspberry Pi Imager**:
   - "Choose OS" → "Use custom" → select the `.img.xz`.
   - "Choose Storage" → your SD card.
   - "Settings" cog → enable SSH, set username/password, set WiFi SSID +
     password if needed. Imager writes these into the image at flash time
     so you don't have to bake credentials into the release.
4. After flashing finishes, **before** ejecting the card, drop the auth key:
   ```
   echo "tskey-auth-...your-key..." > /Volumes/bootfs/tailscale-authkey
   ```
   (On Windows, the boot partition shows up as a drive letter; create a file
   named `tailscale-authkey` with the key inside.)
5. Insert the card into the Pi, power on. First boot:
   - Resizes the root filesystem to fill the SD card (Raspberry Pi OS default).
   - Sets the hostname to `cliffscanner-XXXXXX` (last 6 hex of eth0 MAC).
   - If `tailscale-authkey` is present: joins the tailnet with `--ssh`,
     then shreds the key file.
   - Starts `cliffscanner-edge` and `cliffscanner-control-api`.
6. Find the device in your Tailscale admin console (or local LAN); SSH in
   and run the smoke checks below.

### Smoke checks

```bash
systemctl is-active cliffscanner-edge        # active
systemctl is-active cliffscanner-control-api # active
curl -s http://127.0.0.1:9090/health         # {"ok":true}
curl -s http://127.0.0.1:8080/healthz        # {"ok":true}
cat /etc/cliffscanner/version                 # matches release tag
journalctl -u cliffscanner-firstboot --no-pager
```

If something looks wrong, the rest of this doc covers the build-from-source
path so you can rebuild any one component on the device.

## Build-from-source path

## Runtime processes

- `cliffscanner-edge` — native systemd service, owns GPIO + I²C.
- `control-api` — Go backend, runs in Docker with host networking.
- `tailscaled` — optional, for remote operator access.

## Port plan

| Endpoint                      | Binds on          |
|-------------------------------|-------------------|
| edge-daemon API               | 127.0.0.1:9090    |
| control-api (Go)              | 127.0.0.1:8080    |
| Tailscale Serve (optional)    | tailnet HTTPS → 127.0.0.1:8080 |

## Prerequisites

```bash
sudo apt update
sudo apt install build-essential pigpio libpigpio-dev nlohmann-json3-dev \
                  i2c-tools docker.io docker-compose-plugin
sudo raspi-config nonint do_i2c 0   # enable I²C bus 1
sudo usermod -aG i2c "$USER"
```

Confirm `/boot/config.txt` has `dtparam=i2c_arm=on` (raspi-config sets this).

## Installation order

1. Flash Raspberry Pi OS 64-bit, install Docker and Tailscale.
2. Clone this repo to `/opt/cliffscanner`.
3. Wire the TB6600 drivers and LIDAR per [hardware-pins.md](./hardware-pins.md).
4. Install the edge daemon:
   ```bash
   sudo bash /opt/cliffscanner/deploy/pi/install-edge-daemon.sh
   ```
5. Start the Go backend container:
   ```bash
   sudo bash /opt/cliffscanner/deploy/pi/install-control-api-service.sh
   ```
6. (Optional) `sudo bash deploy/pi/configure-tailscale-serve.sh`.

The edge-daemon installer:

- builds with `HAS_PIGPIO=1` and installs to `/opt/cliffscanner/bin/`.
- seeds `/etc/prism-scanner/hardware.json` from `hardware.json.example` on
  first install. Later edits survive reinstalls.
- seeds `/etc/cliffscanner/edge-daemon.env` with the config-file path.
- disables `pigpiod` if it's running (we use the library in-process).

## Configuration

Runtime hardware config lives in `/etc/prism-scanner/hardware.json`. The
shipped defaults match the documented envelope:

```json
{
  "motion": {
    "yaw":   {"min_deg": -50, "max_deg": 50, "max_speed_deg_s": 18, "accel_deg_s2": 60},
    "pitch": {"min_deg": -30, "max_deg": 30, "max_speed_deg_s": 12, "accel_deg_s2": 40}
  },
  "mechanics": {"full_steps_per_rev": 200, "microsteps": 128, "yaw_gear_ratio": 1.0, "pitch_gear_ratio": 1.0},
  "gpio":     {"yaw_step": 17, "yaw_dir": 27, "pitch_step": 22, "pitch_dir": 23,
                "enable": 24, "lidar_trigger": 25, "status_led": 18,
                "step_active_low": true, "dir_active_low": false, "enable_active_low": true},
  "safety":   {"host_watchdog_ms": 1500, "step_pulse_us": 4, "lidar_trigger_pulse_us": 25},
  "lidar":    {"i2c_bus": 1, "i2c_address": 98, "simulate": false},
  "service":  {"bind_host": "127.0.0.1", "bind_port": 9090,
                "grid_width": 48, "grid_height": 24,
                "tick_interval_ms": 20, "status_broadcast_interval_ms": 100},
  "simulate_hardware": false
}
```

Motion limits are hot-swappable via the HTTP API:

```bash
# Read
curl -s http://127.0.0.1:8080/api/config/motion | jq

# Write (requires current control lease)
curl -s -X PUT http://127.0.0.1:8080/api/config/motion \
     -H 'Content-Type: application/json' \
     -d '{"user": "alice",
           "motion": {
             "yaw":   {"min_deg": -40, "max_deg": 40, "max_speed_deg_s": 15, "accel_deg_s2": 50},
             "pitch": {"min_deg": -25, "max_deg": 25, "max_speed_deg_s": 10, "accel_deg_s2": 35}}}'
```

Pin maps, mechanics, and safety timings require a daemon restart to take effect:

```bash
sudo systemctl restart cliffscanner-edge
```

## Simulation mode (no Pi, no wiring)

On a laptop, build without pigpio:

```bash
cd apps/edge-daemon
make clean && make        # HAS_PIGPIO unset ⇒ mock backend
./cliffscanner-edge
```

Or on the Pi with hardware attached but you want a dry-run, flip
`"simulate_hardware": true` in the config. The mock backend accepts the same
waveform plans and returns success without touching GPIO.

## Health checks

```bash
curl -s http://127.0.0.1:9090/health             # edge-daemon
curl -s http://127.0.0.1:9090/api/hardware/state # full snapshot
systemctl status cliffscanner-edge               # watchdog + uptime
journalctl -u cliffscanner-edge -f               # logs
```

## Why localhost binding

The edge daemon and Go backend both bind to `127.0.0.1`. The lab Wi-Fi cannot
reach the hardware directly; remote operators come in through Tailscale.

## Current caveats

- No endstops. `home` is a logical move-to-zero; the operator is expected to
  hand-zero the gantry before the first service start.
- LIDAR normalisation (`(6.4 - distance) / 2.8`) is a placeholder that maps
  typical cliff-face ranges into the UI's 0..1 band. Retune per site.
- Pause is cooperative; stop/estop abort the DMA waveform immediately and
  mark axis positions as unknown — rehome before the next move.
