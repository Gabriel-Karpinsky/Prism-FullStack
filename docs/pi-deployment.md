# Raspberry Pi Deployment

This document describes the intended lab deployment on the Raspberry Pi.

## Runtime processes

The Pi runs:

- `cliffscanner-edge` as a native Linux service
- `control-api` in Docker with host networking
- `tailscaled` on the host

The browser never talks directly to the Arduino or the edge daemon.

## Repository layout on the Pi

Clone or copy this repository to:

- `/opt/cliffscanner`

The deployment files in `deploy/pi` assume that repository root.

## Port plan

- edge daemon local API: `127.0.0.1:9090`
- Go backend: `127.0.0.1:8080`
- Tailscale Serve: HTTPS on the device tailnet name, forwarding to `http://127.0.0.1:8080`

## Recommended installation order

1. Install Raspberry Pi OS, Docker, and Tailscale.
2. Copy this repository to `/opt/cliffscanner`.
3. Flash the Arduino Mega firmware and wire the step/dir drivers plus trigger line.
4. Build and install the native edge daemon.
5. Start the Go backend container with Docker Compose.
6. Authenticate the host into your tailnet.
7. Publish the backend with Tailscale Serve.

## Arduino firmware assumptions

Before first motion tests, verify:

- the stepper driver DIP switches are physically set to `128` microsteps
- pin assignments in [HardwareConfig.h](/E:/_Data/_TUE/Prism/Codex web/firmware/arduino-mega/include/HardwareConfig.h)
- gear ratios and soft travel limits in [HardwareConfig.h](/E:/_Data/_TUE/Prism/Codex web/firmware/arduino-mega/include/HardwareConfig.h)
- the Garmin trigger line is connected to the configured Arduino trigger pin if you are using that sync pulse

## Edge daemon

The edge daemon is the sole hardware owner.

Install it with:

```bash
cd /opt/cliffscanner
sudo bash deploy/pi/install-edge-daemon.sh
```

Adjust `/etc/cliffscanner/edge-daemon.env` after the first install.

Important environment values:

- `EDGE_SERIAL_PORT=/dev/ttyACM0`
- `EDGE_USE_SIMULATION=false`
- `EDGE_USE_MOCK_LIDAR=false`
- `EDGE_ENABLE_SERIAL=true`
- `EDGE_BIND_HOST=127.0.0.1`
- `EDGE_BIND_PORT=9090`
- `EDGE_LIDAR_BUS=1`
- `EDGE_LIDAR_ADDRESS=98`

## Go backend container

The backend is intentionally bound to localhost so it is not exposed on the lab LAN.

Install and start it with:

```bash
cd /opt/cliffscanner
sudo bash deploy/pi/install-control-api-service.sh
```

The compose stack uses:

- `network_mode: host`
- `HTTP_BIND=127.0.0.1:8080`
- `SCANNER_BACKEND=edge`
- `EDGE_DAEMON_BASE_URL=http://127.0.0.1:9090`

## Tailscale

Install and authenticate Tailscale on the host, then publish the Go backend to the tailnet:

```bash
cd /opt/cliffscanner
sudo bash deploy/pi/configure-tailscale-serve.sh
```

That forwards a tailnet HTTPS endpoint to the locally bound backend.

## Why localhost binding matters

Binding the edge daemon and Go backend to `127.0.0.1` means:

- the lab Wi-Fi cannot directly reach the scanner services
- only local host processes can talk to the hardware API
- remote operators connect through Tailscale instead of the LAN

This is the right default for a scanner with moving hardware.

## Current MVP caveats

- `HOME` is currently a logical move-to-zero, not a limit-switch homing routine
- the Garmin driver assumes the standard v3HP I2C address/register flow and should be bench-verified on your exact wiring
- the Arduino soft limits and motion constants should be tuned against the real gantry before full-range scans
- the edge daemon localhost API is intentionally minimal and meant only for the colocated Go backend
