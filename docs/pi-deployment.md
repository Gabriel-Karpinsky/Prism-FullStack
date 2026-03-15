# Raspberry Pi Deployment

This document describes the intended MVP deployment on the Raspberry Pi.

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
- Tailscale Serve: HTTPS on the device's tailnet name, forwarding to `http://127.0.0.1:8080`

## Recommended installation order

1. Install Raspberry Pi OS, Docker, and Tailscale.
2. Copy this repository to `/opt/cliffscanner`.
3. Build and install the native edge daemon.
4. Start the Go backend container with Docker Compose.
5. Authenticate the host into your tailnet.
6. Publish the backend with Tailscale Serve.

## Edge daemon

The edge daemon is designed to be the sole hardware owner.

Install it with:

```bash
cd /opt/cliffscanner
sudo bash deploy/pi/install-edge-daemon.sh
```

Adjust `/etc/cliffscanner/edge-daemon.env` after the first install.

Important environment values:

- `EDGE_SERIAL_PORT=/dev/ttyACM0`
- `EDGE_USE_SIMULATION=false` once the Arduino and lidar path are ready
- `EDGE_ENABLE_SERIAL=true`
- `EDGE_BIND_HOST=127.0.0.1`
- `EDGE_BIND_PORT=9090`

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

- the C++ edge daemon includes a mock lidar implementation today; it is ready for the Garmin v3HP driver to be dropped in behind the `LidarSensor` interface
- the Arduino firmware still uses the temporary ASCII command bridge for motion commands
- once the framed serial protocol is wired up, the edge daemon should switch to that binary transport instead of ASCII strings
