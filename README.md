# Cliff Face Scanner Prototype

This repository now contains the first pass of the production-oriented stack for a cliff face scanner MVP.

The intended deployment shape is:

- Arduino Mega for deterministic motion control only
- Raspberry Pi native C++ edge daemon for hardware ownership and lidar processing
- Go control API in Docker on the Raspberry Pi
- Browser UI served directly by the Go backend
- Tailscale on the Raspberry Pi host for remote operator access when the lab LAN blocks direct connections

## Repository layout

- `apps/control-api`: Go backend that serves the browser UI and talks either to a simulator or the local edge daemon
- `apps/edge-daemon`: native C++ Raspberry Pi service that owns the Arduino/LIDAR side of the MVP
- `apps/prototype-server`: the original PowerShell prototype server kept for reference
- `apps/web-ui`: dependency-free browser UI
- `firmware/arduino-mega`: Arduino Mega firmware scaffold
- `proto/scanner/v1`: future protobuf contract for a stronger multi-language boundary
- `deploy/pi`: Raspberry Pi deployment assets for systemd, Docker Compose, and Tailscale
- `docs`: architecture and packaging notes

## Current status

Implemented now:

- Go backend can run in `sim` mode or `edge` mode
- edge mode calls a localhost edge daemon API instead of the in-process simulator
- Docker packaging exists for the Go backend
- native Raspberry Pi edge daemon scaffold exists in C++
- systemd service files and Pi deployment scripts are included
- Tailscale-based remote access is documented as the preferred connection path

Still intentionally scaffolded for the MVP:

- the Garmin LIDAR-Lite v3HP driver is represented by a mock lidar layer that should be replaced with the real register-level implementation on the Pi
- the Arduino firmware still uses the temporary ASCII control path for motion commands; the framed binary protocol is defined but not yet wired end to end

## Local development

Run the Go backend in simulator mode:

```powershell
powershell -ExecutionPolicy Bypass -File .\deploy\run-go-backend.ps1 -Port 8080
```

Run the original PowerShell prototype if needed:

```powershell
powershell -ExecutionPolicy Bypass -File .\deploy\run-prototype.ps1 -Port 8080
```

## Raspberry Pi deployment target

The intended MVP host layout on the Raspberry Pi is:

- native service: `cliffscanner-edge`
- Docker container: `control-api`
- Tailscale host service: `tailscaled`
- Tailscale Serve publishes `https://<pi-hostname>.<tailnet>.ts.net` to `http://127.0.0.1:8080`

See [docs/pi-deployment.md](/E:/_Data/_TUE/Prism/Codex web/docs/pi-deployment.md) for the Pi deployment path.
