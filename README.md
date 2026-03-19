# Cliff Face Scanner MVP

This repository contains a deployable MVP stack for the cliff-face scanner.

The intended lab deployment is:

- Arduino Mega for deterministic yaw/pitch motion control and Garmin trigger output
- Raspberry Pi native C++ edge daemon for Arduino serial control, Garmin LIDAR-Lite v3HP reads, and scan orchestration
- Go control API in Docker on the Raspberry Pi
- Browser UI served directly by the Go backend
- Tailscale on the Raspberry Pi host for remote operator access when the lab network blocks local connections

## Repository layout

- `apps/control-api`: Go backend that serves the browser UI and operator API
- `apps/edge-daemon`: native C++ Raspberry Pi service that owns the Arduino and lidar hardware path
- `apps/web-ui`: dependency-free browser UI
- `firmware/arduino-mega`: Arduino Mega firmware for dual step/dir control and trigger output
- `proto/scanner/v1`: future protobuf contract if the localhost JSON boundary is later upgraded
- `deploy/pi`: Raspberry Pi deployment assets for systemd, Docker Compose, and Tailscale
- `docs`: architecture, packaging, protocol, and Pi deployment notes

## Current MVP behavior

Implemented now:

- Arduino firmware drives two stepper axes through external step/dir drivers at 128 microstepping
- Arduino exposes a line-based serial command/status protocol with heartbeat watchdog and trigger output
- edge daemon sequences raster scans as `move -> settle -> trigger -> read lidar -> commit cell`
- Garmin LIDAR-Lite v3HP reads are implemented on the Pi over Linux I2C with a mock fallback for bench work
- Go backend runs in Docker and talks to the edge daemon over localhost
- Tailscale is the intended remote access path for lab operation

Important assumptions to verify in the lab:

- Arduino pin assignments in [HardwareConfig.h](/E:/_Data/_TUE/Prism/Codex web/firmware/arduino-mega/include/HardwareConfig.h)
- stepper driver DIP switches are physically set to `128` microsteps
- gear ratios in [HardwareConfig.h](/E:/_Data/_TUE/Prism/Codex web/firmware/arduino-mega/include/HardwareConfig.h) match the gantry mechanics
- Garmin wiring and I2C bus/address match [edge-daemon.env.example](/E:/_Data/_TUE/Prism/Codex web/deploy/pi/edge-daemon.env.example)

## Local development

Run the Go backend against the in-process simulator:

```powershell
powershell -ExecutionPolicy Bypass -File .\deploy\run-go-backend.ps1 -Port 8080
```

## Raspberry Pi deployment target

The intended MVP host layout on the Raspberry Pi is:

- native service: `cliffscanner-edge`
- Docker container: `control-api`
- Tailscale host service: `tailscaled`
- Tailscale Serve publishes `https://<pi-hostname>.<tailnet>.ts.net` to `http://127.0.0.1:8080`

See [docs/pi-deployment.md](/E:/_Data/_TUE/Prism/Codex web/docs/pi-deployment.md) for the Pi deployment flow and [docs/lab-bringup.md](/E:/_Data/_TUE/Prism/Codex web/docs/lab-bringup.md) for the first bench test checklist.
