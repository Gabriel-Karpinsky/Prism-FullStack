# Cliff Face Scanner Prototype

This repository contains a full-stack prototype for a cliff face scanner MVP. It is structured so that:

- the hardware-facing layer stays in C++ for deterministic motion control on an Arduino Mega,
- the browser UI supports multiple observers and one active controller,
- the backend exposes a clear contract that can later be implemented in Go, Rust, or C++,
- the current demo can be run locally with PowerShell only, because this workspace does not currently have Go, Node, Docker, or a C++ toolchain installed.

## What is included

- `apps/prototype-server`: a PowerShell control server that simulates the scanner and serves the UI
- `apps/web-ui`: a dependency-free browser UI with live telemetry, control locking, and a live surface map
- `firmware/arduino-mega`: C++ firmware scaffolding for the Arduino Mega
- `proto/scanner/v1`: a protobuf contract for the future production services
- `docs`: architecture notes and the serial protocol
- `deploy`: helper scripts for running the prototype

## Run the prototype

Open PowerShell in the repository root and run:

```powershell
powershell -ExecutionPolicy Bypass -File .\deploy\run-prototype.ps1 -Port 8080
```

Then open:

- `http://localhost:8080`

Use any user name to acquire control. One user may control the scanner at a time while other users monitor the session.

## Prototype features

- shared scanner state across multiple browser clients
- operator lease / control lock
- simulated connect, home, jog, start, pause, resume, stop, e-stop, clear fault
- live telemetry polling
- synthetic live surface mapping on a raster grid
- activity log and scan progress visualization

## Suggested production path

For production, keep the architecture split like this:

- Arduino Mega firmware in C++
- edge daemon in C++ near the hardware
- control API in Go
- browser app in Nuxt or another non-React web framework you prefer
- protobuf as the contract between layers

The prototype server is intentionally simple so the folder structure and interactions are easy to replace later with the production services.
