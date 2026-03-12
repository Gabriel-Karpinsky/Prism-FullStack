# Architecture Notes

## Prototype architecture

The current prototype is intentionally simple:

- a PowerShell server owns the shared scanner state
- a browser app polls the server and renders the control dashboard
- the Arduino Mega firmware scaffold defines the hardware-side responsibilities and serial contract

This lets the demo run in a minimal environment while still preserving the production system boundaries.

## Production architecture

For a production build, keep these layers separate:

1. `firmware/arduino-mega`
   Motion control, homing, limit switches, watchdogs, and compact telemetry.
2. `edge-daemon` in C++
   The sole hardware owner. It speaks serial to the Mega and vendor SDKs to lidar/radar hardware.
3. `control-api` in Go
   Authentication, operator lock, scan sessions, persistence, audit log, and browser-facing APIs.
4. `web-ui`
   Browser-based monitoring and control, including the live map or point cloud viewer.

## Multi-user control model

- only one user may hold the control lease
- observers can watch telemetry and the live map without the lease
- all commands pass through the backend
- the hardware layer never trusts the browser directly
- emergency stop must remain available even if the web stack fails

## Why the Arduino should stay small

The Arduino Mega is a good motion controller but a poor application server. Keep it responsible for:

- target position tracking
- step generation or motion scheduling
- homing and limit switch handling
- heartbeat / watchdog safety behavior
- compact binary telemetry

Do not put these on the Mega:

- raw point cloud aggregation
- browser networking
- JSON-heavy APIs
- multi-user session logic
- long-term scan storage
