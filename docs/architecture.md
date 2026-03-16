# Architecture Notes

## MVP production split

The MVP is organized around a Raspberry Pi edge host:

1. `firmware/arduino-mega`
   Dual-axis step/dir control on the Arduino Mega with watchdog protection and a Garmin trigger pulse output.
2. `apps/edge-daemon`
   Native C++ service on the Raspberry Pi. It owns the Arduino serial link, Garmin lidar reads, scan state, and the localhost hardware API.
3. `apps/control-api`
   Go backend for browser-facing APIs, operator lease logic, and serving the web UI.
4. `apps/web-ui`
   Browser dashboard for control and live monitoring.
5. `tailscaled` on the Raspberry Pi host
   Secure remote connectivity when the lab LAN does not allow direct local access.

## Why the edge daemon exists

The edge daemon separates hardware timing and fault handling from the web stack.

It is the only process that should:

- open the Arduino serial port
- send motion/trigger commands to the Arduino
- read lidar measurements from the Pi I2C bus
- own the live hardware state machine
- translate scan progress into spatial data
- keep the scanner safe if the web stack disconnects

The Go backend talks to the edge daemon over localhost JSON. It does not touch the Arduino or lidar devices directly.

## Scan loop

For the current MVP, one raster sample is acquired like this:

1. Go backend asks the edge daemon to `start_scan`.
2. The edge daemon sends `START_SCAN` to the Arduino.
3. For each raster cell, the edge daemon sends `MOVE yaw pitch`.
4. The edge daemon polls `STATUS` until motion settles.
5. The edge daemon sends `TRIGGER` to the Arduino.
6. The edge daemon reads the Garmin LIDAR-Lite v3HP over I2C.
7. The edge daemon writes that sample into the current grid.
8. The backend serves the updated grid to connected browsers.

## Network and hosting model

For the Raspberry Pi deployment:

- the edge daemon binds to `127.0.0.1:9090`
- the Go backend container binds to `127.0.0.1:8080` using Docker host networking
- Tailscale runs on the Pi host and publishes the Go backend over the tailnet
- the lab LAN never needs direct access to the scanner ports

That gives this request path:

`Browser -> Tailscale -> Go backend -> edge daemon -> Arduino/LIDAR`

## Safety model

- only one user may hold the control lease
- all browser commands terminate at the Go backend first
- the edge daemon remains the sole hardware owner
- the Arduino watchdog faults the motion layer if host heartbeats stop
- emergency stop must remain local and effective even if the web stack fails
- the Arduino remains intentionally small and deterministic
