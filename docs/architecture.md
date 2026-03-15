# Architecture Notes

## MVP production split

The MVP is now organized around a Raspberry Pi edge host:

1. `firmware/arduino-mega`
   Motion control, homing, limit switches, watchdogs, and compact telemetry.
2. `apps/edge-daemon`
   Native C++ service on the Raspberry Pi. It owns the Arduino link, lidar acquisition, scan state, and the localhost hardware API.
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
- read lidar measurements
- own the live hardware state machine
- translate scan progress into spatial data
- keep the scanner safe if the web stack disconnects

The Go backend should not talk to the Arduino directly. It should talk to the edge daemon over a local API.

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
- emergency stop must remain local and effective even if the web stack fails
- the Arduino remains intentionally small and deterministic
