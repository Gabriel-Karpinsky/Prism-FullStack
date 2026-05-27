# CLAUDE.md

Guidance for Claude Code working in this repo. Read [`docs/README.md`](docs/README.md)
for the full documentation set; this file is the operational quick-reference plus
the active backlog.

## What this is

**Cliff Face Scanner** — a 2-axis LIDAR gantry, Pi-native (no microcontroller).

```
Browser ──HTTP──► control-api (Go, :8080) ──HTTP──► edge-daemon (C++, :9090) ──► GPIO/I²C ──► 2× TB6600 steppers + Garmin LIDAR-Lite v3HP
```

- `apps/edge-daemon` (C++17) — owns all GPIO + I²C, the scan state machine, and the
  safety watchdog. pigpio DMA waveforms drive the steppers.
- `apps/control-api` (Go, stdlib only) — operator-lease broker, serves the UI, and
  proxies to the daemon. Has a built-in simulator (`SCANNER_BACKEND=sim`).
- `apps/web-ui` — dependency-free vanilla JS; polls `/api/state` every 700 ms.

Architecture detail: [`docs/architecture.md`](docs/architecture.md). Inner workings &
data flow: [`docs/data-flow.md`](docs/data-flow.md). Wiring & deploy:
[`docs/hardware-setup.md`](docs/hardware-setup.md). Known issues:
[`docs/code-review.md`](docs/code-review.md).

## Branch workflow (IMPORTANT)

- **`streamline-claude` is the working branch — make all commits here.**
- After each commit (or batch) on `streamline-claude`, **merge it into `master`**.
  Never commit directly on `master`; it only ever receives merges from
  `streamline-claude`.
- Feature work branches off `streamline-claude` (e.g. `continous_scan`). Validate it,
  then merge the feature branch back into `streamline-claude`, then `streamline-claude`
  → `master`.
- The `claude/<name>` worktrees are throwaway.
- Worktrees live under `.claude/worktrees/`; each checks out a different branch (git
  forbids the same branch in two worktrees).

## Build & test

```bash
# Go control-api (works on the dev host)
cd apps/control-api && go build ./... && go vet ./...

# Edge-daemon ON THE PI (needs libpigpio):
cd apps/edge-daemon && make HAS_PIGPIO=1

# Edge-daemon mock build (no Pi; HAS_PIGPIO unset → mock GPIO/LIDAR backend)
cd apps/edge-daemon && make
```

**There is no C++ toolchain on the Windows dev host, but WSL has `g++`.** To
compile-check / smoke-test the daemon without a Pi, build the mock set in WSL
(exclude the pigpio backend, which needs `libpigpio`):

```bash
wsl bash -lc "mkdir -p /tmp/nl/nlohmann && \
  curl -fsSL https://raw.githubusercontent.com/nlohmann/json/v3.11.3/single_include/nlohmann/json.hpp -o /tmp/nl/nlohmann/json.hpp && \
  cd '/mnt/e/.../apps/edge-daemon' && \
  g++ -std=c++17 -Wall -Wextra -pthread -Iinclude -I/tmp/nl \
    src/edge_daemon.cpp src/gpio_backend_factory.cpp src/hardware_config.cpp \
    src/http_server.cpp src/lidar_sensor.cpp src/main.cpp src/mock_gpio_backend.cpp \
    src/motion_controller.cpp src/safety_supervisor.cpp src/stepper_axis.cpp -o /tmp/edge"
```
WSL recycles `/tmp` between invocations — fetch nlohmann and build in the **same**
`wsl bash -lc "..."` call. Set `simulate_hardware: true` (via a `PRISM_HARDWARE_CONFIG`
file) to get the mock LIDAR for a runtime smoke test. The `pigpio_gpio_backend.cpp`
path can only be compiled on the Pi.

**No automated tests exist yet** (see backlog).

## Conventions

- Wire JSON is **camelCase**; the daemon's `/api/config` uses **snake_case** to match
  `hardware.json`.
- The `Snapshot` struct is the single contract across all three tiers — keep the Go
  (`scanner.Snapshot`/`ScanSettings`) and C++ (`types.hpp`) shapes in sync, and the
  daemon's `SnapshotToJson`.
- Resolution presets are **hardware-grounded**: `coarse|standard|fine|max` map to a
  microstep stride; the grid size is derived, not hardcoded.
- Scan motion has two modes (`scan.mode`): `step` (stop-and-shoot) and `sweep`
  (continuous). Sweep speed is LIDAR-limited.
- Lock discipline in the daemon: `mutex_` guards `state_`/`scan_state_` and is **never**
  held across `MoveTo`/sweep/LIDAR reads. Don't acquire `MotionController`'s lock while
  holding `EdgeDaemon::mutex_`.

## Current state (as of this cycle)

Done on `streamline-claude` / `master`:
- Docs restructured into `docs/` with an index; deprecated Arduino-era docs removed.
- **B1** host-watchdog-on-idle, **B2** waveform chaining (`microsteps` default 128),
  **B3** motion-config Apply button, **B7** `fmt.Errorf` — all fixed.
- Resolution model reworked to hardware-grounded presets.

On the **`continous_scan`** branch (NOT yet merged — needs bench validation at speed):
- Continuous sweep scanning (`scan.mode=sweep`, default). Verified via WSL mock build +
  runtime smoke (scan ran to completion, no faults). Needs a real-Pi `HAS_PIGPIO=1`
  build and a cautious first run (start with low `sweep_max_speed_deg_s`/`sweep_accel_deg_s2`).
- Note: this branch makes `IGpioBackend::IsMotionBusy()` **load-bearing** (the sweep loop
  polls it). Do not "remove as dead" once merged.

## Future direction / backlog

Priority order. Sourced from `docs/code-review.md` and the operator's notes.

### Remaining correctness/safety bugs
- **B4** — guard the `Content-Length` parse in `http_server.cpp` (an unparseable/huge
  value throws *before* the `try` block → uncaught → daemon dies). Wrap + clamp.
- **B5** — cap request body size in the daemon; add `ReadTimeout`/`WriteTimeout`/
  `ReadHeaderTimeout` to the Go `http.Server` (currently slowloris-open).
- **B6** — make `position_known` load-bearing: after an aborted move/sweep, refuse
  further moves until the operator re-homes (today it's recorded but never checked).
- **B8** — get `home`/`jog` off the daemon's single HTTP thread (return "moving" + poll,
  like `start_scan`, or add a small thread pool). Long moves currently block all polling.
- **B9** — return HTTP **400** (not 409) for malformed request bodies; reserve 409 for
  lease conflicts.
- **B10** — decide on auth: the lease is just a username string with no token. Either add
  a token or make "tailnet-only" an explicit, documented decision.
- **B11** — bounded retry (e.g. 3×) on transient I²C errors before latching `LidarFault`;
  one noisy read shouldn't discard a whole scan.

### Web UI rework (do as one coherent effort; **fold continuous-scan into the UI here**)
The full-grid-on-every-poll model breaks down now that grids can be huge (fine/max
density) and `sweep` fills cells continuously.
- **Incremental grid** (biggest win): stop shipping the whole grid each poll. Add a grid
  generation/version counter on the daemon; client sends `?since=N` and gets only changed
  cells (or `{unchanged:true}`). Required for large grids + continuous fill.
- **Client framework**: keep it dependency-light. Recommended: a thin reactive layer
  (`lit` web components, or `preact` + signals) — or hand-rolled. For minimal compute,
  hold the grid client-side as a flat `Float32Array`, apply deltas, and **redraw only
  dirty cells** on the canvas. Consider **SSE** to push snapshots instead of 700 ms
  polling, but only after the incremental endpoint exists.
- Add a **sweep/step scan-mode toggle** to the UI (and surface sweep velocity/density).
- **Remove mock telemetry**: strip the fabricated motor temp/current/fps/latency from the
  daemon and UI; leave a minimal/empty telemetry struct as a placeholder for real sensor
  readings later. (Ties to the dead-code item below.)
- Drop `json.MarshalIndent` → `json.Marshal` (pretty-print only on a debug endpoint).

### Dead / unused code to remove
- `proto/scanner/v1/scanner.proto` — unused gRPC contract.
- `radarFps`/`radar_fps` — there is no radar.
- `packetsDropped` — always 0.
- Fabricated metrics in `EdgeDaemon::UpdateMetricsLocked` — see telemetry removal above.
- `SetStatusLed`/`status_led` — defined/configured, never driven.
- `tick_interval_ms`, `status_broadcast_interval_ms` — parsed, never read.
- `EdgeService.lastError` — set, never read.
- **Do NOT** remove `IsMotionBusy()` — it becomes used by continuous scan.

### Repo hygiene
- `.gocache/` (≈101 MB, 1339 files) is committed despite being in `.gitignore`. Run
  `git rm -r --cached .gocache && git commit`; consider a `git filter-repo` history purge.

### Testing
- No tests exist. Start with the pure, high-value units: `GenerateStepTimes`
  (trapezoidal profile), the lease state machine, `CoordForIndex` (boustrophedon),
  and the resolution→stride→grid derivation.
