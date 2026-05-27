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

Merged from **`continous_scan`** into `streamline-claude` → `master` (2026-05-28).
**⚠ Not yet bench-validated on real hardware** — merged at the operator's request;
still needs a real-Pi `HAS_PIGPIO=1` build, a browser smoke of the new rendering
path, and a cautious first sweep (start with low `sweep_max_speed_deg_s` /
`sweep_accel_deg_s2`):
- Continuous sweep scanning (`scan.mode=sweep`, default). Verified via WSL mock build +
  runtime smoke (scan ran to completion, no faults).
- Note: `IGpioBackend::IsMotionBusy()` is **load-bearing** (the sweep loop polls it).
  Do not "remove as dead".
- **Bug fixes (2026-05-28):** B4 (Content-Length crash), B5 (request-body cap + Go server
  timeouts), B6 (`position_known` now blocks moves; `home` re-zeros), B8 (`home`/`jog` run
  on an async move worker), B9 (400 vs 409), B11 (3× LIDAR read retry in step mode). B10
  (auth) deferred by decision.
- **Online-rendering rework (2026-05-28):** incremental grid (`/api/state?since=&gen=` →
  `gridUpdate` deltas; grid held client-side as a `Float32Array`; dirty-cell redraw +
  marker overlay canvas); fabricated telemetry removed (`Metrics` is now an empty
  placeholder; `radarFps`/`packetsDropped` gone); `json.Marshal` instead of `MarshalIndent`;
  sweep/step toggle in the UI via a new `set_scan_mode` command.
- All validated via WSL mock builds + `go build`/`vet` + runtime mock smokes. See
  `docs/code-review.md` §7–§8 for the full status and review findings.

## Future direction / backlog

Priority order. Sourced from `docs/code-review.md` and the operator's notes.

### Correctness/safety bugs
Done on `continous_scan` (pending merge): ~~B4~~, ~~B5~~, ~~B6~~, ~~B8~~, ~~B9~~, ~~B11~~.
Still open:
- **B10** — decide on auth: the lease is just a username string with no token. Either add
  a token or make "tailnet-only" an explicit, documented decision. **Deferred by decision.**
- **Follow-ups from the 2026-05-28 review** (all 🟢 low, see `code-review.md` §8): the
  daemon's whole-request timeout is still just the 1 s `SO_RCVTIMEO` loop, not a hard
  deadline; extend the transient-vs-terminal retry (B11) to single failed moves + the I²C
  register helpers; structured logging is still unbuilt (`EdgeService.lastError` set,
  never read).

### Web UI rework — **DONE on `continous_scan`** (pending merge)
Folded continuous-scan into the UI as one effort:
- ~~Incremental grid~~ — done: `/api/state?since=&gen=` → compact `gridUpdate` deltas; grid
  held client-side as a flat `Float32Array`; dirty-cell redraw + marker overlay canvas.
  Server-side find is still O(W×H) per poll (no dirty-set) — fine under the 300k clamp.
- ~~sweep/step toggle~~, ~~remove mock telemetry~~, ~~`json.Marshal`~~ — all done.
- Kept it dependency-free (hand-rolled, no framework). **Still open: SSE** to replace
  700 ms polling now that the incremental endpoint exists (optional, bigger change).

### Dead / unused code to remove
- `proto/scanner/v1/scanner.proto` — unused gRPC contract.
- ~~`radarFps`/`packetsDropped`/fabricated metrics~~ — removed with the telemetry struct.
- `SetStatusLed`/`status_led` — defined/configured, never driven.
- `tick_interval_ms`, `status_broadcast_interval_ms` — parsed, never read.
- `EdgeService.lastError` — set, never read.
- **Do NOT** remove `IsMotionBusy()` — it is now load-bearing for continuous scan.

### Repo hygiene
- `.gocache/` (≈101 MB, 1339 files) is committed despite being in `.gitignore`. Run
  `git rm -r --cached .gocache && git commit`; consider a `git filter-repo` history purge.

### Testing
- No tests exist. Start with the pure, high-value units: `GenerateStepTimes`
  (trapezoidal profile), the lease state machine, `CoordForIndex` (boustrophedon),
  the resolution→stride→grid derivation, and now the `GetGridUpdate` /
  `buildGridUpdateLocked` delta logic (generation bump → full; version cursor →
  delta; unfilled cells skipped).
