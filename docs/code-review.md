# Cliff Face Scanner — Codebase Review

> Status: review snapshot, 2026-05-18. A point-in-time audit of correctness,
> error handling, performance, and dead code. File/line references are accurate
> as of this date; verify against current `HEAD` before acting.

This document explains how the system works internally, then evaluates it for
correctness bugs, error handling, performance, and unused code, and ends with a
prioritized action plan.

## 1. How the system works (inner workings)

The system is a **three-tier architecture**. A request flows browser → Go →
C++ → silicon, and a state snapshot flows back the same way.

### Tier 1 — Browser UI (`apps/web-ui/`)

Pure vanilla JS, no framework, no build step. `app.js` does three things:

- **Polls** `GET /api/state` every 700 ms (`app.js:361`), receives a full
  "snapshot" JSON object, and calls `render()` to repaint every widget and the
  canvas heatmap.
- **Sends commands** via `POST /api/command` with a body
  `{user, command, payload}`. Buttons are wired generically by `data-command` /
  `data-jog-axis` attributes (`app.js:340-357`).
- **Manages the motion-config panel** — `GET/PUT /api/config/motion`.

There is no client-side state machine. The UI is a thin renderer: the server's
snapshot is the single source of truth, and the UI just draws whatever it last
received.

### Tier 2 — Go control-api (`apps/control-api/`)

This is a broker, not a brain. `main.go` builds an `http.ServeMux`, serves the
three static UI files, and exposes the JSON API. The key design pattern is the
`apiService` interface (`main.go:16-23`) with **two interchangeable
implementations**, chosen at startup by the `SCANNER_BACKEND` env var
(`main.go:165-172`):

- **`Service`** (`service.go`) — `SCANNER_BACKEND=sim`. A complete *in-process
  simulator*. It has no hardware. It fakes a scan by interpolating progress
  against a wall-clock timer (`updateLocked`, `service.go:292`), fills grid
  cells from a math function `sampleHeight()`, and synthesizes motor
  temperature/current from sine waves (`updateMetricsLocked`, `service.go:422`).
- **`EdgeService`** (`edge_service.go`) — `SCANNER_BACKEND=edge`. The real path.
  It owns *only* the operator lease and a local activity log; every state read
  and command is proxied over HTTP to the C++ daemon via `edgeclient/client.go`.

**The operator lease** is the one piece of real logic in the Go layer.
`Acquire` records `controlOwner` + a 120 s expiry. `requireControlLocked`
(`service.go:358`) gates every mutating command and *renews* the lease on each
successful call (sliding window). Expired leases are cleared lazily on the next
access (`clearExpiredLeaseLocked`). All of this is protected by a single
`sync.Mutex`.

### Tier 3 — C++ edge-daemon (`apps/edge-daemon/`)

This is where the real work happens. The class graph:

- **`EdgeDaemon`** (`edge_daemon.cpp`) — the orchestrator. Owns everything,
  holds the `Snapshot` (`state_`), runs the scan state machine.
- **`HttpServer`** (`http_server.cpp`) — a hand-rolled, single-threaded
  HTTP/1.1 server. Parses the request line + body by string-searching for
  `\r\n\r\n`, dispatches on `method + path`, serializes via nlohmann-json.
- **`MotionController`** + two **`StepperAxis`** — motion planning.
- **`IGpioBackend`** — abstract; either `PigpioGpioBackend` (real, DMA
  waveforms) or `MockGpioBackend` (host builds). Selected by
  `CreateGpioBackend` (`gpio_backend_factory.cpp`).
- **`LidarSensor`** — abstract; `GarminLidarLiteV3HPSensor` (real I²C) or
  `MockLidarSensor`.
- **`SafetySupervisor`** — a watchdog thread.

**How a move physically happens** — the clever core:

1. `StepperAxis::PlanMove` (`stepper_axis.cpp:79`) converts a target angle to
   an integer microstep count, then `GenerateStepTimes` (`stepper_axis.cpp:28`)
   computes a **trapezoidal velocity profile** — a timestamp (µs from move
   start) for *every individual step pulse*. Acceleration phase: `t = √(2i/a)`.
   Cruise: linear. Deceleration: mirror of accel.
2. `MotionController::MoveTo` (`motion_controller.cpp:62`) plans both axes, then
   `MergeAxisPlans` interleaves the two timestamp lists into one sorted stream,
   merging pulses that land in the same microsecond into a combined bitmask.
3. `PigpioGpioBackend::RunMotionWaveform` (`pigpio_gpio_backend.cpp:104`) turns
   each pulse into a pair of `WaveEvent`s (assert edge, deassert edge), converts
   those to `gpioPulse_t`, and hands the whole thing to pigpio's **DMA waveform
   engine**. The DMA hardware clocks out the step pulses with microsecond
   precision *without CPU involvement* — this is why a Pi can drive steppers
   smoothly despite Linux not being a real-time OS. `MoveTo` then busy-waits
   (`gpioWaveTxBusy`) until the DMA finishes.

**The scan state machine** lives in `EdgeDaemon::ScanWorker`
(`edge_daemon.cpp:376`), a dedicated thread. Loop: wait on a condition variable
until `Scanning`; compute the next cell in **boustrophedon order**
(`CoordForIndex` reverses every other row so the head snakes back and forth
instead of carriage-returning — `edge_daemon.cpp:452`); `MoveTo` that cell;
pulse the LIDAR trigger GPIO; read distance over I²C; normalize to a 0–1
"confidence" and store it in the grid. Pause is *cooperative* — the worker
finishes the current cell then re-parks on the CV. Stop/estop set `Stopping`
and abort the in-flight waveform.

**The safety supervisor** (`safety_supervisor.cpp`) is a separate thread
ticking every 100 ms. It (a) pets the systemd watchdog, and (b) enforces a
*host watchdog*: if no `Heartbeat()` arrives within `host_watchdog_ms`
(1500 ms), it latches a `HostWatchdog` fault, aborts motion, and disables the
drivers. Faults are **first-fault-wins** (`TriggerFault` ignores subsequent
faults so the root cause is preserved).

## 2. Correctness bugs (ranked by severity)

### Critical

**B1 — The host-watchdog faults an idle scanner.** ✅ **Fixed in `a3d6a69`.**
This was the `host_watchdog: no host heartbeat in 1500ms` failure seen during
deployment. `SafetySupervisor::Heartbeat()` used to be called *only* from
`EdgeDaemon::ExecuteCommand`; plain state polling (`GetSnapshot`) did not
heartbeat, so an idle scanner latched a fault after 1500 ms.
**Fix applied:** `GetSnapshot()` now calls `safety_->Heartbeat()` — any HTTP
poll feeds the watchdog — and the timeout is only enforced while motion is in
flight.

**B2 — Large moves overflowed the pigpio waveform buffer.** ✅ **Fixed via
waveform chaining.** At `microsteps: 128` (`microsteps_per_deg ≈ 71`), a
full-envelope move generates ~11 000–13 000 `gpioPulse_t` — over the
~12 000-pulse single-waveform limit. The old code returned
`"waveform too large; chaining not yet implemented"` and the move failed.
**Fix applied:** `PigpioGpioBackend::RunMotionWaveform` now splits an oversized
pulse list into chunks of `kWaveMaxPulses` and transmits them with
`gpioWaveChain`, so the motor sees one continuous trapezoidal profile. The
default `microsteps` stays at **128** (the project's drivers support it). A
move that still exceeds the chained ceiling returns a clear
`"move too large for pigpio wave memory"` error instead of a cryptic one.

**B3 — The motion-config "Apply" button breaks the UI.** ✅ **Fixed.**
`main.go` returns `{ok: true, motion: cfg}` on a successful PUT, but `app.js`
passed that whole envelope to `populateMotionFields`, which expects the bare
`{yaw, pitch}` config — `updated.yaw` was `undefined` → `TypeError` → the panel
errored and the fields went blank. **Fix applied:** `applyMotionConfig` now
unwraps `response.motion` (falling back to the bare object so the GET path
still works).

### High

**B4 — Uncaught exception in the HTTP request parser crashes the daemon.** In
`HttpServer::Run`, `std::stoul(m[1].str())` (`http_server.cpp:215`) parses
`Content-Length`. A malformed/huge value throws `std::out_of_range` or
`std::invalid_argument`. This line sits in the recv loop **before** the `try`
block at line 224 — the exception is uncaught, `std::terminate` runs, the
daemon dies. Any client on the tailnet can crash it with one request.
**Fix:** wrap the parse, clamp the value.

**B5 — No request-body cap → memory-exhaustion DoS.** The recv loop appends
into `raw` with no upper bound (`http_server.cpp:208`). A client advertising
`Content-Length: 9999999999` makes the daemon allocate until OOM. The Go server
is similar — `http.ListenAndServe` with a default `http.Server` has **no
`ReadTimeout`/`WriteTimeout`** (`main.go:160`), so it's open to slowloris.

**B6 — `MarkPositionUnknown` is a decorative safety mechanism.** On an aborted
waveform, `MotionController::MoveTo` calls `yaw_.MarkPositionUnknown()`
(`motion_controller.cpp:105`) — but the motor *did* move partway, and
`current_microsteps_` was never updated (only `Commit` updates it). So the
tracked position is now wrong. `position_known_` records this... and **nothing
ever reads it**. The next `MoveTo` plans from the stale position and silently
drives to the wrong place. **Fix:** after an abort, force the operator to
re-home before any further move; have `PlanMove`/`ExecuteCommand` refuse moves
while `!position_known()`.

**B7 — `fmt.Errorf` with a non-constant format string.** ✅ **Fixed.**
`client.go` passed daemon-supplied error text straight into `fmt.Errorf` as the
format string; a `%` in that text produced `%!s(MISSING)` garbage. **Fix
applied:** both call sites now use `errors.New(...)`; `go vet` is clean.

### Medium

**B8 — Single-threaded daemon: a long move blocks all polling.** `home` and
`jog` call `MotionController::MoveTo` *synchronously inside `ExecuteCommand`*,
which runs on the daemon's only HTTP thread. A multi-second home blocks
`/api/hardware/state` for its whole duration. The Go client times out at 5 s
(`client.go:36`) → any move longer than 5 s returns an error to the UI even
though the move is fine. (`start_scan` is correctly offloaded to a worker; only
the manual moves block.)

**B9 — Malformed JSON returns HTTP 409.** `decodeJSON` failures in
`/api/control/*` and `/api/command` are written as `409 Conflict`
(`main.go:79`, etc.) — a parse error is `400 Bad Request`. 409 is for lease
conflicts. This conflates client-error categories and confuses the UI.

**B10 — No authentication anywhere.** The "lease" is just a username string;
there is no token. On a Tailscale network that is *probably* acceptable, but
this is a machine that physically moves — it should be a conscious decision,
not an accident.

**B11 — Transient I²C glitch kills the whole scan.** `ReadDistanceMeters`
returns `NaN` on any I²C error; the scan worker immediately `FailLocked` +
`TriggerFault(LidarFault)` and latches (`edge_daemon.cpp:426-431`). One noisy
read on cell 900 of 1152 throws away the whole scan. No retry.

## 3. Error-handling evaluation & strategy

**Current state:** error *propagation* is decent — Go returns `error`, C++
returns result structs with `success`/`error` strings, and faults latch
correctly. The problems are at the **boundaries** and in **granularity**.

Weaknesses:

- The C++ HTTP layer is the weak point: unguarded parsing (B4), no body cap
  (B5), no per-request timeout for a stalled client (the 1 s `SO_RCVTIMEO` just
  loops — a client that sends headers then nothing holds the *only* thread
  until daemon shutdown).
- Failure is **all-or-nothing**: any LIDAR or motion hiccup latches a hard
  fault. There is no notion of a *transient* vs *terminal* error, no retry, no
  degraded mode.
- `EdgeService.lastError` is set but **never surfaced** — dead field.
- No structured logging — everything is `std::cerr` / `log.Printf` / the
  in-memory activity ring. No severity-filterable, greppable log for
  post-mortem.

**Strategy (priority order):**

1. **Harden the daemon's HTTP intake** — wrap Content-Length parsing, cap the
   body (e.g. 64 KiB), add a hard deadline for the whole request.
2. **Add Go server timeouts** — replace `http.ListenAndServe` with an explicit
   `&http.Server{ReadHeaderTimeout, ReadTimeout, WriteTimeout, IdleTimeout}`.
3. **Classify errors as transient vs terminal.** Give I²C reads and
   `WriteRegister`/`ReadRegister` a small bounded retry (3×, ~5 ms apart)
   before declaring `NaN`. Only latch `LidarFault` after retries exhaust. Same
   for a single failed move.
4. **Make `position_known` load-bearing** (B6) — refuse moves on an axis with
   unknown position; require `home` to clear it.
5. **Fix the error-status taxonomy** — 400 for malformed input, 409 only for
   lease conflicts, 502/503 for edge-daemon unreachable.
6. **Introduce structured logging** with levels; keep the in-memory ring for
   the UI but also write to stderr/journald in a parseable format.

## 4. Performance evaluation & strategy

For a single-operator bench instrument the performance is *adequate*, but there
is clear waste:

- **Full-grid JSON on every poll.** Every 700 ms the UI fetches a snapshot that
  includes the entire `48×24` (or larger) grid — ~1150 doubles — even when zero
  cells changed. With the edge backend that is also a full localhost HTTP
  round-trip carrying the grid. `writeJSON` uses `json.MarshalIndent`
  (`main.go:200`) — pretty-printing every response for no reason.
- **The 2 ms busy-wait in `RunMotionWaveform`** (`pigpio_gpio_backend.cpp:151`)
  is fine for abort latency but burns a thread spinning; acceptable given DMA
  does the real work.
- **Canvas redraw** repaints all ~1150 cells every 700 ms regardless of change.
- **Single-threaded daemon** (B8) — throughput ceiling and the blocking-move
  problem.

**Strategy:**

1. **Stop shipping the grid on every poll.** Add a grid version/generation
   counter; the UI sends `?since=N` and gets `{unchanged:true}` or just the
   changed cells. Biggest single win — cuts payload ~95% for the common case.
2. **Drop `MarshalIndent`** → `json.Marshal`. Pretty output belongs in a debug
   endpoint only.
3. **Split the daemon's slow path off the HTTP thread**, or give the daemon a
   tiny thread pool (2–4 threads). `home`/`jog` should return immediately with
   a "moving" state and let the UI poll for completion, as `start_scan`
   already does.
4. **Consider Server-Sent Events** instead of 700 ms polling — the daemon
   pushes a snapshot when state changes. Removes idle traffic entirely.
   (Bigger change; do after #1.)
5. Minor: redraw only changed cells in `drawSurfaceMap`.

## 5. Dead / unused / vestigial code

| Item | Location | Verdict |
|---|---|---|
| `proto/scanner/v1/scanner.proto` | gRPC `ScannerControlService` contract | **Fully dead.** Nothing generates or uses it; the system runs JSON. Delete or move to a `docs/future/` folder. |
| `radarFps` / `radar_fps` | `Metrics` in `types.go:23`, `types.hpp:29`, wire JSON | There is no radar. Vestigial; the edge daemon hardcodes it to `0`. Remove from the schema. |
| `packetsDropped` | `Metrics` | Always `0`, never computed. Remove or implement. |
| Fabricated metrics | `EdgeDaemon::UpdateMetricsLocked` (`edge_daemon.cpp:505`) | `motor_temp_c`, `motor_current_a`, `lidar_fps`, `latency_ms` are **made-up numbers**, even on real hardware. The UI displays them as if sensor readings. Either wire real telemetry or label them as estimates. Actively misleading. |
| `IGpioBackend::IsMotionBusy()` | interface + both backends | `MotionController` tracks its own `busy_` atomic and never calls `gpio_->IsMotionBusy()`. Redundant. |
| `SetStatusLed` / `status_led` | backends, config | Defined, configured, never called. |
| `tick_interval_ms`, `status_broadcast_interval_ms` | `ServiceConfig`, loaded from JSON | Parsed and stored, never read. |
| `EdgeService.lastError` | `edge_service.go:24` | Set in three places, read nowhere. |
| The `Service` simulator | `service.go` (~570 lines) | *Not* dead — it is the dev/demo backend. But it is a **parallel reimplementation** of the daemon's scan logic that will drift (its grid is hardcoded `48×24`; the daemon resizes by resolution; `set_resolution` rules differ). Keep it, but treat it as a known sync-drift liability. |
| `firmware/arduino-mega` | — | Already removed from the tree. The streamline goal is done here. |

## 6. Repository hygiene

**`.gocache/` is committed to git: 1339 files, 101 MB.** It is the Go build
cache. It is *listed in `.gitignore`* (`.gitignore:1`) but was committed before
the ignore rule existed, so the ignore is silently doing nothing for it. This
bloats every clone and CI checkout.

```sh
git rm -r --cached .gocache
git commit -m "Remove committed Go build cache"
```

The history still contains it — a `git filter-repo` purge is optional but worth
it before this repo is shared widely.

## 7. Action plan (priority order)

**Done:** B1 (watchdog), B2 (waveform chaining), B3 (Apply button), B7
(`fmt.Errorf`). Remaining:

1. **`git rm -r --cached .gocache`** — 30 seconds, removes 100 MB of noise.
2. **B4 + B5** — harden the daemon's HTTP parser; add Go server timeouts. The
   daemon currently crashes on a malformed request.
3. **B6** — make `position_known` actually block moves. A *physical safety* gap
   on a machine with motors.
4. **B9** — return HTTP 400 (not 409) for malformed request bodies.
5. **B8 + B11** — offload blocking moves; add bounded I²C retries.
6. **Performance #1** — stop shipping the full grid on every poll.
7. **Delete dead code** (§5) — `proto/`, `radarFps`, `packetsDropped`, dead
   config fields.
8. **Add tests.** There is currently **zero test coverage** in any of the
   three languages. At minimum: unit-test `GenerateStepTimes` (the trapezoidal
   profile — pure, math-heavy, easy to test, and a bug there moves the motor
   wrong), the lease state machine, and `CoordForIndex` boustrophedon logic.
