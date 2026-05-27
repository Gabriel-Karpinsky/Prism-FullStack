# Cliff Face Scanner — Codebase Review

> Status: review snapshot, 2026-05-18; **updated 2026-05-28** after the
> `continous_scan` work landed B4/B5/B6/B8/B9/B11 and the online-rendering
> rework (incremental grid, telemetry removal, sweep/step toggle). A
> point-in-time audit of correctness, error handling, performance, and dead
> code. Original file/line references are as of 2026-05-18 — verify against
> current `HEAD` before acting. Findings from the 2026-05-28 cycle are in §8.

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

**B4 — Uncaught exception in the HTTP request parser crashes the daemon.**
✅ **Fixed (`continous_scan`, `289fc99`).** `std::stoul` on `Content-Length` ran
in the recv loop *before* the request `try` block, so a malformed/huge value
threw uncaught → `std::terminate`. Any tailnet client could crash the daemon
with one header. **Fix applied:** the parse is now `std::stoull` wrapped in
`try`/`catch`; an unparseable/oversized value is rejected with HTTP 413 instead
of crashing.

**B5 — No request-body cap → memory-exhaustion DoS.** ✅ **Fixed
(`continous_scan`, `289fc99`).** The daemon recv loop now bounds buffered bytes
at 1 MiB and rejects an oversized/unparseable `Content-Length` with 413. The Go
server replaced bare `http.ListenAndServe` with an explicit `&http.Server{...}`
carrying `ReadHeaderTimeout`/`ReadTimeout`/`WriteTimeout`/`IdleTimeout` +
`MaxHeaderBytes`, and bodies are wrapped in `http.MaxBytesReader` — closing both
the slowloris and the unbounded-read holes.

**B6 — `MarkPositionUnknown` is a decorative safety mechanism.** ✅ **Fixed
(`continous_scan`, `289fc99`).** `MotionController::MoveTo` now refuses to plan
while either axis reports `!position_known()` (the tracked position is stale
after an aborted move). `Home()` is the recovery path: with no endstops it
*declares* the current pose as zero (`StepperAxis::ResetToZero`) rather than
driving to a stale target, restoring tracking. `jog`/`start_scan` fail fast with
a "re-home" message and no fault latch.

**B7 — `fmt.Errorf` with a non-constant format string.** ✅ **Fixed.**
`client.go` passed daemon-supplied error text straight into `fmt.Errorf` as the
format string; a `%` in that text produced `%!s(MISSING)` garbage. **Fix
applied:** both call sites now use `errors.New(...)`; `go vet` is clean.

### Medium

**B8 — Single-threaded daemon: a long move blocks all polling.** ✅ **Fixed
(`continous_scan`, `2d64f0a`).** `home`/`jog` now run on a dedicated move worker
(mirroring `start_scan`) and return immediately with `mode:"manual"`; the UI
polls `/api/state` for the `idle`/`fault` transition. This also removes a latent
hazard — a multi-second synchronous home blocked polling *and* the safety
heartbeat, risking a spurious `host_watchdog` fault. Added scan↔move mutual
exclusion; `estop`/`Stop()` now abort and reap an in-flight manual move (estop
previously didn't even `AbortMotion`).

**B9 — Malformed JSON returns HTTP 409.** ✅ **Fixed (`continous_scan`,
`289fc99`).** Decode failures on `/api/control/acquire`,`/release`,`/api/command`
now return `400 Bad Request`; `409 Conflict` is reserved for genuine lease
conflicts.

**B10 — No authentication anywhere.** ⏸ **Deferred (decision pending).** The
"lease" is still just a username string with no token. On a Tailscale network
that is *probably* acceptable, but it should be a conscious, documented
decision. Left open intentionally; revisit before any non-tailnet exposure.

**B11 — Transient I²C glitch kills the whole scan.** ✅ **Fixed
(`continous_scan`, `2d64f0a`).** Step-mode reads now retry up to 3× (re-trigger
+ short settle) before latching `LidarFault`. Sweep mode already tolerated
dropped reads by skipping the cell.

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

1. ✅ **Harden the daemon's HTTP intake** (B4/B5) — Content-Length parse is
   wrapped + clamped; body capped at 1 MiB with a 413 reject. *(Open nuance: a
   client that sends headers then dribbles still relies on the 1 s `SO_RCVTIMEO`
   loop rather than a single whole-request deadline — acceptable, not a hard
   deadline.)*
2. ✅ **Add Go server timeouts** (B5) — explicit `&http.Server{ReadHeaderTimeout,
   ReadTimeout, WriteTimeout, IdleTimeout, MaxHeaderBytes}` + `MaxBytesReader`.
3. ◑ **Classify errors as transient vs terminal.** *Partial:* step-mode LIDAR
   reads now retry 3× before latching (B11). Still worth extending to a single
   failed move and to the I²C register helpers.
4. ✅ **Make `position_known` load-bearing** (B6) — moves refused on an axis with
   unknown position; `home` re-establishes the datum.
5. ✅ **Fix the error-status taxonomy** (B9) — 400 for malformed input, 409 only
   for lease conflicts; edge-daemon-unreachable already surfaces as a fault via
   `decorateSnapshotLocked`.
6. **Introduce structured logging** with levels — *still open*; keep the
   in-memory ring for the UI but also write to stderr/journald parseably.
   (`EdgeService.lastError` is still set-but-never-read — fold into this.)

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

1. ✅ **Stop shipping the grid on every poll.** *Done (`86a7eb2`).* Grid moved
   out of `Snapshot`; the client holds it as a `Float32Array` and polls
   `/api/state?since=<version>&gen=<generation>`, receiving a compact
   `gridUpdate` (changed cells only, or all filled cells when the generation is
   stale). Steady-state idle polls now ship zero cells. See §8 for a note on the
   O(N) server-side scan.
2. ✅ **Drop `MarshalIndent`** → `json.Marshal`. *Done (`8658444`).*
3. ✅ **Split the daemon's slow path off the HTTP thread.** *Done (B8,
   `2d64f0a`).* `home`/`jog` return immediately and run on a move worker.
4. **Consider Server-Sent Events** instead of 700 ms polling — the daemon
   pushes a snapshot when state changes. Removes idle traffic entirely. Still
   open; the incremental endpoint (#1) is the prerequisite and is now in place.
5. ✅ **Redraw only changed cells.** *Done (`86a7eb2`).* The heatmap repaints
   only delta cells; the head marker moved to a stacked overlay canvas.

## 5. Dead / unused / vestigial code

| Item | Location | Verdict |
|---|---|---|
| `proto/scanner/v1/scanner.proto` | gRPC `ScannerControlService` contract | **Fully dead.** Nothing generates or uses it; the system runs JSON. Delete or move to a `docs/future/` folder. |
| `radarFps` / `radar_fps` | `Metrics` | ✅ **Removed (`8658444`)** with the telemetry struct. |
| `packetsDropped` | `Metrics` | ✅ **Removed (`8658444`)** with the telemetry struct. |
| Fabricated metrics | `EdgeDaemon::UpdateMetricsLocked` | ✅ **Removed (`8658444`).** `UpdateMetricsLocked` + the sim's `updateMetricsLocked`/`round1` are gone; `Metrics` is now an empty placeholder struct (`{}` on the wire). The misleading motor temp/current, lidar/radar fps and latency values no longer exist. |
| `IGpioBackend::IsMotionBusy()` | interface + both backends | ⚠️ **No longer dead — now load-bearing.** The continuous-sweep loop polls it via `MotionController::SweepBusy()` to detect waveform completion. **Do not remove.** (Note: `MotionController::is_busy()` — the separate atomic — is still used by the safety supervisor.) |
| `SetStatusLed` / `status_led` | backends, config | Defined, configured, never called. *Still open.* |
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
(`fmt.Errorf`); **and on `continous_scan`:** B4, B5, B6, B8, B9, B11, plus
Performance #1/#2/#3/#5 (incremental grid, compact JSON, off-thread moves,
dirty-cell redraw) and the telemetry removal. All validated via WSL mock builds
+ `go build`/`vet` + runtime mock smokes; **pending a real-Pi `HAS_PIGPIO=1`
build, a browser run, and merge to `streamline-claude` → `master`.**

Remaining:

1. **`git rm -r --cached .gocache`** — 30 seconds, removes 100 MB of noise.
2. **B10** — decide on auth (token vs. documented tailnet-only). Deferred.
3. **Performance #4 — Server-Sent Events** (optional) now that the incremental
   endpoint exists: push snapshots instead of 700 ms polling.
4. **Delete remaining dead code** (§5) — `proto/`, `SetStatusLed`/`status_led`,
   `tick_interval_ms`/`status_broadcast_interval_ms`, `EdgeService.lastError`.
5. **Add tests.** Still **zero test coverage**. At minimum: `GenerateStepTimes`
   (trapezoidal profile — pure, math-heavy), the lease state machine,
   `CoordForIndex` boustrophedon, the resolution→stride→grid derivation, and
   now **`GetGridUpdate` / `buildGridUpdateLocked`** delta logic (generation
   bump → full; version cursor → delta; unfilled cells skipped).

## 8. Review cycle 2026-05-28 — the `continous_scan` changes

A self-review of the five commits that landed B4/B5/B6/B8/B9/B11 and the
online-rendering rework. **No critical or high-severity new issues.** The async
move worker (B8) was scrutinised for concurrency: the daemon's HTTP server is
single-threaded so command handlers never race each other; worker↔HTTP state is
mediated by `mutex_`; there is no lock inversion (a worker never holds
`MotionController`'s lock while waiting on `EdgeDaemon::mutex_`); and
`estop`/`Stop()` abort + join the worker. The incremental-grid protocol falls
back to a full refresh correctly on generation change and keeps command
responses grid-free.

Minor / informational findings:

| # | Area | Finding | Severity |
|---|------|---------|----------|
| 1 | Performance | `GetGridUpdate` (and the sim's `buildGridUpdateLocked`) scan the **entire** grid O(W×H) every poll to find changed cells — there is no dirty-set. Serialization/transport/redraw are now O(changed), but the server-side find is O(N). Fine under the 300k-cell clamp (sub-millisecond); revisit if the clamp is ever raised much higher. | 🟢 Low |
| 2 | Consistency | A poll takes **two separate `mutex_` acquisitions** (`GetSnapshot` then `GetGridUpdate`), so position and grid can come from instants microseconds apart. Irrelevant for a UI; noted for completeness. | 🟢 Low |
| 3 | Lock hold | On a full refresh (generation change only — resolution change / scan start) the daemon builds two up-to-300k-element vectors while holding `mutex_`, briefly blocking the scan worker and allocating ~3.6 MB. Rare and acceptable. | 🟢 Low |
| 4 | Maintainability | The `if (gu.full)` branch in the UI's `applyGridUpdate` is effectively unreachable: the daemon/sim only set `full` on a generation mismatch, which the *first* branch already handles. Harmless defensive code; could be dropped or kept as a guard. | 🟢 Low |
| 5 | Sim drift | `set_scan_mode` updates the sim's displayed mode but the sim's duration estimate stays step-based (`gridW*gridH*0.11`) regardless of mode — so the sim shows a step-time estimate even in sweep mode. Extends the known sim/daemon drift liability (§5); the real daemon re-derives the estimate correctly. | 🟢 Low |

**What looks good:** targeted, well-commented fixes; the incremental-grid design
(generation + monotonic version + parallel `idx`/`val`) is clean and the
command/poll split keeps command responses small; telemetry removal is complete
with no stray references; every tier builds and vets clean and the runtime mock
smokes pass.

**Verdict:** Approve for merge **after** the real-Pi `HAS_PIGPIO=1` build and a
browser smoke of the new rendering path.
