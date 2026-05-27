const state = {
  snapshot: null,
  lastMessage: "Loading scanner state...",
  pollHandle: null,
  // Last motion config loaded from the server. Used by "Reset to loaded".
  loadedMotionConfig: null,
  // Incremental scan grid. The grid is held client-side as a flat Float32Array
  // (row-major); each poll sends ?since/&gen and the server returns only the
  // cells that changed, which we paint individually instead of redrawing the
  // whole canvas every 700 ms. gridGeneration tracks the server's grid identity
  // (bumps on resolution change / scan reset → triggers a full rebuild).
  grid: null,
  gridW: 0,
  gridH: 0,
  gridGeneration: 0,
  gridSince: 0,
};

const el = {
  userName: document.getElementById("user-name"),
  acquireBtn: document.getElementById("acquire-btn"),
  releaseBtn: document.getElementById("release-btn"),
  resolutionSelect: document.getElementById("resolution-select"),
  scanModeSelect: document.getElementById("scan-mode-select"),
  commandStatus: document.getElementById("command-status"),
  controlOwner: document.getElementById("control-owner"),
  modeBadge: document.getElementById("mode-badge"),
  yawValue: document.getElementById("yaw-value"),
  pitchValue: document.getElementById("pitch-value"),
  coverageValue: document.getElementById("coverage-value"),
  progressValue: document.getElementById("progress-value"),
  progressFill: document.getElementById("progress-fill"),
  scanDurationValue: document.getElementById("scan-duration-value"),
  resolutionDetail: document.getElementById("resolution-detail"),
  faultBanner: document.getElementById("fault-banner"),
  activityLog: document.getElementById("activity-log"),
  mapCanvas: document.getElementById("surface-map"),
  overlayCanvas: document.getElementById("surface-overlay"),
  // Motion config panel
  mcYawMin:    document.getElementById("mc-yaw-min"),
  mcYawMax:    document.getElementById("mc-yaw-max"),
  mcYawSpeed:  document.getElementById("mc-yaw-speed"),
  mcYawAccel:  document.getElementById("mc-yaw-accel"),
  mcPitchMin:  document.getElementById("mc-pitch-min"),
  mcPitchMax:  document.getElementById("mc-pitch-max"),
  mcPitchSpeed: document.getElementById("mc-pitch-speed"),
  mcPitchAccel: document.getElementById("mc-pitch-accel"),
  motionApplyBtn: document.getElementById("motion-apply-btn"),
  motionResetBtn: document.getElementById("motion-reset-btn"),
  motionConfigStatus: document.getElementById("motion-config-status"),
};

const ctx = el.mapCanvas.getContext("2d");
const octx = el.overlayCanvas.getContext("2d");

function getUser() {
  return el.userName.value.trim();
}

function setStatus(message, isError = false) {
  state.lastMessage = message;
  el.commandStatus.textContent = message;
  el.commandStatus.style.color = isError ? "#9e3328" : "";
}

async function request(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  const body = await response.json();
  if (!response.ok || body.error) {
    throw new Error(body.error || `Request failed with ${response.status}`);
  }
  return body;
}

function colorForHeight(value) {
  if (value < 0) return "#12343b";
  const stops = [
    { p: 0.0, c: [17, 60, 54] },
    { p: 0.25, c: [18, 109, 96] },
    { p: 0.55, c: [64, 156, 129] },
    { p: 0.8, c: [231, 169, 61] },
    { p: 1.0, c: [240, 244, 245] },
  ];

  for (let i = 0; i < stops.length - 1; i += 1) {
    const left = stops[i];
    const right = stops[i + 1];
    if (value >= left.p && value <= right.p) {
      const t = (value - left.p) / (right.p - left.p);
      const rgb = left.c.map((channel, index) =>
        Math.round(channel + (right.c[index] - channel) * t)
      );
      return `rgb(${rgb[0]}, ${rgb[1]}, ${rgb[2]})`;
    }
  }

  return "rgb(240, 244, 245)";
}

// The heatmap lives on #surface-map; the head marker lives on the #surface-overlay
// canvas stacked on top. Keeping them separate means a poll repaints only the
// handful of cells that changed (or just the marker), never the whole grid.

function paintCellByIndex(i) {
  if (!state.grid || state.gridW === 0 || state.gridH === 0) return;
  const x = i % state.gridW;
  const y = Math.floor(i / state.gridW);
  const cw = el.mapCanvas.width / state.gridW;
  const ch = el.mapCanvas.height / state.gridH;
  ctx.fillStyle = colorForHeight(state.grid[i]);
  // +1 avoids hairline seams between cells at fractional sizes.
  ctx.fillRect(x * cw, y * ch, cw + 1, ch + 1);
}

function repaintAllCells() {
  const w = el.mapCanvas.width;
  const h = el.mapCanvas.height;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#0b252b";
  ctx.fillRect(0, 0, w, h);
  if (!state.grid) return;
  for (let i = 0; i < state.grid.length; i += 1) {
    if (state.grid[i] < 0) continue; // unfilled cells show through to the background
    paintCellByIndex(i);
  }
}

function drawMarker(snapshot) {
  const w = el.overlayCanvas.width;
  const h = el.overlayCanvas.height;
  octx.clearRect(0, 0, w, h);
  if (!snapshot || !snapshot.scanSettings) return;

  const { yawMin, yawMax, pitchMin, pitchMax } = snapshot.scanSettings;
  const markerX = ((snapshot.yaw - yawMin) / Math.max(1, yawMax - yawMin)) * w;
  const markerY = ((snapshot.pitch - pitchMin) / Math.max(1, pitchMax - pitchMin)) * h;

  octx.strokeStyle = "rgba(255,255,255,0.9)";
  octx.lineWidth = 2;
  octx.beginPath();
  octx.arc(markerX, markerY, 7, 0, Math.PI * 2);
  octx.stroke();

  octx.fillStyle = "rgba(255,255,255,0.24)";
  octx.beginPath();
  octx.arc(markerX, markerY, 13, 0, Math.PI * 2);
  octx.fill();
}

// Apply a GridUpdate from the server. Three cases:
//   • new generation or size change → rebuild the local array, full repaint
//   • full=true                     → clear + apply the provided cells, full repaint
//   • delta                         → apply changed cells, repaint just those
function applyGridUpdate(gu) {
  if (!gu) return; // command/acquire/release responses carry no grid

  const sizeChanged = gu.width !== state.gridW || gu.height !== state.gridH;
  if (gu.generation !== state.gridGeneration || sizeChanged || !state.grid) {
    state.gridGeneration = gu.generation;
    state.gridW = gu.width;
    state.gridH = gu.height;
    state.grid = new Float32Array(gu.width * gu.height).fill(-1);
    for (let k = 0; k < gu.idx.length; k += 1) state.grid[gu.idx[k]] = gu.val[k];
    state.gridSince = gu.version;
    repaintAllCells();
    return;
  }

  if (gu.full) {
    state.grid.fill(-1);
    for (let k = 0; k < gu.idx.length; k += 1) state.grid[gu.idx[k]] = gu.val[k];
    state.gridSince = gu.version;
    repaintAllCells();
    return;
  }

  for (let k = 0; k < gu.idx.length; k += 1) {
    const i = gu.idx[k];
    state.grid[i] = gu.val[k];
    paintCellByIndex(i);
  }
  state.gridSince = gu.version;
}

function renderLog(activity = []) {
  el.activityLog.innerHTML = "";
  const entries = activity.slice(0, 8);

  entries.forEach((entry) => {
    const li = document.createElement("li");
    const meta = document.createElement("div");
    meta.className = "log-meta";

    const source = document.createElement("span");
    source.textContent = `${entry.source} · ${entry.level}`;
    meta.appendChild(source);

    const ts = document.createElement("span");
    ts.textContent = new Date(entry.ts).toLocaleTimeString();
    meta.appendChild(ts);

    const message = document.createElement("div");
    message.textContent = entry.message;

    li.appendChild(meta);
    li.appendChild(message);
    el.activityLog.appendChild(li);
  });
}

function formatDuration(seconds) {
  const s = Math.max(0, Math.round(seconds || 0));
  if (s < 60) return `${s}s`;
  if (s < 3600) return `${Math.floor(s / 60)}m ${s % 60}s`;
  return `${Math.floor(s / 3600)}h ${Math.floor((s % 3600) / 60)}m`;
}

// Scan density: resolution presets map to a microstep stride; the grid size
// and scan-time estimate fall out of that and the motion range. This caption
// makes the real numbers visible so the preset choice isn't a black box.
function renderScanDensity(snapshot) {
  const cols = state.gridW;
  const rows = state.gridH;
  const ss = snapshot.scanSettings;
  const stride = ss.sampleStrideMicrosteps || 0;
  let detail = `Scan grid: ${cols} × ${rows} samples`;
  if (stride > 0) {
    detail += ` · stride ${stride} µstep${stride === 1 ? "" : "s"}`;
  }
  if (ss.scanMode === "sweep") {
    detail += ` · sweep ≤ ${Math.round(ss.sweepMaxSpeedDegS || 0)}°/s`;
  } else if (ss.scanMode === "step") {
    detail += " · step (stop & shoot)";
  }
  detail += ` · est. ${formatDuration(snapshot.scanDurationSeconds)}`;
  if (cols * rows > 250000) {
    detail += " · grid clamped — narrow the scan range for finer detail";
  }
  el.resolutionDetail.textContent = detail;
}

// renderState updates everything except the heatmap (text fields, faults, log,
// scan-density caption, and the head marker). The heatmap is driven separately
// by applyGridUpdate, so command/lease responses — which carry no grid — call
// renderState only and leave the grid untouched.
function renderState(snapshot) {
  state.snapshot = snapshot;
  el.controlOwner.textContent = snapshot.controlOwner || "Unclaimed";
  el.modeBadge.textContent = snapshot.mode;
  el.yawValue.textContent = `${snapshot.yaw.toFixed(1)}°`;
  el.pitchValue.textContent = `${snapshot.pitch.toFixed(1)}°`;
  el.coverageValue.textContent = `${Math.round(snapshot.coverage * 100)}%`;
  el.progressValue.textContent = `${Math.round(snapshot.scanProgress * 100)}%`;
  el.progressFill.style.width = `${Math.round(snapshot.scanProgress * 100)}%`;
  el.scanDurationValue.textContent = `Estimated duration: ${formatDuration(snapshot.scanDurationSeconds)}`;
  el.resolutionSelect.value = snapshot.scanSettings.resolution;
  if (snapshot.scanSettings.scanMode) {
    el.scanModeSelect.value = snapshot.scanSettings.scanMode;
  }
  renderScanDensity(snapshot);

  if (snapshot.faults && snapshot.faults.length > 0) {
    el.faultBanner.classList.remove("hidden");
    el.faultBanner.textContent = snapshot.faults.join(" · ");
  } else {
    el.faultBanner.classList.add("hidden");
    el.faultBanner.textContent = "";
  }

  renderLog(snapshot.activity);
  drawMarker(snapshot);
}

async function pollState() {
  try {
    // Send our grid cursor so the server can reply with only changed cells.
    const url = `/api/state?since=${state.gridSince}&gen=${state.gridGeneration}`;
    const snapshot = await request(url, { method: "GET" });
    applyGridUpdate(snapshot.gridUpdate); // sets gridW/gridH before renderState reads them
    renderState(snapshot);
    if (!state.lastMessage || state.lastMessage === "Loading scanner state...") {
      setStatus("Scanner state synchronized.");
    }
  } catch (error) {
    setStatus(error.message, true);
  }
}

async function acquireControl() {
  const user = getUser();
  if (!user) {
    setStatus("Enter a user name before acquiring control.", true);
    return;
  }

  try {
    const response = await request("/api/control/acquire", {
      method: "POST",
      body: JSON.stringify({ user }),
    });
    renderState(response.state);
    setStatus(`Control acquired by ${user}.`);
  } catch (error) {
    setStatus(error.message, true);
  }
}

async function releaseControl() {
  const user = getUser();
  if (!user) {
    setStatus("Enter your user name before releasing control.", true);
    return;
  }

  try {
    const response = await request("/api/control/release", {
      method: "POST",
      body: JSON.stringify({ user }),
    });
    renderState(response.state);
    setStatus(`Control released by ${user}.`);
  } catch (error) {
    setStatus(error.message, true);
  }
}

async function sendCommand(command, payload = {}) {
  const user = getUser();
  if (command !== "connect" && !user) {
    setStatus("Enter a user name before sending control commands.", true);
    return;
  }

  try {
    const response = await request("/api/command", {
      method: "POST",
      body: JSON.stringify({ user, command, payload }),
    });
    renderState(response.state);
    setStatus(`Command '${command}' sent.`);
  } catch (error) {
    setStatus(error.message, true);
  }
}

// ---------------------------------------------------------------------------
// Motion config panel
// ---------------------------------------------------------------------------

function setMotionStatus(message, isError = false) {
  el.motionConfigStatus.textContent = message;
  el.motionConfigStatus.style.color = isError ? "#9e3328" : "";
}

function populateMotionFields(cfg) {
  el.mcYawMin.value    = cfg.yaw.min_deg;
  el.mcYawMax.value    = cfg.yaw.max_deg;
  el.mcYawSpeed.value  = cfg.yaw.max_speed_deg_s;
  el.mcYawAccel.value  = cfg.yaw.accel_deg_s2;
  el.mcPitchMin.value  = cfg.pitch.min_deg;
  el.mcPitchMax.value  = cfg.pitch.max_deg;
  el.mcPitchSpeed.value = cfg.pitch.max_speed_deg_s;
  el.mcPitchAccel.value = cfg.pitch.accel_deg_s2;
}

function readMotionFields() {
  return {
    yaw: {
      min_deg:          parseFloat(el.mcYawMin.value),
      max_deg:          parseFloat(el.mcYawMax.value),
      max_speed_deg_s:  parseFloat(el.mcYawSpeed.value),
      accel_deg_s2:     parseFloat(el.mcYawAccel.value),
    },
    pitch: {
      min_deg:          parseFloat(el.mcPitchMin.value),
      max_deg:          parseFloat(el.mcPitchMax.value),
      max_speed_deg_s:  parseFloat(el.mcPitchSpeed.value),
      accel_deg_s2:     parseFloat(el.mcPitchAccel.value),
    },
  };
}

async function loadMotionConfig() {
  try {
    const cfg = await request("/api/config/motion", { method: "GET" });
    state.loadedMotionConfig = cfg;
    populateMotionFields(cfg);
    setMotionStatus("Limits loaded from device.");
  } catch (error) {
    setMotionStatus(`Failed to load motion config: ${error.message}`, true);
  }
}

async function applyMotionConfig() {
  const user = getUser();
  if (!user) {
    setMotionStatus("Enter a user name and acquire control before changing limits.", true);
    return;
  }
  const motion = readMotionFields();
  if (motion.yaw.min_deg >= motion.yaw.max_deg) {
    setMotionStatus("Yaw: min must be less than max.", true); return;
  }
  if (motion.pitch.min_deg >= motion.pitch.max_deg) {
    setMotionStatus("Pitch: min must be less than max.", true); return;
  }
  try {
    const response = await request("/api/config/motion", {
      method: "PUT",
      body: JSON.stringify({ user, motion }),
    });
    // PUT replies with an envelope {ok, motion}; GET replies with the bare
    // {yaw, pitch} config. Accept either so populateMotionFields never sees
    // an undefined axis.
    const updated = response.motion || response;
    state.loadedMotionConfig = updated;
    populateMotionFields(updated);
    setMotionStatus("Motion limits applied and persisted to device.");
  } catch (error) {
    setMotionStatus(error.message, true);
  }
}

el.motionApplyBtn.addEventListener("click", applyMotionConfig);
el.motionResetBtn.addEventListener("click", () => {
  if (state.loadedMotionConfig) {
    populateMotionFields(state.loadedMotionConfig);
    setMotionStatus("Reset to last loaded values.");
  }
});

// ---------------------------------------------------------------------------

el.acquireBtn.addEventListener("click", acquireControl);
el.releaseBtn.addEventListener("click", releaseControl);

document.querySelectorAll("[data-command]").forEach((button) => {
  button.addEventListener("click", () => {
    sendCommand(button.dataset.command);
  });
});

document.querySelectorAll("[data-jog-axis]").forEach((button) => {
  button.addEventListener("click", () => {
    sendCommand("jog", {
      axis: button.dataset.jogAxis,
      delta: Number(button.dataset.jogDelta),
    });
  });
});

el.resolutionSelect.addEventListener("change", () => {
  sendCommand("set_resolution", { resolution: el.resolutionSelect.value });
});

el.scanModeSelect.addEventListener("change", () => {
  sendCommand("set_scan_mode", { mode: el.scanModeSelect.value });
});

pollState();
loadMotionConfig();
state.pollHandle = window.setInterval(pollState, 700);
