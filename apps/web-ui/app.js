const state = {
  snapshot: null,
  lastMessage: "Loading scanner state...",
  pollHandle: null,
};

const el = {
  userName: document.getElementById("user-name"),
  acquireBtn: document.getElementById("acquire-btn"),
  releaseBtn: document.getElementById("release-btn"),
  resolutionSelect: document.getElementById("resolution-select"),
  commandStatus: document.getElementById("command-status"),
  controlOwner: document.getElementById("control-owner"),
  modeBadge: document.getElementById("mode-badge"),
  yawValue: document.getElementById("yaw-value"),
  pitchValue: document.getElementById("pitch-value"),
  coverageValue: document.getElementById("coverage-value"),
  progressValue: document.getElementById("progress-value"),
  motorTempValue: document.getElementById("motor-temp-value"),
  latencyValue: document.getElementById("latency-value"),
  progressFill: document.getElementById("progress-fill"),
  scanDurationValue: document.getElementById("scan-duration-value"),
  faultBanner: document.getElementById("fault-banner"),
  activityLog: document.getElementById("activity-log"),
  mapCanvas: document.getElementById("surface-map"),
};

const ctx = el.mapCanvas.getContext("2d");

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

function drawSurfaceMap(snapshot) {
  const grid = snapshot.grid || [];
  const rows = grid.length;
  const cols = rows > 0 ? grid[0].length : 0;
  const width = el.mapCanvas.width;
  const height = el.mapCanvas.height;
  const cellWidth = cols > 0 ? width / cols : width;
  const cellHeight = rows > 0 ? height / rows : height;

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#0b252b";
  ctx.fillRect(0, 0, width, height);

  for (let y = 0; y < rows; y += 1) {
    for (let x = 0; x < cols; x += 1) {
      const value = grid[y][x];
      ctx.fillStyle = colorForHeight(value);
      ctx.fillRect(x * cellWidth, y * cellHeight, cellWidth + 1, cellHeight + 1);
    }
  }

  const yawMin = snapshot.scanSettings.yawMin;
  const yawMax = snapshot.scanSettings.yawMax;
  const pitchMin = snapshot.scanSettings.pitchMin;
  const pitchMax = snapshot.scanSettings.pitchMax;

  const markerX = ((snapshot.yaw - yawMin) / Math.max(1, yawMax - yawMin)) * width;
  const markerY = ((snapshot.pitch - pitchMin) / Math.max(1, pitchMax - pitchMin)) * height;

  ctx.strokeStyle = "rgba(255,255,255,0.9)";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(markerX, markerY, 7, 0, Math.PI * 2);
  ctx.stroke();

  ctx.fillStyle = "rgba(255,255,255,0.24)";
  ctx.beginPath();
  ctx.arc(markerX, markerY, 13, 0, Math.PI * 2);
  ctx.fill();
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

function render(snapshot) {
  state.snapshot = snapshot;
  el.controlOwner.textContent = snapshot.controlOwner || "Unclaimed";
  el.modeBadge.textContent = snapshot.mode;
  el.yawValue.textContent = `${snapshot.yaw.toFixed(1)}°`;
  el.pitchValue.textContent = `${snapshot.pitch.toFixed(1)}°`;
  el.coverageValue.textContent = `${Math.round(snapshot.coverage * 100)}%`;
  el.progressValue.textContent = `${Math.round(snapshot.scanProgress * 100)}%`;
  el.motorTempValue.textContent = `${snapshot.metrics.motorTempC.toFixed(1)} C`;
  el.latencyValue.textContent = `${snapshot.metrics.latencyMs} ms`;
  el.progressFill.style.width = `${Math.round(snapshot.scanProgress * 100)}%`;
  el.scanDurationValue.textContent = `Estimated duration: ${snapshot.scanDurationSeconds.toFixed(0)}s`;
  el.resolutionSelect.value = snapshot.scanSettings.resolution;

  if (snapshot.faults && snapshot.faults.length > 0) {
    el.faultBanner.classList.remove("hidden");
    el.faultBanner.textContent = snapshot.faults.join(" · ");
  } else {
    el.faultBanner.classList.add("hidden");
    el.faultBanner.textContent = "";
  }

  renderLog(snapshot.activity);
  drawSurfaceMap(snapshot);
}

async function pollState() {
  try {
    const snapshot = await request("/api/state", { method: "GET" });
    render(snapshot);
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
    render(response.state);
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
    render(response.state);
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
    render(response.state);
    setStatus(`Command '${command}' sent.`);
  } catch (error) {
    setStatus(error.message, true);
  }
}

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

pollState();
state.pollHandle = window.setInterval(pollState, 700);
