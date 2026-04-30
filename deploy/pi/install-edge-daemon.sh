#!/usr/bin/env bash
# Installer for the Pi-native edge-daemon.
#
# Preconditions (apt): build-essential pigpio libpigpio-dev nlohmann-json3-dev
#   sudo apt install build-essential pigpio libpigpio-dev nlohmann-json3-dev
#
# The build links libpigpio and must run on the Pi (or be cross-compiled with
# the same headers). HAS_PIGPIO=1 enables the real backend; omit it for a
# simulate-only build on a dev host.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

cd "$REPO_ROOT/apps/edge-daemon"
make clean
make HAS_PIGPIO=1

sudo install -d /opt/cliffscanner/bin
sudo install -m 0755 cliffscanner-edge /opt/cliffscanner/bin/cliffscanner-edge

sudo install -d /etc/cliffscanner
sudo install -d /etc/prism-scanner
sudo install -d /var/lib/cliffscanner

if [[ ! -f /etc/cliffscanner/edge-daemon.env ]]; then
  sudo install -m 0644 "$REPO_ROOT/deploy/pi/edge-daemon.env.example" /etc/cliffscanner/edge-daemon.env
fi

# Seed /etc/prism-scanner/hardware.json with the shipped defaults so the first
# boot comes up healthy; subsequent PUT /api/config/motion calls persist here.
if [[ ! -f /etc/prism-scanner/hardware.json ]]; then
  sudo install -m 0644 "$REPO_ROOT/deploy/pi/hardware.json.example" /etc/prism-scanner/hardware.json
fi

# pigpiod is optional — the daemon uses the in-process pigpio library directly.
# If the user has pigpiod enabled system-wide, disable it to avoid fighting
# over /dev/gpiomem.
if systemctl is-enabled --quiet pigpiod 2>/dev/null; then
  sudo systemctl disable --now pigpiod || true
fi

sudo install -m 0644 "$REPO_ROOT/deploy/pi/cliffscanner-edge.service" \
                      /etc/systemd/system/cliffscanner-edge.service
sudo systemctl daemon-reload
sudo systemctl enable --now cliffscanner-edge.service
