#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT/apps/edge-daemon"
make clean
make
sudo install -d /opt/cliffscanner/bin
sudo install -m 0755 cliffscanner-edge /opt/cliffscanner/bin/cliffscanner-edge
sudo install -d /etc/cliffscanner
if [[ ! -f /etc/cliffscanner/edge-daemon.env ]]; then
  sudo install -m 0644 "$REPO_ROOT/deploy/pi/edge-daemon.env.example" /etc/cliffscanner/edge-daemon.env
fi
sudo install -m 0644 "$REPO_ROOT/deploy/pi/cliffscanner-edge.service" /etc/systemd/system/cliffscanner-edge.service
sudo systemctl daemon-reload
sudo systemctl enable --now cliffscanner-edge.service
