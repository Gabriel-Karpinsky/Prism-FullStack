#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
sudo install -d /opt/cliffscanner/deploy/pi
sudo install -m 0644 "$REPO_ROOT/deploy/pi/docker-compose.yml" /opt/cliffscanner/deploy/pi/docker-compose.yml
sudo install -m 0644 "$REPO_ROOT/deploy/pi/cliffscanner-control-api.service" /etc/systemd/system/cliffscanner-control-api.service
sudo systemctl daemon-reload
sudo systemctl enable --now cliffscanner-control-api.service
