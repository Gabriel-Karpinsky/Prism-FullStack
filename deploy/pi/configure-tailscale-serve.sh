#!/usr/bin/env bash
set -euo pipefail

if ! command -v tailscale >/dev/null 2>&1; then
  echo "Install Tailscale on the Raspberry Pi first."
  exit 1
fi

echo "This command will expose the locally bound Go backend through your tailnet."
sudo tailscale serve --https=443 http://127.0.0.1:8080
sudo tailscale serve status
