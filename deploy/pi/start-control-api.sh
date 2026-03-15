#!/usr/bin/env bash
set -euo pipefail

STACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$STACK_ROOT"
sudo docker compose -f docker-compose.yml up -d --build
