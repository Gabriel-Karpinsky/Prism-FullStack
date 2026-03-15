# Packaging Notes

## Raspberry Pi target layout

The Raspberry Pi now hosts three runtime layers:

- `cliffscanner-edge` as a native `systemd` service
- `control-api` as a Docker container started via Docker Compose
- `tailscaled` as the VPN transport used for remote access

## Why the Go backend is containerized but the edge daemon is not

The Go backend is a good Docker candidate because it is a self-contained web/API process.

The edge daemon stays native because it is directly attached to:

- `/dev/ttyACM*` for the Arduino
- local GPIO/I2C resources for future lidar work
- host-level service supervision through `systemd`

That split keeps hardware access simple while still giving the web layer repeatable deployment.

## Service boundaries

- edge daemon: `127.0.0.1:9090`
- Go backend: `127.0.0.1:8080`
- Tailscale Serve: `https://<device>.<tailnet>.ts.net -> http://127.0.0.1:8080`

## Deployment assets

- [docker-compose.yml](/E:/_Data/_TUE/Prism/Codex web/deploy/pi/docker-compose.yml)
- [cliffscanner-edge.service](/E:/_Data/_TUE/Prism/Codex web/deploy/pi/cliffscanner-edge.service)
- [cliffscanner-control-api.service](/E:/_Data/_TUE/Prism/Codex web/deploy/pi/cliffscanner-control-api.service)
- [install-edge-daemon.sh](/E:/_Data/_TUE/Prism/Codex web/deploy/pi/install-edge-daemon.sh)
- [install-control-api-service.sh](/E:/_Data/_TUE/Prism/Codex web/deploy/pi/install-control-api-service.sh)
- [configure-tailscale-serve.sh](/E:/_Data/_TUE/Prism/Codex web/deploy/pi/configure-tailscale-serve.sh)
