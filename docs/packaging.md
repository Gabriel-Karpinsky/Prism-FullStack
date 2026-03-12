# Packaging Notes

## Prototype today

- run the PowerShell server on a machine near the scanner
- let multiple users connect over LAN or VPN
- keep the Arduino attached locally over USB

## Production package

Use four deployable units:

- `edge-daemon` in C++ on the hardware-adjacent host
- `control-api` in Go for auth, sessions, and operator locking
- `web-ui` as a built static or SSR app
- `caddy` as the HTTPS reverse proxy

## Suggested production compose layout

```yaml
services:
  control-api:
    build: ./apps/control-api
    restart: unless-stopped
  web-ui:
    build: ./apps/web-ui
    restart: unless-stopped
  caddy:
    image: caddy:2
    restart: unless-stopped
    ports:
      - "80:80"
      - "443:443"
```

For the MVP, keep the web and control services on the same machine as the scanner. Once the hardware loop is stable, split the cloud-facing control plane away from the edge host.
