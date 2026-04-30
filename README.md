# Cliff Face Scanner MVP

A 2-axis cliff-face scanner controlled from a browser. Pi-native: a single
Raspberry Pi 4B drives two TB6600 stepper drivers via pigpio DMA waveforms,
reads a Garmin LIDAR-Lite v3HP over I²C, and serves the operator UI.

## Quickstart (flash + power on)

1. Download the latest pre-baked image:
   <https://github.com/Gabriel-Karpinsky/Prism-FullStack/releases/latest>
   (file: `cliffscanner-pi-aarch64-vX.Y.Z.img.xz`).
2. Flash with **Raspberry Pi Imager** → "Use custom image". Set SSH/WiFi via
   Imager's customisation pane (Ctrl-Shift-X).
3. (Optional) Drop a Tailscale auth key at `/boot/firmware/tailscale-authkey`
   on the SD card before ejecting; first boot joins the tailnet and shreds
   the file.
4. Power on. After ~90 seconds: SSH in, hit `http://<pi>:8080`, control the
   scanner.

Full flash-path docs in [docs/pi-deployment.md](docs/pi-deployment.md).
Wiring in [docs/hardware-pins.md](docs/hardware-pins.md).

## Repository layout

- `apps/edge-daemon` — native C++ service owning all GPIO + I²C. pigpio DMA
  waveforms for stepper timing, Garmin v3HP reads, scan state machine,
  systemd watchdog integration.
- `apps/control-api` — Go backend serving the operator UI and brokering the
  single-operator control lease. Proxies to the edge daemon over localhost.
- `apps/web-ui` — dependency-free browser UI.
- `deploy/pi` — systemd units, env examples, install scripts for manual
  bring-up.
- `deploy/image` — provisioning script + first-boot service consumed by the
  GitHub Actions image-build workflow.
- `.github/workflows/build-image.yml` — turns a tag push into a flashable
  `.img.xz` via CustoPiZer + qemu-arm64.
- `docs` — architecture, hardware pinout, deployment notes.

## Local development (no Pi required)

The edge-daemon's GPIO backend has a mock implementation for host builds:

```bash
cd apps/edge-daemon && make    # HAS_PIGPIO unset → mock backend
./cliffscanner-edge            # listens on 127.0.0.1:9090
```

Or run the Go backend against its built-in sim service:

```powershell
powershell -ExecutionPolicy Bypass -File .\deploy\run-go-backend.ps1 -Port 8080
```

Browse to `http://localhost:8080`.

## Cutting a release

```bash
git tag v0.1.0
git push --tags
```

This triggers `.github/workflows/build-image.yml`, which:

1. Cross-compiles the control-api container and pushes it to GHCR
   (`ghcr.io/gabriel-karpinsky/cliffscanner-control-api:v0.1.0`).
2. Bakes that container into a Raspberry Pi OS Lite image via CustoPiZer.
3. Attaches the resulting `.img.xz` to a GitHub Release.

Expect 18–25 min end-to-end. The first run also needs the GHCR package set
to public visibility — see the comment block at the top of the workflow file.
