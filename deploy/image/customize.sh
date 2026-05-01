#!/usr/bin/env bash
# Provisioning script that runs INSIDE the qemu-arm64 chroot during the
# CustoPiZer image bake. Mutates a Raspberry Pi OS Lite root filesystem
# into a turn-key Cliff Scanner appliance.
#
# Inputs (env vars set by the GitHub Actions workflow):
#   CONTROL_API_IMAGE        Full GHCR ref to docker-pull and tag locally
#                            (e.g. ghcr.io/gabriel-karpinsky/cliffscanner-control-api:v0.1.0)
#   CLIFFSCANNER_VERSION     Version string written to /etc/cliffscanner/version
#
# Inputs (filesystem):
#   /files/repo
#                            Staged project files copied by the workflow. This
#                            path is visible inside the Pi rootfs chroot.
#
# Anything that needs the *real* hardware (MAC-derived hostname, Tailscale
# auth) deferred to /usr/local/sbin/cliffscanner-firstboot. This script must
# stay machine-agnostic so the same image flashes onto any Pi.

set -euxo pipefail
export DEBIAN_FRONTEND=noninteractive
SOURCE_ROOT="${CLIFFSCANNER_SOURCE_ROOT:-/files/repo}"

# ---------------------------------------------------------------------------
# 1. Packages
# ---------------------------------------------------------------------------
# Phase 1a — prerequisites needed to add Docker's apt repo. Debian Bookworm
# does NOT ship docker-compose-plugin (and the docker.io package only gives
# us the daemon, not the v2 compose plugin). The cliffscanner-control-api
# systemd unit calls `docker compose ... up`, which is v2 subcommand syntax,
# so we install the official Docker CE stack from Docker's own repo.
apt-get update
apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg

install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/debian/gpg \
    | gpg --dearmor -o /etc/apt/keyrings/docker.gpg
chmod a+r /etc/apt/keyrings/docker.gpg

# Bookworm-arm64. EDITBASE_ARCH is forced to aarch64 by the workflow so we
# can hardcode arm64 here without sniffing dpkg.
echo "deb [arch=arm64 signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/debian bookworm stable" \
    > /etc/apt/sources.list.d/docker.list

apt-get update

# Phase 1b — everything we actually need on the device.
apt-get install -y --no-install-recommends \
    build-essential \
    pigpio libpigpio-dev nlohmann-json3-dev \
    i2c-tools \
    docker-ce docker-ce-cli containerd.io \
    docker-buildx-plugin docker-compose-plugin

# ---------------------------------------------------------------------------
# 2. Boot config: enable I²C bus 1 unconditionally.
# ---------------------------------------------------------------------------
# Bookworm moved /boot to /boot/firmware. Handle both paths so the script
# survives a pre-Bookworm base image too.
CONFIG_TXT=/boot/firmware/config.txt
[ -f "$CONFIG_TXT" ] || CONFIG_TXT=/boot/config.txt

sed -i 's/^#dtparam=i2c_arm=on/dtparam=i2c_arm=on/' "$CONFIG_TXT" || true
grep -q '^dtparam=i2c_arm=on' "$CONFIG_TXT" || \
    echo 'dtparam=i2c_arm=on' >> "$CONFIG_TXT"

# pigpio uses /dev/mem + DMA; nothing extra needed in config.txt for it.
# pigpiod (the standalone daemon) fights pigpio-the-library, so disable it.
systemctl disable pigpiod || true

# ---------------------------------------------------------------------------
# 3. Source tree -> /opt/cliffscanner
# ---------------------------------------------------------------------------
install -d /opt/cliffscanner
cp -r "${SOURCE_ROOT}/apps"   /opt/cliffscanner/
cp -r "${SOURCE_ROOT}/deploy" /opt/cliffscanner/
cp -r "${SOURCE_ROOT}/docs"   /opt/cliffscanner/

# ---------------------------------------------------------------------------
# 4. Build the C++ edge daemon under qemu (slow but correct).
# ---------------------------------------------------------------------------
make -C /opt/cliffscanner/apps/edge-daemon HAS_PIGPIO=1
install -d /opt/cliffscanner/bin
install -m 0755 /opt/cliffscanner/apps/edge-daemon/cliffscanner-edge \
                /opt/cliffscanner/bin/cliffscanner-edge

# Remove intermediates now that the binary is installed; keeps the image lean.
make -C /opt/cliffscanner/apps/edge-daemon clean
rm -rf /opt/cliffscanner/apps/edge-daemon/src/*.o

# ---------------------------------------------------------------------------
# 5. Bake the control-api container into the rootfs as a docker-save tarball.
#
# The workflow already did `docker pull --platform linux/arm64` on the GitHub
# runner (where Docker actually works) and ran `docker save` to produce
# /files/control-api.tar. We just copy it into the image; a oneshot systemd
# unit (cliffscanner-load-image.service, installed below) runs `docker load`
# against this file once on first start, before the compose service comes up.
# This means the device needs zero network for the control-api container path
# on first boot — which matches the Tailscale-on-first-boot guarantee for the
# rest of the appliance.
# ---------------------------------------------------------------------------
install -d /var/lib/cliffscanner /etc/cliffscanner
if [ -f /files/control-api.tar ]; then
    install -m 0644 /files/control-api.tar /var/lib/cliffscanner/control-api.tar
    echo "baked-tarball" > /etc/cliffscanner/control-api-source
else
    # Workflow forgot to stage the tarball — refuse to ship a half-broken image.
    echo "FATAL: /files/control-api.tar missing; CI workflow is misconfigured" >&2
    exit 1
fi

# Stage docker-compose.yml and rewrite the build: stanza to reference the
# loaded image by its full GHCR tag. After `docker load` the image is in the
# local store under exactly that name, so compose finds it without a network
# round-trip.
install -d /opt/cliffscanner/deploy/pi
cp "${SOURCE_ROOT}/deploy/pi/docker-compose.yml" \
   /opt/cliffscanner/deploy/pi/docker-compose.yml
sed -i \
    -e "s|^\s*build:.*|    image: ${CONTROL_API_IMAGE}|" \
    -e '/^\s*context:/d' \
    -e '/^\s*dockerfile:/d' \
    /opt/cliffscanner/deploy/pi/docker-compose.yml

# ---------------------------------------------------------------------------
# 6. Config seeds.
# ---------------------------------------------------------------------------
install -d /etc/cliffscanner /etc/prism-scanner /var/lib/cliffscanner
install -m 0644 "${SOURCE_ROOT}/deploy/pi/edge-daemon.env.example" \
                /etc/cliffscanner/edge-daemon.env
install -m 0644 "${SOURCE_ROOT}/deploy/pi/hardware.json.example" \
                /etc/prism-scanner/hardware.json

# ---------------------------------------------------------------------------
# 7. Systemd units.
# ---------------------------------------------------------------------------
install -m 0644 "${SOURCE_ROOT}/deploy/pi/cliffscanner-edge.service" \
                /etc/systemd/system/cliffscanner-edge.service
install -m 0644 "${SOURCE_ROOT}/deploy/pi/cliffscanner-control-api.service" \
                /etc/systemd/system/cliffscanner-control-api.service
install -m 0644 "${SOURCE_ROOT}/deploy/image/cliffscanner-firstboot.service" \
                /etc/systemd/system/cliffscanner-firstboot.service
install -m 0644 "${SOURCE_ROOT}/deploy/image/cliffscanner-load-image.service" \
                /etc/systemd/system/cliffscanner-load-image.service

systemctl enable cliffscanner-edge.service
systemctl enable cliffscanner-control-api.service
systemctl enable cliffscanner-firstboot.service
systemctl enable cliffscanner-load-image.service

# ---------------------------------------------------------------------------
# 8. Tailscale (binary only — auth happens at first boot if a key is dropped).
# ---------------------------------------------------------------------------
install -d /usr/share/keyrings
curl -fsSL https://pkgs.tailscale.com/stable/raspbian/bookworm.noarmor.gpg \
    -o /usr/share/keyrings/tailscale-archive-keyring.gpg
curl -fsSL https://pkgs.tailscale.com/stable/raspbian/bookworm.tailscale-keyring.list \
    -o /etc/apt/sources.list.d/tailscale.list
apt-get update
apt-get install -y tailscale
# Don't enable tailscaled here: first-boot decides whether to start it.
systemctl disable tailscaled || true

# ---------------------------------------------------------------------------
# 9. First-boot script + version stamp.
# ---------------------------------------------------------------------------
install -m 0755 "${SOURCE_ROOT}/deploy/image/first-boot.sh" \
                /usr/local/sbin/cliffscanner-firstboot
echo "${CLIFFSCANNER_VERSION:-unknown}" > /etc/cliffscanner/version

# ---------------------------------------------------------------------------
# 10. Cleanup — strip apt caches and any stray build artefacts.
# ---------------------------------------------------------------------------
apt-get clean
rm -rf /var/lib/apt/lists/*
rm -rf /var/cache/apt/archives/*
rm -rf /tmp/*

echo "==== customize.sh complete (version=${CLIFFSCANNER_VERSION:-unknown}) ===="
