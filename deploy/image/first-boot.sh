#!/usr/bin/env bash
# Runs once on first boot, then disables itself via the systemd
# ConditionPathExists guard. Handles the bits that need the real device:
#   - per-Pi unique hostname (derived from MAC) so multiple scanners on one
#     tailnet don't collide
#   - Tailscale auth, if the operator dropped a key file in /boot/firmware/
#   - shred the auth key after consuming it so it doesn't sit on the SD card

set -eu

LOG=/var/log/cliffscanner-firstboot.log
exec >>"$LOG" 2>&1
echo "==== cliffscanner-firstboot $(date -Is) ===="

# --- Hostname --------------------------------------------------------------
# Use the lowercased last 6 hex digits of eth0's MAC. Falls back to wlan0,
# then to a date-stamped value if neither interface has a MAC yet.
mac_suffix=""
for iface in eth0 wlan0; do
    if mac=$(cat "/sys/class/net/${iface}/address" 2>/dev/null); then
        mac_suffix=$(echo "$mac" | tr -d ':' | tr 'A-Z' 'a-z' | cut -c7-12)
        [ -n "$mac_suffix" ] && break
    fi
done
[ -z "$mac_suffix" ] && mac_suffix="$(date +%s | tail -c 7)"

hostname_new="cliffscanner-${mac_suffix}"
echo "Setting hostname to ${hostname_new}"
hostnamectl set-hostname "$hostname_new"
sed -i "s/^127\.0\.1\.1.*/127.0.1.1\t${hostname_new}/" /etc/hosts || \
    echo "127.0.1.1\t${hostname_new}" >> /etc/hosts

# --- Tailscale -------------------------------------------------------------
KEY_FILE=/boot/firmware/tailscale-authkey
[ -f "$KEY_FILE" ] || KEY_FILE=/boot/tailscale-authkey   # pre-Bookworm fallback

if [ -f "$KEY_FILE" ]; then
    AUTHKEY=$(tr -d '[:space:]' < "$KEY_FILE")
    if [ -n "$AUTHKEY" ]; then
        echo "Tailscale auth key found; joining tailnet."
        systemctl enable --now tailscaled
        # Retry up to 3× — the first-boot network may not be settled yet.
        for attempt in 1 2 3; do
            if tailscale up \
                --authkey="$AUTHKEY" \
                --ssh \
                --hostname="$hostname_new"; then
                echo "tailscale up succeeded on attempt ${attempt}"
                break
            fi
            echo "tailscale up failed on attempt ${attempt}; retrying in 10s"
            sleep 10
        done
        # Shred regardless — leaking the key is worse than re-authing manually.
        shred -u "$KEY_FILE" || rm -f "$KEY_FILE"
    fi
else
    echo "No Tailscale auth key at ${KEY_FILE}; skipping tailnet join."
fi

# --- One-shot guard --------------------------------------------------------
# The systemd unit also writes a sentinel via ExecStartPost, but we belt+brace
# it here so a manual re-run still no-ops.
install -d /var/lib/cliffscanner
touch /var/lib/cliffscanner/firstboot-done

echo "==== cliffscanner-firstboot done ===="
