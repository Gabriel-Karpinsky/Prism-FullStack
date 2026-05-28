#!/usr/bin/env bash
# update-from-master.sh — Pull the latest source from GitHub and rebuild any
# requested component in place, so you can iterate on the Pi during testing
# without re-baking the SD image.
#
# Usage:
#   sudo /opt/cliffscanner/deploy/pi/update-from-master.sh [target ...]
#
# Targets (omit ⇒ "all"):
#   edge          rebuild + restart the C++ edge-daemon
#   control-api   rebuild + restart the dockerized Go control-api (+ UI)
#   ui            UI-only: sync web-ui files into the running container layer
#                 (fast iteration; skips Go rebuild — use `control-api`
#                  whenever app.js/index.html/styles.css need to ship through
#                  the container build)
#   self          update this script (and the rest of deploy/pi/) from master
#   all           edge + control-api
#
# Environment overrides:
#   PRISM_REPO_URL     default: https://github.com/Gabriel-Karpinsky/Prism-FullStack.git
#   PRISM_REPO_BRANCH  default: master
#   PRISM_SHADOW_DIR   default: /var/lib/cliffscanner-src
#   PRISM_INSTALL_ROOT default: /opt/cliffscanner
#
# Bootstrap (image doesn't ship this script yet):
#   sudo curl -fsSL https://raw.githubusercontent.com/Gabriel-Karpinsky/Prism-FullStack/master/deploy/pi/update-from-master.sh \
#     -o /usr/local/sbin/prism-update && sudo chmod +x /usr/local/sbin/prism-update
#   sudo prism-update self    # now you have it at the canonical location too

set -euo pipefail

REPO_URL="${PRISM_REPO_URL:-https://github.com/Gabriel-Karpinsky/Prism-FullStack.git}"
REPO_BRANCH="${PRISM_REPO_BRANCH:-master}"
SHADOW="${PRISM_SHADOW_DIR:-/var/lib/cliffscanner-src}"
INSTALL_ROOT="${PRISM_INSTALL_ROOT:-/opt/cliffscanner}"

if [[ $EUID -ne 0 ]]; then
  echo "this script needs root (rebuilds binaries and restarts services)"
  exec sudo -E "$0" "$@"
fi

if [[ $# -eq 0 ]]; then
  TARGETS=(all)
else
  TARGETS=("$@")
fi

need() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "missing dependency: $1"
    exit 1
  }
}
need git
need rsync
need make
need install
need systemctl

# ---------------------------------------------------------------------------
# Shadow checkout: clone-once, fetch-and-reset-hard thereafter. Keeps the live
# install at /opt/cliffscanner clean of .git so it stays a plain "files" tree.
# ---------------------------------------------------------------------------
sync_shadow() {
  if [[ ! -d "$SHADOW/.git" ]]; then
    echo ">> first run: cloning $REPO_URL ($REPO_BRANCH) into $SHADOW"
    rm -rf "$SHADOW"
    install -d "$SHADOW"
    git clone --depth 50 --branch "$REPO_BRANCH" "$REPO_URL" "$SHADOW"
  else
    echo ">> fetching latest $REPO_BRANCH into shadow at $SHADOW"
    git -C "$SHADOW" fetch --depth 50 origin "$REPO_BRANCH"
    git -C "$SHADOW" reset --hard "origin/$REPO_BRANCH"
  fi
  SHA=$(git -C "$SHADOW" rev-parse --short HEAD)
  SUBJ=$(git -C "$SHADOW" log -1 --format='%s')
  echo ">> at $SHA  $SUBJ"
}

# ---------------------------------------------------------------------------
# Targets
# ---------------------------------------------------------------------------

update_self() {
  echo ">> updating deploy/pi/ (incl. this script)"
  install -d "$INSTALL_ROOT/deploy/pi"
  rsync -a "$SHADOW/deploy/pi/" "$INSTALL_ROOT/deploy/pi/"
  echo ">> if you ran this from $INSTALL_ROOT/deploy/pi/, re-invoke to use the new copy"
}

update_edge() {
  echo ">> syncing edge-daemon source"
  install -d "$INSTALL_ROOT/apps/edge-daemon"
  # Keep intermediates out of the sync so a previous build doesn't trip rsync.
  rsync -a --delete \
    --exclude='*.o' --exclude='cliffscanner-edge' \
    "$SHADOW/apps/edge-daemon/" "$INSTALL_ROOT/apps/edge-daemon/"

  echo ">> rebuilding edge-daemon"
  systemctl stop cliffscanner-edge || true
  ( cd "$INSTALL_ROOT/apps/edge-daemon" && make clean && make HAS_PIGPIO=1 && make install HAS_PIGPIO=1 )

  systemctl start cliffscanner-edge
  echo ">> edge-daemon restarted; tail of journal:"
  journalctl -u cliffscanner-edge -n 8 --no-pager || true
}

update_ui_files_only() {
  echo ">> syncing UI source files (no container rebuild)"
  install -d "$INSTALL_ROOT/apps/web-ui"
  rsync -a --delete "$SHADOW/apps/web-ui/" "$INSTALL_ROOT/apps/web-ui/"
  # The control-api container serves the UI from the mounted/copied tree —
  # if it COPYs at build time, you need `control-api` target instead, not `ui`.
  echo ">> UI files updated at $INSTALL_ROOT/apps/web-ui/"
  echo "   if the container baked the UI in, run with target 'control-api' to rebuild."
}

update_control_api() {
  echo ">> syncing control-api + UI source"
  install -d "$INSTALL_ROOT/apps/control-api" "$INSTALL_ROOT/apps/web-ui" "$INSTALL_ROOT/deploy/pi"
  rsync -a --delete "$SHADOW/apps/control-api/" "$INSTALL_ROOT/apps/control-api/"
  rsync -a --delete "$SHADOW/apps/web-ui/"      "$INSTALL_ROOT/apps/web-ui/"
  rsync -a            "$SHADOW/deploy/pi/"      "$INSTALL_ROOT/deploy/pi/"

  echo ">> rebuilding control-api container and restarting the unit"
  ( cd "$INSTALL_ROOT/deploy/pi" && docker compose -f docker-compose.yml build )
  systemctl restart cliffscanner-control-api
  echo ">> control-api restarted; tail of journal:"
  journalctl -u cliffscanner-control-api -n 8 --no-pager || true
}

# ---------------------------------------------------------------------------
# Run requested targets (one shadow sync up front; targets reuse it).
# ---------------------------------------------------------------------------
sync_shadow

for t in "${TARGETS[@]}"; do
  case "$t" in
    self)         update_self ;;
    edge)         update_edge ;;
    ui)           update_ui_files_only ;;
    control-api|api) update_control_api ;;
    all)          update_edge; update_control_api ;;
    *)
      echo "unknown target: $t"
      echo "valid: edge | control-api | ui | self | all"
      exit 2
      ;;
  esac
done

echo ">> done. Pi is now running $SHA"
echo ">> hard-refresh any open UI tabs (Ctrl-F5) to drop cached app.js/index.html"
