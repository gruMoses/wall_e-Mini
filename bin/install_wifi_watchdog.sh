#!/usr/bin/env bash
# Installs the Wi-Fi reconnect watchdog as a systemd service.
#
# Run this by hand, with sudo, on the Pi:
#   sudo bash bin/install_wifi_watchdog.sh
#
# It copies bin/wifi-watchdog.service to /etc/systemd/system, reloads the
# systemd daemon, enables the service to start at boot, (re)starts it, and
# prints its status. Safe to run again after pulling a newer watchdog --
# `restart`, not just `enable --now`, is what loads the new code. The
# auto-deploy does not restart this service. See docs/wifi_watchdog.md.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "Run this with sudo: sudo bash bin/install_wifi_watchdog.sh" >&2
  exit 1
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UNIT_SRC="$REPO_DIR/bin/wifi-watchdog.service"
UNIT_DST=/etc/systemd/system/wifi-watchdog.service

if [ ! -f "$UNIT_SRC" ]; then
  echo "Cannot find $UNIT_SRC" >&2
  exit 1
fi

cp "$UNIT_SRC" "$UNIT_DST"
systemctl daemon-reload
systemctl enable wifi-watchdog.service
systemctl restart wifi-watchdog.service
systemctl status wifi-watchdog.service --no-pager
