#!/usr/bin/env bash
# Install or update the clock unit that keeps the system journal of a halted
# boot on the Geekworm X1202 (docs/x1202_power_watcher_design.md, finding 3).
#
#   sudo bash bin/install_walle_clock_floor.sh
#
# Copies bin/walle-clock-floor to /usr/local/sbin/ and the unit to
# /etc/systemd/system/, then enables the unit. It runs at the next boot.
# Remove: sudo systemctl disable walle-clock-floor.service; sudo rm
# /etc/systemd/system/walle-clock-floor.service /usr/local/sbin/walle-clock-floor;
# sudo systemctl daemon-reload. No other file is changed.
set -euo pipefail

if [[ $(id -u) -ne 0 ]]; then
  echo "Run this with sudo: sudo bash bin/install_walle_clock_floor.sh" >&2
  exit 1
fi

REPO_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
say() { echo "==> $*"; }

say "Check the script"
sh -n "$REPO_DIR/bin/walle-clock-floor"

say "Copy the script to /usr/local/sbin/walle-clock-floor"
install -m 0755 -o root -g root "$REPO_DIR/bin/walle-clock-floor" /usr/local/sbin/walle-clock-floor

say "Install the unit /etc/systemd/system/walle-clock-floor.service"
install -m 0644 "$REPO_DIR/bin/walle-clock-floor.service" /etc/systemd/system/walle-clock-floor.service

systemctl daemon-reload
systemctl enable walle-clock-floor.service

say "Dry run (the clock is not changed):"
WALLE_CLOCK_FLOOR_DRY=1 /usr/local/sbin/walle-clock-floor
say "Done. The unit runs at the next boot."
