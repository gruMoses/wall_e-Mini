#!/usr/bin/env bash
# Installs the I2C bus 1 auto-unlock guard as a systemd service.
#
# Run this by hand, with sudo, on the Pi:
#   sudo bash bin/install_i2c_bus_guard.sh
#
# It copies bin/i2c-bus-guard.service to /etc/systemd/system, reloads the
# systemd daemon, enables the service to start at boot, (re)starts it, and
# prints its status. Safe to run again after pulling a newer guard version
# -- `restart`, not just `enable --now`, is what makes a second run load
# the new code. See docs/i2c_bus_guard.md.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "Run this with sudo: sudo bash bin/install_i2c_bus_guard.sh" >&2
  exit 1
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UNIT_SRC="$REPO_DIR/bin/i2c-bus-guard.service"
UNIT_DST=/etc/systemd/system/i2c-bus-guard.service

if [ ! -f "$UNIT_SRC" ]; then
  echo "Cannot find $UNIT_SRC" >&2
  exit 1
fi

cp "$UNIT_SRC" "$UNIT_DST"
systemctl daemon-reload
systemctl enable i2c-bus-guard.service
systemctl restart i2c-bus-guard.service
systemctl status i2c-bus-guard.service --no-pager
