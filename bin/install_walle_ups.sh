#!/usr/bin/env bash
# Install or update the WALL-E UPS watcher (Waveshare UPS HAT (B)) on the Pi.
#
#   sudo bash bin/install_walle_ups.sh
#
# It copies bin/walle-ups.service and bin/walle-ups-wakecheck.service to
# /etc/systemd/system, reloads systemd, enables both, and restarts the
# watcher so it runs the code in this checkout. The wake check is a oneshot
# that only matters at boot. Auto-deploy does NOT restart this service:
# re-run this script after changing bin/walle_ups.py or either unit file
# (for example when --supply-v changes from 8.4 to 7.6 with the robot buck).
# See docs/ups_hat_b.md.
set -euo pipefail

if [[ $(id -u) -ne 0 ]]; then
  echo "Run this with sudo: sudo bash bin/install_walle_ups.sh" >&2
  exit 1
fi

REPO_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
STATE_DIR=/var/lib/walle-ups

install -d -m 0755 "$STATE_DIR"
for unit in walle-ups.service walle-ups-wakecheck.service; do
  install -m 0644 "$REPO_DIR/bin/$unit" "/etc/systemd/system/$unit"
done
chmod +x "$REPO_DIR/bin/walle_ups.py"

systemctl daemon-reload
systemctl enable walle-ups-wakecheck.service walle-ups.service
systemctl restart walle-ups.service
sleep 1
systemctl status walle-ups.service --no-pager | head -12
echo
echo "Reading now:"
/home/pi/.venv/bin/python3 "$REPO_DIR/bin/walle_ups.py" once
