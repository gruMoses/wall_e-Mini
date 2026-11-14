#!/usr/bin/env bash
# Install or update the WALL-E UPS watcher (Waveshare UPS HAT (B)) on the Pi.
#
#   sudo bash bin/install_walle_ups.sh
#
# It copies bin/walle-ups.service and bin/walle-ups-wakecheck.service to
# /etc/systemd/system, installs the shutdown hook bin/walle-ups-shutdown-alarm
# and the stop-timeout limits bin/walle-fast-stop-{system,user}.conf, reloads
# systemd, enables both units, and restarts the watcher so it runs the code
# in this checkout. The wake check is a oneshot
# that only matters at boot. Auto-deploy does NOT restart this service:
# re-run this script after changing bin/walle_ups.py or either unit file
# (for example when --supply-v changes from 8.4 to 7.5 with the robot buck).
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

# Last-moment alarm: systemd-shutdown runs this just before power-off.
install -d -m 0755 /usr/lib/systemd/system-shutdown
install -m 0755 "$REPO_DIR/bin/walle-ups-shutdown-alarm" \
  /usr/lib/systemd/system-shutdown/walle-ups-alarm

# Bounded stop timeouts, so a hung service cannot hold a shutdown for 90 s.
install -d -m 0755 /etc/systemd/system.conf.d /etc/systemd/user.conf.d
install -m 0644 "$REPO_DIR/bin/walle-fast-stop-system.conf" \
  /etc/systemd/system.conf.d/10-walle-fast-stop.conf
install -m 0644 "$REPO_DIR/bin/walle-fast-stop-user.conf" \
  /etc/systemd/user.conf.d/10-walle-fast-stop.conf
systemctl daemon-reexec   # the system manager re-reads system.conf.d; the user manager at next login

systemctl daemon-reload
systemctl enable walle-ups-wakecheck.service walle-ups.service
systemctl restart walle-ups.service
sleep 1
systemctl status walle-ups.service --no-pager | head -12
echo
echo "Reading now:"
/home/pi/.venv/bin/python3 "$REPO_DIR/bin/walle_ups.py" once
