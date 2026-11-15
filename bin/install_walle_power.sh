#!/usr/bin/env bash
# Install or update the WALL-E power watcher (Geekworm X1202) on the Pi.
#
#   sudo bash bin/install_walle_power.sh
#
# Copies bin/walle_power.py to /usr/local/lib/walle-power/ (the unit runs that
# copy, so an auto-deploy pull does not change the running watcher), installs
# bin/walle-power.service, the poweroff.target drop-in and the existing
# stop-timeout drop-ins, reloads systemd, enables and (re)starts the unit.
# Re-run it after each change to bin/walle_power.py or to the unit.
# It stops first if the bootloader EEPROM does not have POWER_OFF_ON_HALT=1.
# It does not edit /boot/firmware/config.txt. See docs/x1202_power_watcher_design.md.
set -euo pipefail

if [[ $(id -u) -ne 0 ]]; then
  echo "Run this with sudo: sudo bash bin/install_walle_power.sh" >&2
  exit 1
fi

REPO_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
LIB_DIR=/usr/local/lib/walle-power
say() { echo "==> $*"; }

say "Check the bootloader EEPROM: POWER_OFF_ON_HALT=1"
if ! command -v rpi-eeprom-config >/dev/null 2>&1; then
  echo "ERROR: rpi-eeprom-config not found. Nothing installed." >&2
  exit 1
fi
if ! EEPROM=$(rpi-eeprom-config); then
  echo "ERROR: rpi-eeprom-config failed. Nothing installed." >&2
  exit 1
fi
if ! grep -Eq '^[[:space:]]*POWER_OFF_ON_HALT[[:space:]]*=[[:space:]]*1[[:space:]]*$' <<<"$EEPROM"; then
  echo "ERROR: the EEPROM does not show POWER_OFF_ON_HALT=1. Without it a halt" >&2
  echo "       does not turn the Pi off, and the board does not restart it." >&2
  echo "       Nothing installed." >&2
  exit 1
fi

say "Check that the script compiles"
/usr/bin/python3 -m py_compile "$REPO_DIR/bin/walle_power.py"

say "Copy the script to $LIB_DIR"
install -d -m 0755 "$LIB_DIR"
install -m 0755 "$REPO_DIR/bin/walle_power.py" "$LIB_DIR/walle_power.py"

say "Install the unit /etc/systemd/system/walle-power.service"
install -m 0644 "$REPO_DIR/bin/walle-power.service" /etc/systemd/system/walle-power.service

say "Install the poweroff.target drop-in (JobTimeoutSec=60s, poweroff-force)"
install -d -m 0755 /etc/systemd/system/poweroff.target.d
install -m 0644 "$REPO_DIR/bin/walle-poweroff-timeout.conf" \
  /etc/systemd/system/poweroff.target.d/10-walle-timeout.conf

REEXEC=0
install_dropin() {  # $1 = source in bin/, $2 = target; sets REEXEC=1 for system.conf.d
  if [[ -f "$2" ]] && cmp -s "$REPO_DIR/bin/$1" "$2"; then
    say "Stop-timeout drop-in $2 is current"
    return
  fi
  say "Install the stop-timeout drop-in $2"
  install -d -m 0755 "$(dirname "$2")"
  install -m 0644 "$REPO_DIR/bin/$1" "$2"
  if [[ "$2" == /etc/systemd/system.conf.d/* ]]; then REEXEC=1; fi
}
install_dropin walle-fast-stop-system.conf /etc/systemd/system.conf.d/10-walle-fast-stop.conf
install_dropin walle-fast-stop-user.conf /etc/systemd/user.conf.d/10-walle-fast-stop.conf

if [[ $REEXEC -eq 1 ]]; then
  say "systemctl daemon-reexec (a system.conf.d file changed)"
  systemctl daemon-reexec
fi
say "systemctl daemon-reload"
systemctl daemon-reload

say "Enable and (re)start walle-power.service"
systemctl enable walle-power.service
systemctl restart walle-power.service

say "Checks (read only)"
sleep 2
echo "walle-power: $(systemctl is-active walle-power.service || true)"
pinctrl get 6,16 || true
systemctl show poweroff.target -p JobTimeoutUSec -p JobTimeoutAction || true
echo "uptime: $(cut -d' ' -f1 /proc/uptime)"
cat /run/walle-power/status.json 2>/dev/null || echo "(no status file yet)"
echo
