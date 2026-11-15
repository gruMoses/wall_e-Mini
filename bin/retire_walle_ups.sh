#!/usr/bin/env bash
# Retire the old UPS watcher (Waveshare UPS HAT (B), bin/walle_ups.py) on the Pi.
#
#   sudo bash bin/retire_walle_ups.sh
#
# Step 5 of the procedure in docs/x1202_power_watcher_design.md, section 8:
# run it only after walle-power.service is installed and the bench tests 1 to
# 9 passed. It disables and stops walle-ups.service and
# walle-ups-wakecheck.service, removes their unit files, the shutdown hook,
# /var/lib/walle-ups, /run/walle-ups and /tmp/ups_status.json, clears the RTC
# alarm and reloads systemd. It keeps the stop-timeout drop-ins (the new
# installer owns them). It does not edit /boot/firmware/config.txt.
set -euo pipefail

if [[ $(id -u) -ne 0 ]]; then
  echo "Run this with sudo: sudo bash bin/retire_walle_ups.sh" >&2
  exit 1
fi
say() { echo "==> $*"; }

if ! systemctl is-active --quiet walle-power.service || \
   ! systemctl is-enabled --quiet walle-power.service; then
  echo "ERROR: walle-power.service is not active and enabled. Install it first" >&2
  echo "       (sudo bash bin/install_walle_power.sh). Nothing changed." >&2
  exit 1
fi

for unit in walle-ups.service walle-ups-wakecheck.service; do
  say "Stop and disable $unit"
  systemctl stop "$unit" 2>/dev/null || true
  systemctl disable "$unit" 2>/dev/null || true
  say "Remove /etc/systemd/system/$unit and its drop-in directory"
  rm -f "/etc/systemd/system/$unit"
  rm -rf "/etc/systemd/system/$unit.d"
done

say "systemctl daemon-reload"
systemctl daemon-reload

say "Remove the shutdown hook /usr/lib/systemd/system-shutdown/walle-ups-alarm"
rm -f /usr/lib/systemd/system-shutdown/walle-ups-alarm

say "Remove /var/lib/walle-ups, /run/walle-ups and /tmp/ups_status.json"
rm -rf /var/lib/walle-ups /run/walle-ups
rm -f /tmp/ups_status.json

if [[ -w /sys/class/rtc/rtc0/wakealarm ]]; then
  say "Clear the RTC alarm (write 0 to /sys/class/rtc/rtc0/wakealarm)"
  if ! echo 0 > /sys/class/rtc/rtc0/wakealarm; then
    echo "WARNING: the RTC alarm write failed; check /sys/class/rtc/rtc0/wakealarm by hand." >&2
  fi
else
  say "No writable /sys/class/rtc/rtc0/wakealarm: nothing to clear"
fi

systemctl reset-failed walle-ups.service walle-ups-wakecheck.service 2>/dev/null || true

say "Kept: /etc/systemd/system.conf.d/10-walle-fast-stop.conf and /etc/systemd/user.conf.d/10-walle-fast-stop.conf"
say "Checks (read only)"
systemctl list-unit-files 'walle-ups*' --no-legend || true
ls /usr/lib/systemd/system-shutdown/ 2>/dev/null || true
echo "wakealarm: '$(cat /sys/class/rtc/rtc0/wakealarm 2>/dev/null || true)'"
