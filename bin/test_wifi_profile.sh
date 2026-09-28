#!/usr/bin/env bash
# Tries one Wi-Fi profile, checks its gateway, then goes back to the main
# profile. Run it detached, because the SSH session drops when the link
# changes:
#   sudo systemd-run --unit=wifi-profile-test --collect bash /home/pi/wall_e-Mini/bin/test_wifi_profile.sh "wwr outdoor"
# Read the result after 4 minutes: cat /tmp/wifi_profile_test.txt
# GATEWAY_OK means the profile reaches the robot's LAN with its IPv4
# settings. The test also gives NM a "last connected" time for the profile.
set -u
PROFILE="${1:?usage: test_wifi_profile.sh PROFILE [MAIN_PROFILE]}"
MAIN="${2:-preconfigured}"
exec >/tmp/wifi_profile_test.txt 2>&1
echo "start $(date '+%F %T') profile='$PROFILE' main='$MAIN'"
if nmcli --wait 60 connection up "$PROFILE"; then
  sleep 5
  ip -4 addr show wlan0 | grep inet || true
  GW=$(ip -4 route show default dev wlan0 | awk '/via/ {print $3; exit}')
  echo "gateway=${GW:-none}"
  if [ -n "${GW:-}" ] && ping -c 3 -W 2 -I wlan0 "$GW" >/dev/null; then
    echo GATEWAY_OK
  else
    echo GATEWAY_FAIL
  fi
else
  echo ACTIVATION_FAILED
fi
for i in 1 2 3; do
  if nmcli --wait 60 connection up "$MAIN"; then
    echo "back on '$MAIN'"
    break
  fi
  sleep 20
done
echo "end $(date '+%F %T')"
echo DONE
