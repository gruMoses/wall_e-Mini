#!/usr/bin/env bash
# Tries one Wi-Fi profile, checks its gateway, then goes back to the main
# profile. Run it detached, because the SSH session drops when the link
# changes:
#   sudo systemd-run --unit=wifi-profile-test --collect bash /home/pi/wall_e-Mini/bin/test_wifi_profile.sh "wwr outdoor"
# Read the result after 6 minutes: cat /tmp/wifi_profile_test.txt
# GATEWAY_OK means the profile reaches the robot's LAN with its IPv4
# settings. A valid ARP entry counts when the gateway drops ICMP, the
# same rule as bin/wifi_watchdog.py. The test also gives NM a "last
# connected" time for the profile.
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
  # Ping first. A router that drops ICMP still answers ARP, and the ping
  # is what fills the neighbour entry the fallback reads.
  if [ -n "${GW:-}" ] && ping -c 3 -W 2 -I wlan0 "$GW" >/dev/null; then
    echo GATEWAY_OK
  else
    NEIGH=""
    if [ -n "${GW:-}" ]; then
      NEIGH=$(ip neigh show "$GW" dev wlan0 2>/dev/null || true)
    fi
    NEIGH_UP=$(printf '%s' "$NEIGH" | tr '[:lower:]' '[:upper:]')
    if printf '%s' "$NEIGH_UP" | grep -q LLADDR \
        && ! printf '%s' "$NEIGH_UP" | grep -Eq 'FAILED|INCOMPLETE'; then
      echo GATEWAY_OK
    else
      echo GATEWAY_FAIL
    fi
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
