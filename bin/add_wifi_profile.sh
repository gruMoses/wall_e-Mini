#!/usr/bin/env bash
# Adds a Wi-Fi profile that copies the IPv4 settings of the main profile.
#
# Run it by hand, with sudo, on the Pi (Kevin types the password):
#   sudo bash /home/pi/wall_e-Mini/bin/add_wifi_profile.sh "wwr outdoor" -10
#
# Arguments: SSID [AUTOCONNECT_PRIORITY (default -10)] [TEMPLATE (default preconfigured)].
# It asks for the password without showing it. The password is never on the
# command line that sudo logs (sudo sees only this script and its arguments).
# It is on nmcli's own argument list for the few milliseconds nmcli runs.
# The profile gets autoconnect-retries 0 (NM never stops retrying) and the
# default auth-retries. See docs/wifi_watchdog.md.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "Run this with sudo: sudo bash $0 \"SSID\" [PRIORITY]" >&2
  exit 1
fi

SSID="${1:?usage: add_wifi_profile.sh SSID [PRIORITY] [TEMPLATE]}"
PRIO="${2:--10}"
TEMPLATE="${3:-preconfigured}"

if nmcli -t -f NAME connection show | grep -Fxq "$SSID"; then
  echo "A profile named '$SSID' already exists. Delete it first: nmcli connection delete \"$SSID\"" >&2
  exit 1
fi

METHOD=$(nmcli -g ipv4.method connection show "$TEMPLATE")
ADDR=$(nmcli -g ipv4.addresses connection show "$TEMPLATE")
GW=$(nmcli -g ipv4.gateway connection show "$TEMPLATE")
DNS=$(nmcli -g ipv4.dns connection show "$TEMPLATE")

read -rsp "Wi-Fi password for '$SSID': " PSK
echo
if [ ${#PSK} -lt 8 ]; then
  echo "A WPA password has at least 8 characters. Nothing was added." >&2
  exit 1
fi

args=(connection add type wifi con-name "$SSID" ifname wlan0 ssid "$SSID"
      wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PSK"
      connection.autoconnect yes connection.autoconnect-priority "$PRIO"
      connection.autoconnect-retries 0 ipv4.method "$METHOD")
if [ "$METHOD" = "manual" ]; then
  args+=(ipv4.addresses "$ADDR" ipv4.gateway "$GW" ipv4.dns "$DNS")
fi
nmcli "${args[@]}" >/dev/null
unset PSK args
echo "Added '$SSID': priority $PRIO, IPv4 $METHOD ${ADDR:-}. The current connection did not change."
