# Wi-Fi reconnect watchdog

## 1. Purpose

The robot has one IPv4 path: Wi-Fi on `wlan0`. The watchdog makes sure that the robot gets back on the network when it comes back into Wi-Fi range. The service is `wifi-watchdog.service`. The script is `bin/wifi_watchdog.py`.

## 2. The incident that it corrects (2026-09-27)

1. At 17:35 the robot drove out of range of the 5 GHz access point.
2. NetworkManager (NM) 1.42 tried to associate three times. Each try timed out.
3. At 17:36:56 NM logged `failed (reason 'no-secrets')`. NM decided that the password was incorrect.
4. NM then blocked autoconnect for the `preconfigured` profile. The 300 s retry timer of NM does not clear this block.
5. At 18:05 the robot was back in range. It stayed off the network.
6. At 18:13 a power cycle brought it back. The Pi was up for the full time.

NM 1.42 clears a no-secrets block only on one of these events: a secret agent registers, networking restarts, the profile gets to IP configuration, or the profile gets new secrets. A disconnect during the WPA 4-way handshake always asks for new secrets. With no agent, that also ends in a no-secrets block, even with `connection.auth-retries 0`.

## 3. What the watchdog does

The watchdog runs each 15 s:

1. It reads the NM state of `wlan0`. State 100 is "connected". State 30 is "disconnected".
2. It acts on one of these conditions:
   - NM is idle: `wlan0` is in state 30 for 120 s. NM has stopped trying.
   - The robot is stuck: `wlan0` is not connected for 600 s, in any state. This covers an activation that never ends.
3. When it acts, it:
   1. Rescans.
   2. Keeps the saved autoconnect Wi-Fi profiles that NM lists as available on `wlan0` (`CONNECTIONS.AVAILABLE-CONNECTIONS`). NM checks the SSID and the band of the profile.
   3. Runs `nmcli --wait 150 connection up uuid <uuid> ifname wlan0` for the best profile. The best profile has the highest autoconnect priority, then the strongest signal.
4. `nmcli connection up` registers a secret agent for the call. That clears each no-secrets block.
5. A profile that failed in the last 600 s goes behind the other candidates. Two profiles that both fail are tried in turn.
6. Tries are at least 60 s apart, from the end of the last try. A failed try can take 150 s.
7. When `wlan0` is connected, the watchdog checks the default gateway each 30 s: one ping, then the neighbour (ARP) entry. A gateway with a valid link-layer address counts as "answers", so a router that drops ICMP never counts as down.
8. The watchdog stores each profile whose gateway has answered, in `/var/lib/wifi-watchdog/state.json`. The store survives a Wi-Fi blip and a reboot. A failed `nmcli` read skips that tick and changes nothing.
9. If the gateway has not answered for 180 s, and the active profile is not in that store, the profile counts as failed, and step 3 runs. An example is a profile on the wrong subnet.
10. If the active profile is in that store and the gateway then stops (for example, the router restarts), the watchdog writes one log line and does nothing. A new Wi-Fi connection cannot correct a router.

NOTE: The watchdog never stops an NM activation before the 600 s limit. It never takes down a link whose gateway has answered. It uses only the secrets that NM already has. It never reads, enters, or prints a password.

## 4. Install

1. Pull the repository on the Pi. The auto-deploy does this.
2. Run the installer:

   ```bash
   sudo bash /home/pi/wall_e-Mini/bin/install_wifi_watchdog.sh
   ```

3. Set the NM retry settings (section 5).
4. Examine the log:

   ```bash
   journalctl -u wifi-watchdog -n 20 --no-pager
   ```

   The first line is `Wi-Fi watchdog started on wlan0 (idle 120 s, stuck 600 s, gateway dark 180 s)`.

CAUTION: The auto-deploy does not restart this service. After a change to `bin/wifi_watchdog.py`, run the installer again.

## 5. NM profile settings

Set these on each Wi-Fi profile:

| Setting | Value | Reason |
|---|---|---|
| `connection.autoconnect-retries` | `0` (forever) | NM never stops its own retries. |
| `connection.auth-retries` | `0` while this is the only profile. `-1` after a fallback passes section 6. | `0` stops the no-secrets block after association timeouts. `-1` lets NM end a failed activation and try the other profile. |

NOTE: `preconfigured` is the only profile today. It keeps `connection.auth-retries 0` (set 2026-09-27 at 18:50). Leave that value while it is the only profile. With `0`, NM can loop on an access point that it sees but cannot join, and the watchdog waits for the 600 s limit. That wait is acceptable until a fallback exists. A drop during the WPA 4-way handshake can still end in a no-secrets block.

Set `preconfigured` to `-1` only after `wwr outdoor` returns `GATEWAY_OK`:

```bash
sudo nmcli connection modify preconfigured connection.auth-retries -1
```

The `preconfigured` profile has `802-11-wireless.band a` (5 GHz only). No record gives the reason. The 2.4 GHz radios of WHITE ROCK ROAD have a longer range.

## 6. Add the fallback profile `wwr outdoor`

Kevin types the password. The agent does not.

1. Run this command. It asks for the password and does not show it. The password is not in the command line that sudo logs:

   ```bash
   ssh -t pi@192.168.86.54 "sudo bash /home/pi/wall_e-Mini/bin/add_wifi_profile.sh 'wwr outdoor' -10"
   ```

   The script copies the IPv4 settings of `preconfigured` (static `192.168.86.54/22`). The current connection does not change.

2. Do a test of the profile one time. The test also gives NM a "last connected" time for the profile. Without that time, the first timeout asks for new secrets. The test changes back to `preconfigured` automatically:

   ```bash
   ssh pi@192.168.86.54 "sudo systemd-run --unit=wifi-profile-test --collect bash /home/pi/wall_e-Mini/bin/test_wifi_profile.sh 'wwr outdoor'"
   ```

3. After 6 minutes, read the result. The return to `preconfigured` can take three tries:

   ```bash
   ssh pi@192.168.86.54 cat /tmp/wifi_profile_test.txt
   ```

   - `GATEWAY_OK`: `wwr outdoor` is on the same LAN, and the static address works.
   - `GATEWAY_FAIL` or `ACTIVATION_FAILED`: delete the profile with `sudo nmcli connection delete "wwr outdoor"`. Do not change it to DHCP: a DHCP address on another subnet gets a gateway that answers, so the watchdog does not act, and the robot is not at `192.168.86.54`.

## 7. Technical names

Access point, ARP, autoconnect, DHCP, gateway, ICMP, IPv4, LAN, NetworkManager (NM), `nmcli`, ping, profile, secret agent, SSID, subnet, systemd, `systemd-run`, WPA, Wi-Fi, `wlan0`, WHITE ROCK ROAD, `wwr outdoor`, `preconfigured`.
