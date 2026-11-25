# Phone app over Tailscale HTTPS (2026-11-25)

The drive page (`/drive`) already has an app manifest (name "WALL-E Drive", standalone, icons; `pi_app/web/teleop.py`). Android Chrome installs a standalone app only from an HTTPS address. The robot serves plain HTTP on port 8080, so Tailscale gives it HTTPS.

## What is on the robot

- Tailscale 1.104.5 from the official apt repo (`/etc/apt/sources.list.d/tailscale.list`, pkgs.tailscale.com, bookworm). Installed by hand on 2026-11-25 with Kevin's OK. Auto-deploy does not manage it.
- History: installed 2026-03-28, removed 2026-03-29 with no recorded reason (`/var/log/apt/history.log`). The old node key expired 2026-09-25. Re-enrolled 2026-11-25 as the same node, `walle`, 100.66.162.38.
- Started with `tailscale up --reset --accept-dns=false --hostname=walle`.
  - `--accept-dns=false`: Tailscale must not take over `/etc/resolv.conf`. The robot needs its normal DNS (192.168.86.1) for the auto-deploy `git fetch` and NTP.
  - No `--accept-routes`: the old state had it on. It would let another tailnet device's subnet route capture the robot's home-network traffic (charger plug, base listener).
- HTTPS: `tailscale serve --bg --https=443 http://127.0.0.1:8080`. The setting persists across reboots. Address: `https://walle.tail67f7fa.ts.net` (tailnet only). WARNING: never turn on Funnel for this. The web POST routes have no authentication except the drive token (`docs/status_audit_2026-11-25.md`, section 2, item 3).
- Node key expiry: 2027-05-24. To avoid a re-approval then, disable key expiry for `walle` in the Tailscale admin console (Kevin's account).

## Checked after the install (2026-11-25)

- `/etc/resolv.conf` unchanged. Default route still `wlan0` via 192.168.86.1.
- `git fetch` to GitHub works, NTP synchronised. `wall-e`, `wifi-watchdog`, `i2c-bus-guard` and `walle-power` are active. `wall-e` was not restarted.
- `/drive`, `/drive/manifest.json`, `/` and the SSE stream `/api/telemetry` work through HTTPS. The first request took 29 s (certificate issue); after that 0.18 s, the same as the direct LAN address. `tailscale ping walle` from the Mac goes direct over the LAN (12 ms), so the phone override's 0.6 s age limit has a large margin.

## Install the app on Android

1. The phone runs the Tailscale app, signed in to the same account, connected.
2. In Chrome, open `https://walle.tail67f7fa.ts.net/drive`.
3. Menu (three dots), then "Install app" (or "Add to Home screen", then "Install").
4. The first start asks for the drive token one time, because a new address has its own storage. It is the same token as before. It is in `/home/pi/.config/wall_e/teleop_token` on the robot. Never put it in the URL of the home-screen icon.

CAUTION: Tailscale makes the drive page reachable from anywhere on the tailnet, also off the home network. MANUAL assumes that the operator sees the robot (he stops it by letting go of the stick). Drive only with the robot in sight.

## Undo

- HTTPS off: `sudo tailscale serve --https=443 off`.
- Tailscale off: `sudo tailscale down` (keeps the package), or `sudo apt remove tailscale`.
