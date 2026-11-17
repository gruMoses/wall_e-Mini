# Charger plug: holding the main pack near 50 %

A TP-Link Kasa EP10 Wi-Fi plug powers WALL-E's charger (XM2703, 54.6 V 4 A).
The robot switches it from the BMS state of charge, so the pack no longer
sits at 100 % between drives. Set up 2026-11-16 (Kevin's design calls are in
the list at the end).

## Parts
- `pi_app/hardware/kasa_plug.py`: the plug driver. Original Kasa protocol,
  JSON over TCP 9999 with the XOR autokey cipher, plain sockets, no library,
  no cloud. One short connection per call.
- `pi_app/control/charge_policy.py`: the decision, pure and tested.
- `pi_app/control/charge_manager.py`: a thread that polls the BMS and the
  plug every 10 s, runs the policy, switches the plug, persists the mode in
  `logs/charger_state.json`.
- `config.py` `ChargerPlugConfig` (`config.charger_plug`): the plug address
  and every threshold. An empty `host` keeps the manager off.
- Web: `GET /api/charger`, `POST /api/charger/mode` with `{"mode": ...}`, and
  the "Charger Plug" panel on the status page with four buttons.

## Modes
| Mode | Plug | Ends |
|---|---|---|
| `hold` (default) | on at or under 45 %, off at or over 55 %; never on above 52.0 V | never |
| `full` | on until the BMS says full (SOC 100 % or pack 54.3 V) with the current under 0.3 A for 2 min, then off; a top-up at 95 % | by itself, once the pack has been above 60 % and then under it (a drive) |
| `on`, `off` | manual | never |

`hold` and `full` act only on a fresh BMS reading (link up, sample under
30 s old). Without one for 2 min the plug goes off and stays off until a
reading returns. Automatic switches are at least 60 s apart; an operator
mode change applies within a second.

On a system shutdown (ch5 e-stop halt, main switch off, `poweroff`) the
manager switches the plug off in `hold` and `full`. A plain `wall-e` restart
(every deploy) leaves the plug alone; the new process decides again within
a poll or two. A hard power loss of the Pi mid-charge leaves the plug on;
the charger then ends the charge itself at 54.6 V and the BMS trips sit
above that since 2026-11-10 (`docs/battery_settings.md`).

## Operating it
- Phone: the status page on port 8080, panel "Charger Plug": HOLD 50 % /
  CHARGE FULL / PLUG ON / PLUG OFF.
- Command line (any machine on the LAN):
  ```bash
  curl -s http://192.168.86.54:8080/api/charger
  curl -s -X POST -H 'Content-Type: application/json' -d '{"mode":"full"}' http://192.168.86.54:8080/api/charger/mode
  ```
- Journal: every switch and mode change is a WARNING line `Charger plug ON: ...`
  in `journalctl -u wall-e`.
- The plug itself: `kasa --host <ip> state` from any machine with python-kasa.

## Gotchas
- The 50 % hold never discharges anything. A pack at 100 % stays there until
  the next drive; the hold begins after that.
- A pack at 50 % gives about half the range. Say "charge it fully" the day
  before a long run; 50 % to full takes about 1.5 h at 4 A.
- The BMS state of charge is coulomb-counted and resets at full. After weeks
  at 50 % it drifts; the 52.0 V guard in `hold` catches a reading that is far
  too low. If the band looks wrong, one `full` cycle re-syncs the BMS.
- The plug needs a fixed address: give it a DHCP reservation on the Omada
  gateway, or `host` in `config.py` goes stale.
- The plug cannot be told to turn off on its own when the robot dies. See the
  shutdown paragraph above.

## Kevin's calls (2026-11-16)
1. The logic runs on the robot, not on the home dashboard.
2. No reading, no charging: the plug goes off when the BMS is dark.
3. Full mode returns to hold by itself after the robot has been used.
4. The robot's main switch is normally on while it sits on the charger, so
   the BMS reading is live and the hold works unattended.
