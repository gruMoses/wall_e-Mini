Moved out of CLAUDE.md on 2026-11-14 (text kept as it was: architecture, motor output, waypoint navigation, BMS, config, logging and tests).

## Architecture Overview

Python control stack running on a Raspberry Pi 5. Entry point is `pi_app/app/main.py` (a tight control loop at ~30 Hz; vision is 15 fps via `OakDetectionConfig.camera_fps`, not 30). All subsystem state is threaded and shared via lightweight dataclasses; the main loop reads latest snapshots and publishes commands.

```
pi_app/
  app/main.py          # Main loop, subsystem wiring, SSE web server (port 8080)
  control/             # Logic: controller, safety, follow_me, waypoint_nav, obstacle_avoidance, imu_steering
  hardware/            # Drivers: vesc, arduino_rc, oak_depth, oak_imu, imu_reader, bms, rtk_gps
  cli/                 # Calibration and test harnesses
  tests/               # Unit tests
  web/                 # Flask UIs: viewer, teleop, waypoint nav, property map
config.py              # Single Config dataclass (root of repo)
bin/                   # root services: UPS watcher, I2C bus guard, Wi-Fi watchdog, auto-deploy
tools/                 # log analyzers, replay tools, bench scripts
```

## Key Subsystems

### Motor Output
VESC over CAN (`can0`) is the only drive path. The Arduino only streams the RC channels over USB; its old Model X motor-driver board is removed (Kevin, 2026-11-13), so the Model X fallback code drives nothing. The repo sketch (since 2026-11-13: no Model X code, 6 fields with a CRC-8, sentinel 0 for no signal) is NOT flashed yet; the Arduino still runs `dcc550e`, and the Pi accepts both formats. Wiring and open checks: `docs/rc_arduino_gps_wiring.md`. All motor commands are "bytes" (0–254, centre 126 = stop; `pi_app/control/mapping.py`). Left/right are issued separately via `VescCanDriver`. A command-space mixer converts `(v_cmd, yaw_cmd)` floats in `[-1, 1]` into left/right bytes.

### Waypoint Navigation
State machine in `pi_app/control/waypoint_nav.py`:
- **ALIGN**: pivot in place until heading error < `align_threshold_deg` (default 12°). Yaw sign: positive yaw_cmd for positive (right) heading error. The pivot is proportional to the error (max `pivot_yaw_cmd` 0.35 at ≥90°, floor `pivot_yaw_min` 0.18); a fixed 0.5 pivot overshot 25–30° and oscillated with DRIVE (2026-09-19).
- **DRIVE**: forward at cruise speed with PID steering; falls back to ALIGN if error exceeds `recovery_threshold_deg`.
- **ARRIVE**: within `arrival_radius_m` of the last waypoint; zeroes commands. An intermediate arrival advances the index and goes straight to ALIGN for the next waypoint, with no stop.
- `compute()` returns `(v_cmd, yaw_cmd, state)` as floats; caller's mixer converts to motor bytes.
- Default gate requires exactly RTK fixed quality 4; RTK float quality 5 is rejected. Stale GPS (>3 s) halts motion.
- Start gate and run guard (2026-11-10): `Controller.waypoint_nav_activation_blocked` refuses a start when there is no GPS reading (`gps_state_unknown`), when the reading is older than the stale timeout (`gps_stale`), when the detection stream is stale (`vision_stale`), when there is no heading source, when the heading is older than `heading_stale_s` (0.5 s), and when the heading lock is absent. During a run, `_leave_waypoint_nav_if_unsafe` checks the heading and the detection stream on each tick, and it stops one ALIGN episode after `align_timeout_s` (10 s). An exit sets MANUAL and ramps to the sticks through the slew limiter.
- API limits (2026-11-10): `/api/nav/start` and `/api/nav/go` clamp `cruise_speed` to 10–80 and `arrival_radius` to 0.3–3.0 (the slider ranges, one definition in `pi_app/web/waypoint_nav_ui.py`). A speed of 0 or less gets HTTP 400. The routes ask the gate before they write, and they restore the mission from `snapshot_mission` if the activation fails, so a refused start does not change a mission that is in progress.

### BMS
JHB-R24TH-V1.3 (Daly-app compatible, 24S 40 A hardware; `docs/battery_settings.md`) over BLE (`bleak`); the Bluetooth radio is built into the BMS. The `config.py` docstring still says "SPIM08HP", which is wrong. (Checked 2026-11-14: that is stale; the `BmsConfig` docstring in `config.py`, line 1378, now says JHB-R24TH-V1.3.) Polls SOC / cell voltages / temp / MOSFET status. Auto-reconnect. `is_charging()` feeds charger-inhibit into controller every loop tick. Fail-open after 30 s BLE dropout. The long dark periods match the BMS's own sleep (default 3600 s with no current; it wakes on current). Setting the sleep to 65535 in the Daly app would stop them at about 25 mA of standby drain; Kevin, 2026-11-13: not yet (`docs/rc_arduino_gps_wiring.md`, section 11).

## Config

`config.py` (repo root) — one frozen `Config` dataclass with nested sub-configs. Edit this to change thresholds, enable/disable subsystems, set BMS MAC, VESC CAN IDs, etc.

## Logging

`logs/run_*.log` / `arm_*.log` mix four JSON line shapes (per-tick, 1 Hz "slow", session header, event) — see `docs/logging_troubleshooting.md` for the field-by-field format. While armed, a tick with an event always writes a line (2026-11-10); before that date, events on ticks that the 10 Hz gate skipped are not in the armed logs.

## Running Tests

```bash
cd /home/pi/wall_e-Mini
python3 -m pytest pi_app/tests -q
```

Use pytest, not `unittest discover`: eight test files are pytest-style (plain
`def test_` functions, among them `test_teleop.py` with the phone deadman and
`test_spp_control_gate.py`), and `unittest discover` collects zero cases from
them (found 2026-11-03).

