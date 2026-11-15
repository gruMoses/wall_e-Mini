# WALL-E Mini — Codebase Guide

WALL-E is a real robot that can injure a person. The safety rules below are the point of this file.

## Safety

### Ask before the wheels can move (Kevin, 2026-11-14)
- Before any command that can spin the wheels, ask Kevin whether the wheels are blocked, and wait for his answer.
- This covers drive forward or back, a turn, any VESC command, engaging follow-me, and any test with wheel movement.
- Ask every time, even if he said so earlier in the session.
- Reason: the robot can drive over a person or through the garage door. Kevin has been hurt or nearly hurt before.

### Deploys and services
- A push to `main` that changes motor, follow-me or startup code reaches the robot within 5 minutes. Tell Kevin before that push.
- Tell Kevin before a commit that will restart the `wall-e` service.
- Stopping or killing a service on the robot needs Kevin's go-ahead in the same turn.

### Safety principle (Kevin, 2026-09-27) — apply to every motion change
- Human input can carry a little extra risk. Autonomous input must not cause human harm.
- MANUAL (RC sticks or phone): the operator stops the robot by letting go of the stick.
- FOLLOW_ME and WAYPOINT_NAV: the operator must take the remote out of a pocket and disarm, which takes much longer.
- So a MANUAL relaxation is acceptable (the creep floor, the MANUAL obstacle curve for netting over the lens). FOLLOW_ME and WAYPOINT_NAV keep every protection.
- The person/animal hard stop (0.8 m, `safety_stop_radius_m`) is absolute in every mode.
- The 0.8 m stop zeroes the forward part only; a pivot in place passes, in FOLLOW_ME too (Kevin judged it fine, 2026-11-08). Do not freeze the pivot during the stop without his word.
- Kevin removed the person/animal cap on the MANUAL obstacle curve (2026-09-27): it switched off the netting relief whenever the camera saw him but could not range him through the mesh. Do not add it back without his word.

### Person stop hold (2026-11-10, `pi_app/control/person_stop_hold.py`)
- In FOLLOW_ME and WAYPOINT_NAV the 0.8 m stop has memory: after a trigger, forward stays at zero until the same object is seen beyond 1.0 m (3 polls, then 5 polls of evidence). The corridor is never release evidence.
- Object lost for more than 0.6 s, or a second person/animal close: the hold becomes operator-only (arm switch off, FOLLOW_ME off and on, or stop and GO on the waypoint page).
- A hold active at a change to MANUAL stays until the sticks are at neutral for 0.25 s. MANUAL engages no new hold (`person_stop_hold_in_manual = False`).
- Do not set `person_stop_hold_in_manual` to True, and do not add a timer release or a corridor release, without Kevin's word AND an independent safety review by a second model (Grok). Five reviews were needed (`docs/status_audit_2026-11-10.md` §9.4).
- Not yet tested with a person on the robot.

### Open safety items
- Arming with the transmitter on is not tested since the monotonic-clock change (2026-11-10). Test it before the next drive. Observation, 2026-11-15, on the stand: the robot armed three times with the transmitter on and drove for 326 s (logs `arm_20261115_1358*`). Kevin has not closed this item.
- The pack-low motor cutoff is now `voltage_shutdown_threshold_v = 44.0` in `config.py` (the old 39.0 V was below the 40.0 V BMS trip and could never fire). The old CLAUDE.md still listed this as open; confirm with `docs/status_audit_2026-11-10.md` before relying on it.
- The IMU-stall watchdog is not verified on a real stall. The new Arduino RC sketch is not flashed yet (flash only with `wall-e` stopped and Kevin's word).

## Do not (each has cost a drive or nearly hurt someone)
- IMU sign: if steering turns the wrong way, fix the sign at the reader and re-chalk. Never add a second inversion downstream (two compensating inversions made the D-term anti-damping, fixed 2026-09-19).
- `OakDetectionConfig.camera_fps = 15.0` is load-bearing. Never remove it without re-measuring `oak.det_latency_s` (latency was 0.80 s at 8 fps, 2026-09-20).
- Arms-up twitch latch (`POST /api/arms_up/twitch_test`): do not reuse it for the continuous back-up feature; that needs its own latch and its own independent safety review by a second model (Grok) (2026-09-22).
- Trail follow stays off (`trail_follow_enabled = False`, Kevin 2026-11-10). Do not turn it on before the field checks in `tools/odometry_analysis/README.md`, an independent safety review by a second model (Grok), and Kevin's approval.
- Do not propose again a ramp on the first command after re-arm (`snap_first_command`) or a charger latch / "charger unplugged" button. Kevin ruled both out on 2026-11-10.
- Netting mute is explicit only (phone button, armed, MANUAL, netting already in view). Do not add an automatic engage on depth + odometry: a missed close person, wheel slip or a pushed object passes the same test (2026-10-08). The 0.8 m stop and stale depth are never muted.
- Do not make `get_person_detections()` return `[]` when the stream is stale. A stale close person must still force the stop; follow-me drops out on `get_detection_freshness()` instead (a stale list drove a 44 s left circle, 2026-09-25).
- The OAK stays at USB 2 (`max_usb_speed = "HIGH"`) on purpose: USB 3 jams the RTK GPS (2026-11-06). Do not set `"SUPER"` without the GPS re-test.
- Do not restore `imu_source = "auto"` (opposite yaw-rate sign) or `oak_yaw_rate_scale` 0.46.
- Never bypass the slew limiter on a mode exit or a stop release: a held stick would jump to the motors.
- Never re-apply a heading target every tick in MANUAL (it dragged the robot back toward old headings, 2026-09-27).
- Accepted residual risk: with the transmitter switched off, the receiver failsafe reads like switch-off on ch3, so the robot re-arms when the transmitter comes back with the switch up.

## Incidents (details in `docs/known-issues-history.md`)
- 2026-09-20: FOLLOW_ME near-run-over; identity gates now never hide a closer range from the speed command.
- 2026-09-25: CAM_A stall; stale detections drove FOLLOW_ME in a left circle for 44 s.
- 2026-09-25: I2C bus 1 lock-up, GPS and UPS dark for 17 h (`docs/i2c_bus_guard.md`).
- 2026-09-27: Wi-Fi `no-secrets` block, robot off the network 36 min (`docs/wifi_watchdog.md`).
- 2026-10-04 to 10-08: OAK vision worker hung in depthai teardown for 4 days.
- 2026-11-03 to 11-10: UPS HAT (B) halts and flat-pack saga (`docs/ups_hat_b.md`).
- 2026-11-05: a held stick went neutral to full reverse in one tick after a stop release.
- 2026-11-05: HotRC CH3/CH4 hybrid-mode fault; arm channel fail-safe added.
- 2026-11-06 to 11-08: IMU stream stopped silently; heading frozen 38.8 h.
- 2026-11-08: one bad close frame released the 0.8 m stop for 0.66 s (led to the stop hold).

## Architecture
Python control stack on a Raspberry Pi 5. Entry point `pi_app/app/main.py`: a fast control loop (about 30 Hz) plus an SSE web server on port 8080. Subsystems run in threads and share state through small dataclasses; the main loop reads the latest snapshots and publishes motor commands. Vision runs at 15 fps. All settings are in one frozen `Config` dataclass in `config.py` (repo root).

- Motors: VESC over CAN (`can0`) is the only drive path. Commands are bytes 0–254, 126 = stop. The VESC stops on its own after ~1 s without a command.
- RC: an Arduino streams the RC channels over USB. ch3 arms (held high), ch5 is the e-stop (it powers the Pi off), ch4 starts FOLLOW_ME. Details: `docs/rc_arduino_gps_wiring.md`.
- Camera: OAK-D Lite for depth corridor, YOLOv8n person/animal detection and the onboard BMI270 IMU (no magnetometer). Geometry comes from the EEPROM via `OakDepthReader.get_intrinsics()`.
- Heading: gyro-integrated, clockwise-positive, relative to boot; GPS COG alignment can lock it to true north.
- Follow-me: direct pursuit only; a lost target pivots in place, then stops.
- Waypoint nav: ALIGN / DRIVE / ARRIVE; needs RTK fixed, a fresh heading and fresh vision to start.
- BMS: JHB-R24TH-V1.3 over BLE; feeds the charger inhibit. Fails open after 30 s BLE dropout.
- Pi power: a Geekworm X1202 board (2 cells) under the Pi. `walle-power` halts the Pi 5 s after the input goes off, and it keeps the cells near 3.75 V with charge bursts of 2 min (`--hold` in the unit, Kevin 2026-11-15); the board then cuts the 5 V and starts the Pi when the input returns. Stay-off gotcha: after a halt with the input present (ch5 e-stop, `poweroff`), or an input-off of about 5 s to 14 s, the Pi stays off until the input is off for 30 s (or the button on the board). The system journal of a halted boot stays only because the clock unit `walle-clock-floor` is installed (the Pi 5 RTC has no cell); the sure record of a halt is `/var/lib/walle-power/events.jsonl`. Never make a 12 V lead with an XH 2-pin plug: it fits the 5 V outputs of the X1202 and the 5 V inputs of the shield; 12 V goes into the barrel jack (Kevin, 2026-11-15). Storage rule (Kevin, 2026-11-15): parked for more than 6 weeks, remove the two cells; the board takes 0.85 mA from them with all power off and has no low-voltage cutoff. Status (2026-11-15): on the robot, on the stand. The motors do not disturb the input. A charge burst with a full CPU load gives an input dip of approximately 0.1 s (the burst stops, no halt); with the usual load the burst is correct. The 12 V at the plug stays at 12.03 V or more, thus the limit is in the board, not in the converter (10 A, on a 5 A fuse). Go/no-go not yet (`docs/pi_power_architecture.md` 7.6).
- Root services in `bin/`: power watcher (`walle-power`; it replaced `walle-ups` on 2026-11-14), I2C bus guard, Wi-Fi watchdog, auto-deploy. The power watcher and the I2C bus guard do not restart on auto-deploy; reinstall them by hand (`bin/install_walle_power.sh`).

## Tests
```bash
cd /home/pi/wall_e-Mini
python3 -m pytest pi_app/tests -q
```
Use pytest, not `unittest discover` (it collects zero cases from the pytest-style files).

## Deploy model
- `main` auto-deploys to the robot every 5 minutes (cron, `bin/auto_deploy.sh`, `docs/auto_deploy.md`).
- Arm guard: no pull while the robot reports armed; a held deploy lands within 5 minutes of disarm.
- A change only under `docs/` or to a root `*.md` pulls without a service restart. Any other change restarts `wall-e` if the smoke test passes (a failed smoke test rolls back).
- Kill switch: a `.deploy_hold` file in the repo root on the Pi holds all deploys.

## Docs index — read when
- `docs/known-issues-history.md`: before work on any subsystem with a past incident; full known-issues text, branches, the old field-test handoff.
- `docs/status_audit_2026-11-10.md`: before you plan work; open safety gaps in priority order.
- `docs/subsystems-reference.md`: motor output, waypoint nav, BMS, logging detail.
- `docs/camera-calibration.md`: before you touch any camera geometry, FOV or height filter.
- `docs/follow-me-tuning.md`: before you change follow-me gains, speed, corridor or arms-up.
- `docs/imu-heading.md`, `docs/heading_tuning.md`, `docs/gps_heading_alignment.md`: before you change IMU, heading or ZUPT code.
- `docs/rc_arduino_gps_wiring.md`: RC, arming, Arduino sketch and flashing.
- `docs/x1202_power_watcher_design.md`, `docs/pi_power_architecture.md` (section 7): the X1202 power board, the `walle-power` watcher, bench results and open items. `docs/ups_hat_b.md` is the retired UPS HAT (B).
- `docs/i2c_bus_guard.md`, `docs/wifi_watchdog.md`: the root watchdog services.
- `docs/depthai_upgrade.md`: depthai 3.6.1, IMU report types, VIO.
- `docs/auto_deploy.md`: the deploy script.
- `docs/logging_troubleshooting.md`: log line formats.
- `docs/wheel_speed_scale_2026-11-10.md`: `wheel_ground_scale` 0.815; do not change one speed constant alone.
- `docs/NEXT_SESSION.md`: backlog and branch restore SHAs.
