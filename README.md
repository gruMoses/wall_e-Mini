# wall_e-Mini

Modular Raspberry Pi robot control stack for RC/manual drive, phone drive (web
teleop), IMU steering compensation, OAK-D obstacle avoidance + Follow Me, and
optional RTK waypoint navigation.

## Features

- RC input from Arduino over serial (`ch1..ch5`)
- Arm/disarm + emergency safety state machine
- Phone drive through the web teleop (`pi_app/web/teleop.py`, page `/drive`):
  arming ceremony, latched e-stop, 250 ms deadman; RC stays the safety authority
- Bluetooth SPP diagnostics server (`wall-e-spp.service`); its control writes are
  gated OFF by default
- VESC over CAN (`can0`); the old Arduino motor-driver fallback drives nothing (the Model X board is removed)
- IMU steering hold (OAK-D onboard BMI270; `imu_source = "oak_d"`)
- OAK-D Lite obstacle avoidance + person-following mode
- Activity-triggered recording (H.265 + MCAP) and web viewer
- Optional RTK GPS waypoint navigation

## Repo Layout

- `pi_app/app/main.py`: main runtime loop and system wiring
- `pi_app/control/`: controller, safety, follow-me, obstacle avoidance, mapping
- `pi_app/hardware/`: RC reader, VESC, OAK, IMU, GPS drivers
- `pi_app/cli/`: calibration/testing/support tools
- `docs/`: subsystem docs and operational notes

## Quick Start

```bash
cd /home/pi/wall_e-Mini
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
cd pi_app
python3 -m app.main
```

Service-based runtime in production typically uses `run_main.sh` via
`wall-e.service`.

## Key Runtime Notes

- The phone drives through the web teleop (`pi_app/web/teleop.py`), which owns
  the command file `/tmp/wall_e_bt_latest.json`. The main loop reads that file
  and applies it only while it is fresh (600 ms). RC arming still gates the motors.
- `wall-e-spp.service` is a Bluetooth SPP diagnostics server. Its control writes
  to the command file are gated OFF by default (commit `a2628f0`, 2026-07-17;
  environment variable `WALL_E_SPP_CONTROL_ENABLED`). See `docs/spp_writer_audit.md`.
- The UPS watcher is a separate root service, `walle-ups.service`
  (`bin/walle_ups.py`, Waveshare UPS HAT (B)), installed by
  `bin/install_walle_ups.sh`. See `docs/ups_hat_b.md`.
- Safety defaults:
  - `ch3` high/low arms/disarms
  - `ch4` high enters Follow Me, low exits
  - `ch5` high triggers latched emergency stop and shutdown sequence

## Tests

Run unit tests:

```bash
cd /home/pi/wall_e-Mini
python3 -m pytest pi_app/tests -q
```

Use pytest, not `unittest discover`. Several test files use plain `def test_`
functions and no `TestCase`. `unittest discover` collects 0 cases from them and
reports no error.

## PID Debug Mode

Enable detailed steering PID telemetry with:

```bash
cd /home/pi/wall_e-Mini/pi_app
python3 -m app.main --pid-debug
```

The loop prints:

```text
PID err=<heading_error> P=<proportional> I=<integral> D=<derivative> yaw=<yaw_rate> int=<integral_state>
```

## Waypoint Navigation

The GPS waypoint navigator (`pi_app/control/waypoint_nav.py`) implements a three-state machine:

- **ALIGN**: pivot in place until heading error drops below threshold (default 12°)
- **DRIVE**: drive forward with PID steering; falls back to ALIGN on large errors
- **ARRIVE**: within arrival radius of target; advance to next waypoint

`compute()` returns `(v_cmd, yaw_cmd, state)` as normalized floats; a mixer converts these to left/right motor bytes. The default gate requires exactly fix quality 4 (RTK fixed); quality 5 is RTK float and halts autonomous motion.

GPS COG heading alignment is **implemented** (software); field validation is pending — see `docs/gps_heading_alignment.md`.

## Known Issues

- **VESC telemetry / RPM** — *resolved 2026-06-11*, bench-verified with
  `tools/vesc_rpm_bench.py` (wheels off ground): commanded +1500 eRPM read back
  with −1.0% / −0.4% steady-state error and zero parse errors. The old "always
  0" reading did not reproduce. The velocity PID is on (`speed_kp` 0.6,
  `speed_ki` 0.15, `speed_kd` 0.0 since 2026-09-19; see `FollowMeConfig` in
  `config.py`), and the 2026-09-20 follow-me logs show nonzero
  `left_rpm`/`right_rpm`.
- **IMU heading drift**: OAK-D Lite BMI270 is gyro+accel only (no magnetometer). Heading drifts over time and is relative to startup orientation.
- **Camera corridor — field-validated.** The camera geometry comes from the
  device EEPROM (measured 2026-07-26; several constants were corrected, see
  "Camera calibration" in `CLAUDE.md`). Follow-me and waypoint navigation were
  field-validated on 2026-09-19 and 2026-09-20 (see `CLAUDE.md`). Still open:
  `detect_min_person_height_m` 1.20 was tuned under the old vertical FOV and
  needs one recorded walk replayed through `tools/replay_follow_me_log.py`.

## Documentation Index

- `docs/services.md`: systemd services, startup flow, logs
- `docs/hardware_mapping.md`: RC channels, Arduino pins, motor pin map
- `docs/rc_arduino_gps_wiring.md`: wiring diagram for the RC receiver, Arduino and RTK GPS, safety findings, inputs for the Pi 5 shield spec
- `docs/can_vesc.md`: CAN/VESC protocol and configuration
- `docs/testing.md`: unit + hardware-in-loop test workflow
- `docs/bluetooth_setup.md`: SPP pairing and transport details
- `docs/imu_steering.md`: IMU steering behavior and tuning
- `docs/oak_d_lite_integration_plan.md`: historical design plan (not current source of truth)
