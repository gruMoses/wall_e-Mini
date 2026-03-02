# wall_e-Mini

Modular Raspberry Pi robot control stack for RC/manual drive, Bluetooth override,
IMU steering compensation, OAK-D obstacle avoidance + Follow Me, and optional RTK
waypoint navigation.

## Features

- RC input from Arduino over serial (`ch1..ch5`)
- Arm/disarm + emergency safety state machine
- Bluetooth SPP control path through external `wall-e-spp.service`
- VESC over CAN (`can0`) with Arduino motor-driver fallback
- IMU steering hold (external IMU preferred, OAK IMU fallback)
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

- Main loop consumes BT commands from `/tmp/wall_e_bt_latest.json` written by
  `wall-e-spp.service`.
- `run_main.sh` attempts to start UPS monitoring (`upsplus-power.service`) when
  UPS hardware is detected on I2C.
- Safety defaults:
  - `ch3` high/low arms/disarms
  - `ch4` high enters Follow Me, low exits
  - `ch5` high triggers latched emergency stop and shutdown sequence

## Tests

Run unit tests:

```bash
cd /home/pi/wall_e-Mini
python3 -m unittest discover -s pi_app/tests -p "test_*.py"
```

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

## Documentation Index

- `docs/services.md`: systemd services, startup flow, logs
- `docs/hardware_mapping.md`: RC channels, Arduino pins, motor pin map
- `docs/can_vesc.md`: CAN/VESC protocol and configuration
- `docs/testing.md`: unit + hardware-in-loop test workflow
- `docs/bluetooth_setup.md`: SPP pairing and transport details
- `docs/imu_steering.md`: IMU steering behavior and tuning
- `docs/oak_d_lite_integration_plan.md`: historical design plan (not current source of truth)
