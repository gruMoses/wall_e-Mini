# Testing Guide

## Unit Tests

Run full unit suite:

```bash
cd /home/pi/wall_e-Mini
python3 -m unittest discover -s pi_app/tests -p "test_*.py"
```

Key coverage areas:

- mapping and byte conversion
- safety state machine (arm/disarm/emergency/follow)
- controller behavior (manual, obstacle, follow, waypoint integration)
- obstacle avoidance math
- follow-me targeting
- hardware selection logic

## Hardware-in-Loop Checks

### Arduino RC stream

```bash
cd /home/pi/wall_e-Mini
python3 -m pi_app.cli.monitor_rc
```

Verify all five channels update.

### OAK-D depth and detections

```bash
cd /home/pi/wall_e-Mini
python3 -m pi_app.cli.test_oak_depth
```

Optional viewer:

```bash
python3 -m pi_app.cli.oak_live_view
```

### IMU calibration / diagnostics

```bash
cd /home/pi/wall_e-Mini
python3 -m pi_app.cli.calibrate_imu
python3 -m pi_app.cli.advanced_imu_calibration
python3 -m pi_app.cli.imu_axis_mapping_tuner
```

### RTK GPS sanity check

```bash
cd /home/pi/wall_e-Mini
python3 -m pi_app.cli.test_rtk_gps
```

## Service-Level Validation

```bash
systemctl status wall-e.service --no-pager
systemctl status wall-e-spp.service --no-pager
journalctl -u wall-e.service -n 200 --no-pager
```

## Performance Test Harness

For profiling and scripted load tests, see:

- `tools/perf/README.md`
