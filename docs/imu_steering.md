# IMU Steering Compensation

This document describes the IMU-based steering compensation system for the WALL-E Mini robot.

## Overview

The IMU steering compensation system uses an IMU (accelerometer + gyroscope) to:
- **Compensate for bumps and terrain variations** that cause unintended turning
- **Maintain straight-line travel** when no steering input is given
- **Provide smooth corrections** for external disturbances

## Hardware

The active IMU is the **OAK-D Lite onboard BMI270** (gyroscope + accelerometer only — **no magnetometer**). Heading is integrated from the gyro and is **relative to the robot's orientation at startup**, not referenced to magnetic north.

The system also supports external I2C breakout boards (ICM-20948, or ISM330DHCX + MMC5983MA combo). Source priority is controlled by `imu_source` in `config.py` (default `"auto"`: external first, OAK-D BMI270 fallback). The magnetometer is disabled by default (`imu_use_magnetometer = False`).

## How It Works

### 1. PID Control Loop
The system implements a PID controller:
- **Proportional (P)**: Corrects based on current heading error
- **Integral (I)**: Accumulates and corrects for systematic drift
- **Derivative (D)**: Dampens oscillations using yaw rate

### 2. Steering Intent Detection
- Compensation is strongest in straight-intent conditions and scales down as steering input grows.
- Neutral/intent handling is controlled by hysteresis thresholds in `config.py`.

### 3. Fallback Behavior
- If IMU is unavailable or fails, the system automatically falls back to RC-only control
- No manual intervention required

## Configuration

Edit `config.py` to adjust IMU steering parameters:

```python
@dataclass(frozen=True)
class ImuSteeringConfig:
    enabled: bool = True
    kp: float = 1.7
    ki: float = 0.4
    kd: float = 0.08
    max_correction: int = 220
    deadband_deg: float = 0.9
    max_integral: float = 80.0
    steering_neutral_enter: float = 0.08
    steering_neutral_exit: float = 0.15
    neutral_dwell_s: float = 0.0
    straight_equal_tolerance_us: int = 120
    straight_min_throttle_us: int = 80
    straight_relative_tolerance_pct: float = 0.35
    straight_disengage_hysteresis_s: float = 0.80
    correction_zero_at_steering: float = 0.50
    update_rate_hz: float = 80.0
```

Magnetometer fusion is off by default (`imu_use_magnetometer = False`). If an external I2C IMU with a magnetometer is present and you want to use it, set `imu_use_magnetometer = True` in `Config`.

### Tuning Guidelines

1. **Start with low gains**: Begin with the default values
2. **Increase P gain**: For faster response to heading errors
3. **Add I gain**: To eliminate steady-state drift
4. **Add D gain**: To reduce oscillations and overshoot
5. **Adjust deadband**: Increase if corrections are too sensitive

## Installation

1. **Install dependencies**:
   ```bash
   cd /home/pi/wall_e-Mini
   pip install -r requirements.txt
   ```

2. **Connect IMU hardware** to I2C bus

3. **Test IMU functionality** (available tools):
   ```bash
   cd /home/pi/wall_e-Mini
   python3 -m pi_app.cli.calibrate_imu
   python3 -m pi_app.cli.advanced_imu_calibration
   python3 -m pi_app.cli.imu_axis_mapping_tuner
   python3 -m pi_app.cli.imu_cli_manual
   ```

## Runtime Source Selection

`config.py` controls IMU source priority:
- `imu_source="auto"`: external I2C IMU first, fallback to OAK-D IMU
- `imu_source="external"`: external only
- `imu_source="oak_d"`: OAK-D onboard IMU only
- `imu_source="none"`: disable IMU steering

## Integration

The IMU steering compensation is automatically integrated into the main control loop:

1. **Controller initialization**: IMU compensator is created if IMU is available
2. **Real-time compensation**: Corrections are applied during each control cycle
3. **Automatic fallback**: RC-only control if IMU fails

## Monitoring

The system provides real-time status information:

```python
# Get IMU status from controller
imu_status = controller.get_imu_status()
if imu_status:
    print(f"Heading: {imu_status['heading_deg']:.1f}°")
    print(f"Target: {imu_status['target_heading_deg']:.1f}°")
    print(f"Available: {imu_status['is_available']}")
```

## Troubleshooting

### IMU Not Detected
- Check I2C connections (for external breakout)
- Verify OAK-D is connected and OAK pipeline is running (for BMI270)
- Run `i2cdetect -y 1` to scan for external devices

### Excessive Corrections
- Reduce `kp` gain
- Increase `deadband_deg`

### Oscillations
- Reduce `kp` gain
- Increase `kd` gain
- Check for loose hardware

### Drift Issues (heading wandering over time)
- Expected with gyro-only integration — the BMI270 has no magnetometer
- Increase `ki` gain to correct slow bias
- GPS COG heading alignment locks a true-north offset while driving on RTK fixed; see `docs/gps_heading_alignment.md`

## Safety Features

- **Maximum correction limits**: Prevents excessive steering adjustments
- **Automatic fallback**: RC control always available
- **Error detection**: System disables IMU if too many errors occur
- **Configurable timeouts**: Prevents system lockup

## Performance Characteristics

- **Update rate**: 20 Hz (configurable)
- **Latency**: < 50ms from IMU reading to motor correction
- **Accuracy**: ±2° heading hold (configurable deadband)
- **Fallback time**: Immediate (< 1ms) when IMU unavailable
