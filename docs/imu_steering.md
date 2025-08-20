# IMU Steering Compensation

This document describes the IMU-based steering compensation system for the WALL-E Mini robot.

## Overview

The IMU steering compensation system uses a 9-axis IMU (accelerometer, gyroscope, and magnetometer) to:
- **Compensate for bumps and terrain variations** that cause unintended turning
- **Maintain straight-line travel** when no steering input is given
- **Provide smooth corrections** for external disturbances

## Hardware Requirements

- **IMU**: SparkFun ISM330DHCX (6-axis accelerometer + gyroscope)
- **Magnetometer**: SparkFun MMC5983MA (3-axis magnetometer)
- **Connection**: I2C (Qwiic connector)

## How It Works

### 1. PID Control Loop
The system implements a PID controller:
- **Proportional (P)**: Corrects based on current heading error
- **Integral (I)**: Accumulates and corrects for systematic drift
- **Derivative (D)**: Dampens oscillations using yaw rate

### 2. Steering Input Detection
- **Neutral steering** (|input| < 0.1): IMU compensation active
- **Active steering** (|input| ≥ 0.1): IMU compensation disabled, target heading updated

### 3. Fallback Behavior
- If IMU is unavailable or fails, the system automatically falls back to RC-only control
- No manual intervention required

## Configuration

Edit `config.py` to adjust IMU steering parameters:

```python
@dataclass(frozen=True)
class ImuSteeringConfig:
    enabled: bool = True              # Enable/disable IMU steering
    
    # PID gains
    kp: float = 2.0                  # Proportional gain
    ki: float = 0.1                  # Integral gain  
    kd: float = 0.5                  # Derivative gain
    
    # Control parameters
    max_correction: int = 30         # Maximum correction (0-255)
    deadband_deg: float = 2.0        # Minimum error to trigger correction
    max_integral: float = 50.0       # Maximum integral term
    
    # Timing
    update_rate_hz: float = 20.0     # IMU update rate
    heading_hold_timeout_s: float = 0.5  # Heading hold timeout
```

### Tuning Guidelines

1. **Start with low gains**: Begin with the default values
2. **Increase P gain**: For faster response to heading errors
3. **Add I gain**: To eliminate steady-state drift
4. **Add D gain**: To reduce oscillations and overshoot
5. **Adjust deadband**: Increase if corrections are too sensitive

## Installation

1. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

2. **Connect IMU hardware** to I2C bus

3. **Test IMU functionality**:
   ```bash
   cd wall_e-Mini
   python3 pi_app/cli/test_imu.py
   ```

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
- Check I2C connections
- Verify power supply
- Run `i2cdetect -y 1` to scan for devices

### Excessive Corrections
- Reduce `kp` gain
- Increase `deadband_deg`
- Check for magnetic interference

### Oscillations
- Reduce `kp` gain
- Increase `kd` gain
- Check for loose hardware

### Drift Issues
- Increase `ki` gain
- Check IMU calibration
- Verify magnetometer orientation

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
