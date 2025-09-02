# IMU Troubleshooting Guide

This guide helps diagnose and fix common IMU (Inertial Measurement Unit) calibration and performance issues in your WALL-E Mini robot.

## Quick Diagnosis

Run the calibration validation script to check your current IMU status:

```bash
cd /home/pi/wall_e-Mini/pi_app/cli
python calibrate_imu.py --validate-only
```

## Common Issues and Solutions

### 1. Massive Heading Jumps (20-70° variations)

**Symptoms:**
- Heading changes dramatically between readings
- Robot steering corrections are erratic
- PID errors show huge values (>100°)

**Causes & Solutions:**

#### A. Magnetic Interference
**Cause:** Metal objects, motors, batteries, or power wires near IMU
**Solutions:**
- Move IMU away from metal components (minimum 6 inches)
- Use plastic mounting brackets instead of metal
- Route power cables away from IMU
- Shield power wires with ferrite beads
- Calibrate in an interference-free environment

#### B. Poor Magnetometer Calibration
**Cause:** Incomplete or incorrect calibration procedure
**Solutions:**
- Run full calibration: `python calibrate_imu.py --interactive`
- Ensure 360° rotation coverage during calibration
- Keep IMU level during magnetometer calibration
- Avoid metal objects within 10 feet during calibration

#### C. Soft Iron Distortion
**Cause:** Ferromagnetic materials causing field distortion
**Solutions:**
- Remove all metal objects from calibration area
- Calibrate outdoors away from buildings
- Use the soft iron calibration step in the calibration script

### 2. Gyroscope Drift

**Symptoms:**
- Heading slowly drifts over time
- Robot gradually turns without input
- Yaw rate shows non-zero values when stationary

**Causes & Solutions:**

#### A. Temperature Effects
**Cause:** Gyro bias changes with temperature
**Solutions:**
- Allow IMU to reach operating temperature before calibration
- Calibrate at expected operating temperature
- Monitor temperature in logs and recalibrate if it changes significantly

#### B. Vibration
**Cause:** Mechanical vibration affecting gyro readings
**Solutions:**
- Use vibration-damping mounts
- Ensure IMU is securely mounted
- Check for loose components causing vibration

### 3. Accelerometer Issues

**Symptoms:**
- Roll/pitch angles are incorrect
- Robot thinks it's tilted when it's level
- Accelerometer readings don't match gravity (should be ~9.8 m/s²)

**Causes & Solutions:**

#### A. Mounting Orientation
**Cause:** IMU not mounted with correct axis alignment
**Solutions:**
- Verify IMU axes match robot coordinate system
- Check mounting screws are tight
- Confirm axis directions in your code match physical orientation

#### B. Bias/Scale Errors
**Cause:** Accelerometer not properly calibrated
**Solutions:**
- Run interactive calibration: `python calibrate_imu.py --interactive`
- Place IMU in all 6 orientations during calibration
- Hold each position steady for 5+ seconds

### 4. Poor Calibration Quality Score

**Symptoms:**
- Calibration quality score < 60
- "Acceptable" or "Poor" quality messages

**Causes & Solutions:**

#### A. Insufficient Sample Collection
**Cause:** Not enough data collected during calibration
**Solutions:**
- Increase collection times in calibration script
- Ensure smooth, continuous movements
- Avoid jerky or incomplete rotations

#### B. Environmental Interference
**Cause:** Calibration performed in poor environment
**Solutions:**
- Move to outdoor location away from buildings
- Clear area of all metal objects
- Avoid areas with electrical noise
- Calibrate away from power lines and transformers

### 5. I2C Communication Issues

**Symptoms:**
- IMU not detected
- "Failed to initialize" errors
- Intermittent reading failures

**Causes & Solutions:**

#### A. Wiring Issues
**Cause:** Loose connections or incorrect wiring
**Solutions:**
- Check I2C connections (SDA, SCL, VCC, GND)
- Verify pull-up resistors on I2C bus
- Test with I2C scanner: `i2cdetect -y 1`
- Check for voltage level compatibility (3.3V vs 5V)

#### B. Address Conflicts
**Cause:** Multiple devices with same I2C address
**Solutions:**
- Verify IMU addresses (ISM330DHCX: 0x6B, MMC5983MA: 0x30)
- Check for address conflicts with other devices
- Use I2C multiplexer if needed

#### C. Power Supply Issues
**Cause:** Insufficient or noisy power
**Solutions:**
- Ensure stable 3.3V supply
- Add decoupling capacitors (10µF + 0.1µF)
- Check for power supply noise
- Verify ground connections

### 6. Temperature-Related Issues

**Symptoms:**
- Calibration degrades over time
- Performance changes with temperature
- IMU stops working when hot/cold

**Causes & Solutions:**

#### A. Thermal Drift
**Cause:** Sensor characteristics change with temperature
**Solutions:**
- Use temperature-compensated calibration
- Monitor IMU temperature in logs
- Recalibrate if temperature changes >10°C
- Consider active cooling/heating if extreme temperatures

### 7. Fusion Algorithm Issues

**Symptoms:**
- Heading oscillates or is unstable
- Complementary filter not converging
- Roll/pitch estimates are noisy

**Causes & Solutions:**

#### A. Filter Parameters
**Cause:** Incorrect complementary filter alpha values
**Solutions:**
- Check alpha values in IMU reader (currently 0.98 RP, 0.95 yaw)
- Tune based on your application requirements
- Consider using Kalman filter for better performance

#### B. Sensor Synchronization
**Cause:** Timing issues between sensors
**Solutions:**
- Ensure consistent sampling rates
- Check for dropped readings
- Verify timestamp synchronization

## Advanced Diagnostics

### Check Current Calibration

```bash
# Validate existing calibration
python calibrate_imu.py --validate-only

# View calibration data
cat imu_calibration.json
```

### Monitor Live IMU Data

```python
# In Python console
from pi_app.hardware.imu_reader import ImuReader
imu = ImuReader()
while True:
    data = imu.read()
    print(f"Heading: {data['heading_deg']:.1f}°, Roll: {data['roll_deg']:.1f}°, Pitch: {data['pitch_deg']:.1f}°, Yaw Rate: {data['gz_dps']:.2f}°/s")
    time.sleep(0.1)
```

### Check for Interference

1. **Magnetic Field Strength:**
   - Should be ~0.4-0.6 Gauss (40,000-60,000 nT)
   - Use magnetometer readings to check field strength
   - Compare with known Earth's magnetic field for your location

2. **Electrical Noise:**
   - Monitor yaw rate when stationary (should be <0.1°/s)
   - Check accelerometer for high-frequency noise
   - Verify power supply cleanliness

## Best Practices

### Calibration Environment
- Perform calibration outdoors, away from buildings
- Clear 10-foot radius of all metal objects
- Use non-magnetic surface (wood/plastic table)
- Avoid areas with electrical interference
- Maintain consistent temperature

### Mounting Guidelines
- Use plastic or nylon mounting hardware
- Keep IMU at least 6 inches from motors/batteries
- Route wires away from IMU
- Ensure rigid, vibration-free mounting
- Align axes with robot coordinate system

### Maintenance
- Recalibrate after any hardware changes
- Recalibrate if moving to different location
- Monitor calibration quality regularly
- Keep calibration data backed up

## Emergency Fixes

If IMU is completely unusable:

1. **Disable IMU steering temporarily:**
   ```python
   # In config.py
   config.imu_steering.enabled = False
   ```

2. **Use RC-only control** until IMU is fixed

3. **Check hardware connections** before assuming IMU failure

## Getting Help

If issues persist:

1. **Collect diagnostic data:**
   ```bash
   # Run calibration with verbose output
   python calibrate_imu.py --interactive > calibration_log.txt 2>&1

   # Monitor IMU readings
   python -c "
   from pi_app.hardware.imu_reader import ImuReader
   import time
   imu = ImuReader()
   for i in range(100):
       data = imu.read()
       print(f'{i:3d}: H={data[\"heading_deg\"]:6.1f} R={data[\"roll_deg\"]:5.1f} P={data[\"pitch_deg\"]:5.1f} Y={data[\"gz_dps\"]:6.2f}')
       time.sleep(0.1)
   " > imu_readings.txt
   ```

2. **Include in bug report:**
   - Calibration log
   - IMU readings log
   - Hardware setup description
   - Environment details
   - Robot configuration

## Prevention

- Always calibrate IMU in clean environment
- Use quality mounting hardware
- Monitor performance regularly
- Keep backup calibrations
- Document any hardware changes
