# WALL-E Migration Guide: v1 to wall_e-Mini

> Historical migration record. This document captures the original transition
> from v1 and may not reflect current runtime behavior. For current operations,
> refer to `README.md` and docs under `docs/` (especially `services.md`,
> `hardware_mapping.md`, `testing.md`).

This document outlines the migration from the original monolithic WALL-E control system to the new modular wall_e-Mini system.

## Overview of Changes

### Architecture Changes
- **Old System**: Single monolithic Python file (`wall-e v1.py`) ~2800 lines
- **New System**: Modular package structure with separate modules for:
  - Hardware interfaces (Arduino, VESC, IMU)
  - Control logic (PID, steering compensation)
  - Input/Output (Bluetooth, RC processing)
  - CLI tools for calibration and monitoring

### Key Improvements
1. **Modular Design**: Easier to maintain, test, and extend
2. **IMU Steering**: Advanced PID-based heading control (previously commented out)
3. **Better Bluetooth**: Simplified unauthenticated protocol
4. **Configuration Management**: Structured config with dataclasses
5. **CLI Tools**: Separate tools for calibration, monitoring, and testing

## Migration Steps Completed

### ✅ Backup Created
- Full backup saved to: `/home/pi/wall_e_backup_20251009_114045/`
- Includes all code, configs, logs, and data files

### ✅ Dependencies Installed
- All Python packages from `requirements.txt` installed
- Compatible versions of pyserial, numpy, SparkFun IMU libraries

### ✅ Repository Cloned
- New wall_e-Mini code cloned to `/home/pi/wall_e-Mini/`

### ✅ Services Updated
- `wall-e.service` updated to use new `run_main.sh` script
- New `wall-e-spp.service` created for Bluetooth SPP server
- CAN setup service preserved

### ✅ Hardware Verification
- CAN interface: ✅ Available (`can0`)
- Serial devices: ✅ Available (`/dev/ttyUSB0`)
- I2C bus: ✅ Available (bus 1)
- Bluetooth: ✅ Working (manual test successful)

## Configuration Mapping

### RC Signal Parameters
- MIN_PULSE_WIDTH: 990µs → Arduino code bounds: 800-2200µs
- CENTER_PULSE_WIDTH: 1500µs → Center: 1500µs
- DEADBAND_SIZE: 25µs → Deadband: 25µs

### Bluetooth Settings
- BT_FRESH_TIMEOUT: 0.6s (unchanged)
- SPP UUID: 00001101-0000-1000-8000-00805F9B34FB (unchanged)

### IMU Steering
- **Old**: Commented out, not active
- **New**: Enabled by default with tuned PID gains
  - Kp: 1.7, Ki: 0.4, Kd: 0.08
  - Can be disabled in `config.py` if needed

## Arduino Firmware Compatibility

### RC Reading
- **Compatible**: Both systems output 4 CSV integers over serial
- **Differences**:
  - Old: ~20Hz output, debug lines always enabled
  - New: ~50Hz output, debug optional via DEBUG define

### Motor Control
- **Old**: No motor control capability
- **New**: Arduino can receive motor commands via `M,<left>,<right>` protocol
- **Migration**: Update Arduino firmware to new version for full functionality

## Bluetooth Protocol Changes

### Authentication
- **Old**: HMAC-SHA256 authenticated V2 protocol
- **New**: Unauthenticated V2 protocol (HMAC ignored for compatibility)

### Client Requirements
- Sequence numbering still required
- HMAC computation no longer validated on server
- Android app should work with both protocols

## Rollback Procedure

If issues occur with the new system:

1. Stop new services:
   ```bash
   sudo systemctl stop wall-e.service wall-e-spp.service
   ```

2. Restore old service:
   ```bash
   sudo cp /etc/systemd/system/wall-e.service.backup /etc/systemd/system/wall-e.service
   sudo systemctl daemon-reload
   sudo systemctl start wall-e.service
   ```

3. Restore old files:
   ```bash
   cd /home/pi
   cp wall_e_backup_*/wall-e* ./
   cp wall_e_backup_*/wallectl ./
   ```

## Next Steps

1. **Update Arduino Firmware**: Flash `arduino_rc_reader/arduino_rc_reader.ino` for motor control capability
2. **Test IMU Calibration**: Run calibration tools if IMU hardware is present
3. **Test Full System**: Start services and test RC control, Bluetooth, and IMU steering
4. **Tune PID Gains**: Adjust steering PID values for your specific robot
5. **Clean Up**: Remove old files once new system is verified working

## Troubleshooting

### Service Won't Start
- Check logs: `journalctl -u wall-e.service -f`
- Verify Python path: `PYTHONPATH=/home/pi/wall_e-Mini python3 -c "import pi_app"`

### Bluetooth Issues
- Manual test: `sudo python3 -m pi_app.cli.spp_server`
- Check Bluetooth status: `systemctl status bluetooth`

### Hardware Detection
- CAN: `ip link show can0`
- Serial: `ls /dev/ttyUSB* /dev/ttyACM*`
- I2C: `i2cdetect -y 1`

## Files Changed

### Updated
- `/etc/systemd/system/wall-e.service`
- `/etc/systemd/system/wall-e-spp.service` (new)

### Preserved
- `/etc/systemd/system/can-setup.service`
- `/etc/systemd/system/wifi-reconnector.service`

### New Files
- `/home/pi/wall_e-Mini/` (entire directory)
- `/home/pi/wall_e-Mini/MIGRATION_GUIDE.md` (this file)

### Backed Up
- `/home/pi/wall_e_backup_*/` (all original files)
