# CAN and VESC Notes

## Driver Selection

Main runtime (`pi_app/app/main.py`) selects motor backend at startup:

- If `VescCanDriver.detect()` sees `can0`, it uses VESC over CAN.
- Otherwise it falls back to Arduino Model X serial motor driver.

Current default IDs in main:

- left VESC CAN ID: `2`
- right VESC CAN ID: `1`

## Byte to eRPM Mapping

VESC driver converts command bytes to RPM-like command units:

- Input byte range: `0..254`
- Neutral: `126` => `0`
- Above neutral -> positive command
- Below neutral -> negative command

`config.py` default:

- `vesc.max_erpm = 15000`

## CAN Frame Format

`pi_app/hardware/vesc.py` sends:

- arbitration id: `0x300 + can_id`
- extended CAN frame (`is_extended_id=True`)
- payload: signed 32-bit big-endian integer (`struct.pack(">i", rpm)`)

## Bring-up Checklist

```bash
ip link show can0
ls /sys/class/net/can0
```

If `can0` exists but no motion occurs, verify:

- VESC IDs match runtime (`left_id`, `right_id`)
- CAN bitrate and transceiver wiring
- no bus-off errors (`dmesg`, `ip -details link show can0`)
