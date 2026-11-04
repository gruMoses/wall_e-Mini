# UPS HAT (B): mains-loss halt and RTC wake

This document describes the Pi's uninterruptible power supply (UPS) since
2026-11-03: a Waveshare UPS HAT (B), the watcher `bin/walle_ups.py`, and the
bootloader settings that make the halt and the wake work. It replaces the
UPSPlus HAT and `scripts/upsPlus_power_daemon.py`, which were deleted.

## 1. Purpose

The UPS is a buffer and a safe-shutdown device. It is not a ride-through
supply. When mains power to the HAT stops, the Pi halts cleanly. When mains
power returns, the Pi boots again. The pack floats near 50 percent so that
the cells last.

## 2. Hardware facts

Source: `UPS_HAT_B_Schematic.pdf` (Waveshare wiki), read 2026-11-03.

- Two 18650 cells in series (2S). HY2120 protection, HY2213 balancers.
- The HAT has no microcontroller and no charger IC. The 8.4 V barrel jack
  connects to the pack node through the protection MOSFETs. The pack floats
  at the voltage the jack supplies.
- The only sensor is an INA219 at I2C address 0x42, across a 0.1 Ω shunt in
  the battery leg. Positive current flows into the pack.
- An SY8286 buck makes the 5 V output. The output stays on until the HY2120
  low-voltage cutoff. The HAT cannot switch its output off, and it cannot
  signal "mains is back".
- The Pi 5 wakes only from its power button, from the RTC alarm, or when
  5 V is removed and applied again. GPIO3 does not wake a Pi 5.

Bench readings (2026-11-03, 8.4 V brick, Pi idle): pack 8.30 to 8.38 V,
battery current +4 to +15 mA, noise ±4 mA. The pack never discharged while
the brick was connected. A running Pi draws at least 300 mA from the pack
once mains is gone.

## 3. Charge limit

The charge limit is the buck setpoint. There is no code for it.

- Robot: set the 8.4 V buck to **7.6 V** (3.80 V per cell, about 50 percent).
  Use a buck with a current limit (CC/CV), 2 A or more. Kevin chose 7.6 V on
  2026-11-03. Do not go below 7.2 V.
- The balancers only work near 4.2 V per cell. Every few months, set the
  buck to 8.4 V for a few hours, let the current taper, then return to 7.6 V.
- Tell the watcher the setpoint: `--supply-v` in `bin/walle-ups.service` and
  `bin/walle-ups-wakecheck.service` (8.4 on the bench brick, 7.6 with the
  robot buck). Then run the install script (section 6).
- The day the setpoint goes down, the pack sits above the buck and carries
  the Pi. The watcher reads this as `above_supply`, not as a loss, until the
  pack sags to the setpoint. Expect a few hours.

## 4. How the watcher works

`bin/walle_ups.py` runs as root. It has two modes.

### 4.1 `run` mode (`walle-ups.service`)

Every 0.5 s it reads the INA219 and classifies the sample:

| State | Rule | Effect on the loss timer |
|---|---|---|
| `present` | current ≥ −40 mA | reset to 0 |
| `absent` | current ≤ −250 mA and pack ≤ setpoint + 0.15 V | add 0.5 s |
| `above_supply` | current ≤ −250 mA and pack above setpoint + 0.15 V | hold |
| `unknown` | the middle band, or an I2C error | hold |

The timer counts absent samples, not wall-clock time. After 5 s of absent
samples the watcher:

1. writes `/var/lib/walle-ups/loss.json` and an event line;
2. sends SIGKILL to `wall-e.service` and stops it (its camera teardown can
   take 90 s, and the RTC alarm must not fire during the shutdown);
3. arms the RTC alarm, `+90 s`, and checks the read-back;
4. runs `sync` and `systemctl poweroff`.

### 4.2 `boot-check` mode (`walle-ups-wakecheck.service`)

This oneshot runs on every boot before `basic.target`. It reads the sensor
for 2.5 s.

- Any `present` sample: the boot continues.
- At least 75 percent `absent` samples and no `present` sample: it arms the
  alarm again (`+60 s`) and powers off. The Pi was on for about 16 s.
- Anything else (no sensor, I2C errors, middle band, `above_supply`): the
  boot continues. The check fails open on purpose. A Pi that powers itself
  off because a sensor is missing cannot be repaired from the bench.

### 4.3 Bootloader settings

`POWER_OFF_ON_HALT=1` and `WAKE_ON_GPIO=0` in the bootloader EEPROM (set
2026-11-03 with `rpi-eeprom-config --apply`). A halt is then a PMIC standby
of about 3 mA, and the RTC alarm wakes the Pi. The HAT keeps 5 V on the Pi,
so the RTC runs without a coin cell.

Bench 2026-11-03: `systemctl poweroff` to "Journal stopped" 8 s; alarm `+90`
to ssh 108 s after the poweroff command (about 18 s from the alarm to ssh).

## 5. Energy while mains is off

Each wake costs about 16 s of boot. At 60 s polls a 50 percent pack lasts
roughly one day. Then the HY2120 cutoff removes the 5 V, and the Pi boots on
power-up when mains returns. For long storage, switch the HAT off.

## 6. Install and update

```bash
sudo bash bin/install_walle_ups.sh
```

The script copies both unit files, enables them, restarts the watcher and
prints one reading. Auto-deploy pulls the code but does not restart the
watcher: run the script after each change to `bin/walle_ups.py` or to a
unit file. The I2C bus guard restarts `walle-ups.service` after a bus
recovery; after a change to `bin/i2c_bus_guard.py` run
`sudo bash bin/install_i2c_bus_guard.sh`.

## 7. Status and overrides

- `/tmp/ups_status.json`: `ts` is the time of the last good INA219 read. The
  I2C bus guard uses its age as the "reader can see the bus" witness. `ts_loop`
  shows that the process is alive. `input`, `batt_v`, `batt_ma`, `soc_pct`,
  `state`, `mode`, `absent_s`, `last_loss` feed the `/debug` page.
- `/var/lib/walle-ups/events.jsonl`: every loss, alarm and boot check.
- `/var/lib/walle-ups/nohalt`: when this file exists, neither mode halts the
  Pi. The watcher logs a warning every 5 minutes and the dashboard shows
  NOHALT. Remove the file when the bench work is done.
- `walle_ups.py once`: one reading, as JSON.
- `walle_ups.py run --detect-only`: the watcher logs what it would do and
  does nothing.

## 8. Bench procedure

1. Confirm the setpoint in the unit files matches the supply.
2. `journalctl -u walle-ups -f` in one terminal.
3. Pull the mains plug. Within 5 s the journal shows `MAINS LOST`, then the
   kill, the alarm and the poweroff. The Pi's LEDs go dark; the HAT's stay on.
4. After 90 s the Pi wakes. The wake check powers it off again within about
   16 s. Repeat every 60 s.
5. Plug mains in. The next wake continues the boot. `events.jsonl` shows the
   whole sequence with times and currents.
6. `sudo reboot` must come back without an alarm.

## 9. Open items

- Measure the battery current under the worst robot load (camera, boot) with
  the robot buck connected. It must stay above −40 mA. If the buck folds back
  and the pack shares the load, raise the buck's current limit; the watcher
  cannot tell that case from a loss.
- Run the pack to the HY2120 cutoff once and confirm that the 5 V drops to
  zero and the Pi boots when mains returns.
- Kevin plans a custom HAT that can switch its output off after the halt and
  power on with mains, which removes the polling.

## Appendix: Technical Names

UPS HAT (B), Waveshare, UPSPlus, INA219, HY2120, HY2213, SY8286, RTC, PMIC,
EEPROM, `POWER_OFF_ON_HALT`, `WAKE_ON_GPIO`, `rpi-eeprom-config`, GPIO3,
`walle-ups.service`, `walle-ups-wakecheck.service`, `bin/walle_ups.py`,
`bin/install_walle_ups.sh`, `bin/i2c_bus_guard.py`, `wall-e.service`,
`basic.target`, `systemctl poweroff`, SIGKILL, `/tmp/ups_status.json`,
`/var/lib/walle-ups`, `events.jsonl`, `loss.json`, `nohalt`, `above_supply`,
`--supply-v`, `--detect-only`, CC/CV, buck, 18650, 2S, I2C, ssh, `journalctl`,
Kevin.
