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
- The only sensor is an INA219 at I2C address 0x42, across a shunt in the
  battery leg. The schematic labels the shunt 0.1R. The readings say 10 mΩ:
  with the brick unplugged the shunt showed 5 to 9 mV while the Pi's own
  PMIC rails summed to more than 2 W, which is 0.5 to 0.9 A, not 50 to
  90 mA. The watcher uses 10 mΩ. Positive current flows into the pack.
- An SY8286 buck makes the 5 V output. The output stays on until the HY2120
  low-voltage cutoff. The HAT cannot switch its output off, and it cannot
  signal "mains is back".
- The Pi 5 wakes only from its power button, from the RTC alarm, or when
  5 V is removed and applied again. GPIO3 does not wake a Pi 5.

Bench readings (2026-11-03, 8.4 V brick, Pi idle): pack 8.30 to 8.38 V,
battery current +40 to +150 mA, noise ±40 mA. The pack never discharged
while the brick was connected. With the brick unplugged the pack carried the
Pi at −520 to −910 mA and its voltage fell from 8.35 V to 7.4 to 7.6 V
within 90 s; it returned to 8.35 V at once when the brick came back, at
+130 to +210 mA. That swing means the cells have about 1 Ω of internal
resistance (a healthy 18650 has 0.03 to 0.08 Ω). The pack's usable
ride-through for a running Pi is minutes, not hours. The watcher does not
need more than 15 s.

## 3. Charge limit

The charge limit is the buck setpoint. There is no code for it.

- Robot: the supply is set to **7.5 V** (3.75 V per cell, about 45 percent;
  Kevin, 2026-11-04, after first choosing 7.6 V). Use a supply with a
  current limit (CC/CV), 2 A or more. Do not go below 7.2 V.
- The balancers only work near 4.2 V per cell. Every few months, set the
  buck to 8.4 V for a few hours, let the current taper, then return to 7.6 V.
- Tell the watcher the setpoint: `--supply-v` in `bin/walle-ups.service` and
  `bin/walle-ups-wakecheck.service` (7.5 with the robot supply, 8.4 on the
  bench brick). Then run the install script (section 6). The loss rules
  depend on it. The watcher logs `SUPPLY SETPOINT MISMATCH` when the pack
  floats more than 0.2 V away from it for a minute.
- WARNING: when you lower the supply (an 8.4 V top-up back to 7.5 V), change
  `--supply-v` first. The pack then sits above the supply and carries the
  Pi while the supply idles. The watcher holds (`above_supply`, then
  `discharge`) until the pack sags to the setpoint. Expect hours.

## 4. How the watcher works

`bin/walle_ups.py` runs as root. It has two modes.

### 4.1 `run` mode (`walle-ups.service`)

The HAT does not sense its input jack. The watcher infers mains from the
battery current I and the pack voltage V against the setpoint S. The hard
case is a pack above the supply: the supply is present but idle, the
cells carry the Pi, and the readings look like a lost supply. The rules
of 2026-11-04 (Grok design review and two diff reviews, plus a Claude
review) handle it.

Every 0.5 s it reads the INA219 and classifies the sample:

| State | Rule | Effect |
|---|---|---|
| `present` | I ≥ −80 mA | resets the loss count |
| `unknown` | −250 < I < −80 mA, an I2C error, or an implausible reading (V outside 4 to 9.5 V, over 5 A) | holds |
| `above_supply` | I ≤ −250 mA and V > S + 0.15 V | holds: the pack is fuller than the float |
| `absent` | I ≤ −250 mA and V < S − 0.30 V | counts: a present supply would hold the pack at S |
| `discharge` | I ≤ −250 mA, V between those two | holds: the supply may be present and below the pack |

- 5 s of counted `absent` samples halts. The count resets on `present`, and
  after 10 s without a counted sample.
- The hard line has hysteresis. A count starts only below S − 0.30 V
  (7.20 V). While it runs, a discharging sample below S − 0.20 V (7.30 V)
  also counts. A pack that hovers at the line then halts in 5 s instead
  of restarting the count (6 min of that on 2026-11-04).
- Low battery halts on any sample that is not `present`: 7.0 V for 10 s,
  or 6.4 V on one sample. This is the only detector while the pack is
  above the supply. A held discharge then ends in a clean halt, not the
  HY2120 cutoff.
- The edge rule (`--edge-s`) is off. With it on, a `discharge` sample
  within 10 s of a `present` one counts as `absent`. That catches a pull
  from the float in 5 s with healthy cells. It is off because healthy,
  low-resistance cells turn a small supply droop under a load step into
  the same readings (40 mV through 0.07 Ω is 0.57 A out of the pack).
  With it off, a pull is caught when the pack sags below S − 0.30 V: at
  once on the present cells (about 1 Ω), after part of the buffer on
  healthy cells. Measure the supply's droop under load before you turn it
  on.
- Timers run on the boot clock, not the wall clock. The RTC resets to 0
  when 5 V drops, and NTP then steps the wall clock forward.

The halt:

1. writes `/var/lib/walle-ups/loss.json`, an event line, and `chain.json`
   (the loss chain, with the halt kind: `hard`, `edge`, `floor`,
   `critical` or `simulated`);
2. sends SIGKILL to `wall-e.service` and stops it (its camera teardown can
   take 90 s, and the RTC alarm must not fire during the shutdown);
3. arms the RTC alarm and checks the read-back: `+90 s`, `+600 s` after a
   low-battery halt, at least `+300 s` if `wall-e` is still stopping;
4. runs `sync` and `systemctl poweroff`. If the Pi is still up 30 s later
   it re-arms and retries; after 60 s it forces the poweroff.

### 4.2 `boot-check` mode (`walle-ups-wakecheck.service`)

This oneshot runs on every boot before `basic.target`. It reads the sensor
for 2.5 s. It has no history, so it cannot see an edge.

- Any `present` sample: the boot continues and the chain closes.
- Fewer than 4 valid samples, or more than 25 percent bad ones: the boot
  continues (fail open).
- At least 75 percent of the valid samples:
  - at or below 7.0 V, whatever the current: power off, alarm `+600 s`
    (a low pack polls less), never capped. A present supply holds the
    pack at S, so this cannot lock a live supply out;
  - `absent`: power off, alarm `+60 s`;
  - `discharge` at −400 mA or more, and not within 0.03 V of the setpoint,
    inside any chain except a simulated one: power off, `+60 s` (under the
    light early-boot load the pack rebounds into the band, and the outage is
    still going). A supply that came back below the pack pins the voltage at
    its setpoint and shares the load, so it does not count. On 2026-11-04 the
    robot buck held 7.496 to 7.512 V at −270 to −305 mA on 23 wakes, while a
    real outage read 7.30 to 7.35 V at −567 to −600 mA. On doubt the boot
    continues, and run mode's hard line and floor still halt;
  - `above_supply`: the boot continues and the chain closes.
- Anything else: the boot continues. A Pi that powers itself off because
  a sensor is missing cannot be repaired from the bench.
- The cap: after 24 power-offs in one chain (about 30 min, both modes
  count) the boot continues. Run mode then starts with a 10 min grace:
  mains-loss halts off, low-battery halts on. A wrong `--supply-v`, or a
  supply below the pack plugged in during an outage, cannot keep the Pi
  off for good. A pack below the floor at boot is never capped.
- The chain counts power-offs, not time, because the clock is not
  trustworthy after a 5 V drop.

### 4.3 Bootloader settings

`POWER_OFF_ON_HALT=1` and `WAKE_ON_GPIO=0` in the bootloader EEPROM (set
2026-11-03 with `rpi-eeprom-config --apply`). A halt is then a PMIC standby
of about 3 mA, and the RTC alarm wakes the Pi. The HAT keeps 5 V on the Pi,
so the RTC runs without a coin cell.

Bench 2026-11-03: `systemctl poweroff` to "Journal stopped" 8 s; alarm `+90`
to ssh 108 s after the poweroff command (about 18 s from the alarm to ssh).

Real unplug, 2026-11-04 (8.4 V brick, 10 mΩ scale):

| Time | Event |
|---|---|
| 09:03:49 | Brick pulled (first absent sample 5 s before the loss) |
| 09:03:54.6 | `MAINS LOST`, −646 mA at 7.70 V; wall-e killed; alarm +90 s |
| 09:03:59.0 | Journal stopped: the Pi is dark 9.5 s after the pull |
| 09:05:24 | Alarm wake; at 09:05:39.6 the wake check reads 11 of 11 absent (−551 mA), arms +60 s, powers off |
| 09:06:39 | Alarm wake; at 09:06:54.7 the same result (−547 mA) |
| 09:07:54 | Alarm wake after the replug; at 09:08:09.7 the wake check reads 11 of 11 present (+29 mA) and the boot continues |
| 09:08:24 | ssh answers (uptime 20.7 s) |

An earlier pull the same morning (08:53) halted in the same way. That time
Kevin pressed the Pi's power button after the replug, so the Pi booted
before its alarm. The button is not necessary: the next alarm wake sees
mains and continues the boot.

Second pull, 2026-11-04 10:47 (8.4 V brick, old rules): `MAINS LOST` 5 s
after the pull at −510 mA, then six correct poll wakes, about 75 s apart.
Each green flash was a 16 s wake, not a hung shutdown. About 10:56 the
Pi's 5 V dropped for a moment. The RTC reset to 0, which wiped the
alarm, and the power returning booted the Pi. On the 7.5 V supply the
old boot check then powered it off four times, because the cells (8.27 V
at rest, about 1.1 Ω) carried the Pi above the idle supply at 7.59 to
7.70 V and −0.46 to −0.56 A. That is the reason for the rules in 4.1 and
4.2.

Real pull with the new rules, 2026-11-04 (7.5 V supply, cells about
1.1 Ω, the pack a little above the supply before the pull):

| Time | Event |
|---|---|
| 13:23:10 | Supply pulled. The cells carry the Pi: 7.44 V, −600 mA, `discharge` (hold) |
| 13:33-13:40 | The pack hovers at the 7.20 V line; the loss count starts and resets |
| 13:40:06 | `MAINS LOST (hard)` at 7.18 V, −720 mA; alarm +90 s; clean halt |
| 13:41:51-14:03:07 | 18 alarm wakes: the wake check reads 7.30-7.35 V at −0.57 A (`discharge`) inside the hard chain and powers off, +60 s each |
| 14:04:22 | After the replug: the wake check reads 7.67 V, −166 mA (the supply shares the load) and continues the boot |
| 14:04:26 | ssh answers |

The band rule inside a chain did its job: under the light early-boot load
the pack read above the 7.20 V line on every wake.

The robot buck, 2026-11-04 14:21-14:51: a supply swap that took over 5 s
halted the Pi and opened a hard chain. The buck then held the pack at
7.50 V, but the cells rested near 7.8 V after floating on the wall wart
(which sits near 7.67 V, not 7.5 V) and still pushed −270 to −305 mA. The
old band rule read that as the outage going on, and the Pi polled until
the 24-wake cap (30 min). The current and pinned-voltage conditions in 4.2
now let that boot. Swap supplies with the Pi shut down (a button shutdown
opens no loss chain).

Confirmed 2026-11-04 15:18-15:20 with a069476: a swap to the robot buck
halted the Pi (7.18 V, −652 mA, hard). The first alarm wake read 7.50 V at
−262 mA (no strong band samples) and the boot continued. The watcher then
held, flickering between `unknown` and `discharge` at 7.47 to 7.50 V,
−176 to −303 mA, while the cells rode down to the buck.

Pull on the robot buck, 2026-11-04 (cells about 0.25 V above the buck):

| Time | Event |
|---|---|
| 16:04:22 | Buck pulled |
| 16:04:27 | `MAINS LOST (hard)` at 7.07 V, −589 mA; alarm +90 s; clean halt |
| 16:06:12, 16:07:27 | Alarm wakes: 7.10 V at −0.6 A, `absent`; power off, +60 s |
| 16:08:42 | First wake after the replug: 7.50 V, −164 mA; the boot continues |
| 16:09:00 | ssh answers |

## 5. Energy while mains is off

Each wake costs about 16 s of boot at 0.5 to 0.9 A. One poll cycle is about
75 s, so the Pi is on about a fifth of the time. With the cells measured on
2026-11-03 (about 1 Ω, see section 2) the pack reaches the HY2120 cutoff
within the hour. Then the
HAT removes the 5 V, the Pi is fully off, and it boots on power-up when mains
returns. Healthy cells would last roughly a day. For long storage, switch the
HAT off.

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
- `/var/lib/walle-ups/events.jsonl`: every loss, alarm, boot check and cap.
- `/var/lib/walle-ups/chain.json`: the open loss chain (kind, power-off
  count). It closes on `present` or `above_supply`.
- `/run/walle-ups/grace`: the cap grace deadline on the boot clock. It is
  gone after a reboot.
- `/var/lib/walle-ups/nohalt`: when this file exists, neither mode halts the
  Pi. The watcher logs a warning every 5 minutes and the dashboard shows
  NOHALT. Remove the file when the bench work is done.
- `walle_ups.py once`: one reading, its class and the open chain, as JSON.
- `walle_ups.py run --detect-only`: the watcher logs what it would do and
  does nothing.
- `sudo walle_ups.py simulate-loss --yes`: runs the real halt sequence once
  (kill `wall-e`, arm the alarm, sync, poweroff) with mains connected. The
  wake check then sees `present` and the Pi boots back after the first
  alarm. This tests the whole chain without pulling a plug.

## 8. Bench procedure

1. Confirm the setpoint in the unit files matches the supply.
2. `journalctl -u walle-ups -f` in one terminal.
3. Pull the mains plug. With the present cells (about 1 Ω) the journal
   shows `MAINS LOST (hard)` within 5 s, then the kill, the alarm and the
   poweroff; the Pi's LED goes red (standby). With healthy cells and the
   edge rule off, the halt waits until the pack sags 0.3 V below the
   setpoint (section 4.1).
4. After 90 s the Pi wakes. The wake check powers it off again within about
   16 s. This repeats about every 75 s.
5. Plug mains in. Do not press the Pi's power button. The next wake continues
   the boot, at most about 75 s after the replug, and ssh answers about 15 s
   later. `events.jsonl` shows the whole sequence with times and currents.
   Each boot has its own journal (`journalctl --list-boots`).
6. `sudo reboot` must come back without an alarm.

## 9. Open items

- Done 2026-11-04 13:23-14:04: a real pull on the 7.5 V supply with the new
  rules (table in section 4.3).
- Done 2026-11-04: the loss count dithered for about 6 min while the pack
  hovered at the 7.20 V line (13:33-13:40). The hard line now has 0.10 V
  of hysteresis (section 4.1).
- Done 2026-11-04 13:13: the hold check. The Pi booted on the HAT with the
  7.5 V supply and cells resting at 8.32 V. The cells carried it at 7.66
  to 7.71 V and −0.37 to −0.64 A for 3 min, read as `above_supply`, with
  no loss count. The `nohalt` flag was then removed; the watcher is armed.
- Never power the Pi from USB-C and the HAT at the same time: the HAT has
  no back-feed protection on its 5 V output. Swap sources with the Pi
  shut down.
- Before turning on `--edge-s` with new cells, measure the supply's droop
  from idle to full Pi load. It must stay well under 0.30 V.
- Done 2026-11-04: the real unplug test at the 10 mΩ scale (section 4.3).
  The first one, 2026-11-03 20:51, ran with the 0.1R scale, read −52 mA, and
  held in the middle band on purpose.
- The cells: about 1 Ω internal resistance. Decide whether to replace them.
  With them, the pack runs down to the cutoff within the hour at 60 s polls.
- Measure the battery current under the worst robot load (camera, boot) with
  the robot buck connected. It must stay above −80 mA. If the buck folds back
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
`--supply-v`, `--edge-s`, `--detect-only`, `chain.json`, `/run/walle-ups/grace`,
`above_supply`, `discharge`, CLOCK_BOOTTIME, NTP, CC/CV, buck, 18650, 2S, I2C, ssh, `journalctl`,
Kevin.
