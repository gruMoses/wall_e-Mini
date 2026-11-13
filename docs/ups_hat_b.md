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
  buck to 8.4 V for a few hours, let the current taper, then return to 7.5 V.
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
3. arms the RTC alarm and checks the read-back: `+90 s` after every halt
   (`1aa9621`), at least `+300 s` if `wall-e` is still stopping. It also
   writes the seconds and the uptime to `/run/walle-ups/arm`. The shutdown
   hook `bin/walle-ups-shutdown-alarm` (installed in
   `/usr/lib/systemd/system-shutdown/`) arms the alarm again in the last
   second before power-off, so a slow shutdown cannot use the alarm up. It
   ignores a file more than 600 s old, and a manual poweroff or the ch5
   e-stop writes no file, so neither arms a wake;
4. runs `sync` and `systemctl poweroff`. If the Pi is still up 30 s later
   it re-arms and retries; after 60 s it forces the poweroff.

### 4.2 `boot-check` mode (`walle-ups-wakecheck.service`)

This oneshot runs on every boot before `basic.target`. It reads the sensor
for 2.5 s. It has no history, so it cannot see an edge.

- Any `present` sample: the boot continues and the chain closes.
- Fewer than 4 valid samples, or more than 25 percent bad ones: the boot
  continues (fail open).
- At least 75 percent of the valid samples:
  - at or below 7.0 V, whatever the current: power off, never capped,
    alarm `+60 s`, or `+600 s` at or below 6.4 V (a nearly empty pack polls
    less). With 1 Ω cells a half-full pack reads about 6.9 V under the boot
    load. A present supply holds the pack at S, so this cannot lock a live
    supply out;
  - `absent`: power off, alarm `+60 s`;
  - `discharge` at −400 mA or more, and below the setpoint by more than 0.03 V (one-sided since 2026-11-06),
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

The script copies both unit files, installs the shutdown hook and two
stop-timeout limits (`/etc/systemd/system.conf.d/10-walle-fast-stop.conf`,
30 s for system units; `/etc/systemd/user.conf.d/10-walle-fast-stop.conf`,
10 s for desktop-session units), enables the units, restarts the watcher
and prints one reading.

Since 2026-11-04 the Pi boots to the console with autologin
(`raspi-config nonint do_boot_behaviour B2`), not to the desktop. On that
day a hung desktop service, `xdg-document-portal`, held a mains-loss
shutdown for 91 s against the 90 s alarm. The autologin keeps the `pi`
session, so Pi Connect's remote shell still works; its screen sharing
does not. To undo: `sudo raspi-config nonint do_boot_behaviour B4`. Auto-deploy pulls the code but does not restart the
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

- Done 2026-11-05 09:47: motor load on the robot stand, 45 s at up to full
  speed with hard stops (15.5 A drive draw, -12 A regen on one VESC). The
  watcher stayed `monitoring`, the input read `present` or `unknown` (never
  `discharge` or `absent`), the pack read 7.48 V to 7.53 V and the battery
  current -105 mA to +18 mA. EXT5V read 5.17 V to 5.19 V, `throttled=0x0`.
  Motor load does not cause a false mains loss on the robot buck.

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
- Done 2026-11-04 17:07-17:22, in the robot on the test stand, robot buck
  at 7.5 V, wall-e running with the OAK: idle 7.54 V at about −65 mA; all
  four CPU cores loaded 7.48 to 7.49 V at −93 to −124 mA (buck droop about
  60 mV, far under the 0.30 V hard margin; the worst current is half the
  −250 mA discharge line); back to 7.55 V after. No USB over-current trips
  that boot (167 before), `throttled=0x0`, EXT5V 5.20 V. The `/debug` UPS
  panel renders live. The cells were still settling onto the buck, so the
  mains pill flickers between present and unknown; both hold.
- Found in passing: the OAK enumerates at USB 2 speed (480M) under the
  hub's USB 2 side; the hub's USB 3 side is empty. Nothing in the config
  limits it. Check the OAK-to-hub cable (it must be a USB 3 cable).
  2026-11-05 14:43: still 480M with the cables Kevin confirmed as USB 3
  (blue plugs). The kernel journal (kept since 2026-07-10) has no
  SuperSpeed link for the OAK (03e7) at any time; the only SuperSpeed
  devices are the hub and a WD drive in July. The hub's USB 3 side links
  at 5000M through the cut cable, so the fault is on the OAK side: flip
  the USB-C plug at the camera, then try another USB-C cable. CAUTION:
  USB 3 radiates noise near 2.4 GHz (the RC receiver, Wi-Fi and the BMS
  link). After a change to 5000M, check the RC link and re-measure
  `oak.det_latency_s` (the 15 fps cap was tuned at USB 2).
  FIXED 2026-11-05 15:05: Kevin turned the USB-C plug over at the camera
  (the cable that came with it). The booted OAK linked at 5000M on the
  hub's USB 3 side (`usb 2-1.1: new SuperSpeed`), the first SuperSpeed
  link in the journal. Idle `oak.det_latency_s` p50 0.196 s, p90 0.211 s,
  depth 15 fps (the same as at USB 2). The A-to-C cable wires one side of
  the C plug only, so the plug orientation is load-bearing: mark it. Not
  checked yet: the RC link at range (USB 3 noise near 2.4 GHz); at close
  range on the stand at 15:15 the channels were clean. Grok (session
  `01a1ad8f`) advised forcing USB 2 with `maxUsbSpeed=HIGH` because the
  pipeline does not need the bandwidth. Kevin's decision, 2026-11-05: keep
  USB 3, it may be needed for more of the camera's capabilities. If the RC
  link drops at range, force USB 2 in the config first.
  REVERSED 2026-11-06: the USB 3 link jams the RTK GPS. The GPS antenna is
  within 30 cm of the hub. The GPS went from 10 satellites to 0 at
  15:05:24 on 2026-11-05, the second the OAK first linked at 5000M, and
  stayed at 0 for 25 h. Outdoors on 2026-11-06 (Kevin also power-cycled the
  RTK board once): OAK unplugged, 16 satellites and RTK float; OAK back at
  USB 3, 6-8 satellites and no RTK within 10 s; after a reboot with the OAK
  streaming at USB 3, 10-11 satellites while tracking and 0 for 4 min from
  a fresh start, also with the shield on the cut Pi-to-hub extension
  repaired with foil. OAK streaming at USB 2: 32-34 satellites, RTK float.
  The antenna cannot be moved. Decision (Kevin): USB 2 for now.
  `OakDetectionConfig.max_usb_speed = "HIGH"` caps the link in software, so
  the plug orientation no longer decides; the startup line shows
  `USB HIGH (cap HIGH)`. Before a change to `"SUPER"`, shield the hub and
  the OAK cable, put a metal plate under the antenna, and re-test the
  satellite count from a fresh GPS start with the camera streaming.
  REVIEW 2026-11-10 (Kevin asked if that list is still correct; Grok
  session `01a1c6e2-4b2b-7d51-b0a9-071c8070b968` agrees with the order
  below). Measured that day with the OAK at USB 2: the hub's USB 3 side
  (`2-1`, 05e3:0626) is in runtime suspend, and it was active for 0.2 s
  only since the boot. Thus the good GPS result at USB 2 does not clear
  one part and does not blame one part: the Pi-to-hub link, the hub and
  the OAK cable are all quiet together. Recommended order, not decided:
  1. Find out if USB 3 has a task. The only known task is VIO at the same
     time as depth. Bench VIO at 320x200 at USB 2 first. If depth stays
     at 15 fps, USB 3 is not necessary now.
  2. Replace the cut Pi-to-hub extension before each USB 3 test. Its
     shield was opened, and a foil patch is not a shield. Use a cable that
     is not cut and put tape on the 5 V contact of the plug, or close the
     braid fully (copper tape, soldered around the cable).
  3. Find the part that radiates before you shield. Test A: the Pi-to-hub
     link awake with the OAK at USB 2. CAUTION: link power management
     (`usb3_hardware_lpm_u1` and `_u2`) is enabled, so `power/control=on`
     alone possibly does not keep the link active; this is not verified.
     Test B: the OAK directly on a USB 3 port of the Pi, without the hub.
  4. Repair that part: a short cable with a full shield, a hub with a
     metal body or no hub in the USB 3 path. A clip-on ferrite near the
     antenna is cheap, but its effect above 1 GHz is small.
  5. The metal plate under the antenna is good for RTK at USB 2 also. It
     stops noise from below only, not from the side.
  6. Foil around a cable has almost no effect if it is not bonded to the
     two plug shells. The foil repair of 2026-11-06 changed nothing.
  7. Pass test: one fresh GPS start and the RTK fix behaviour during a
     session. The satellite count alone is not sufficient, because an RTK
     fix is already rare at USB 2.
  The robot must be outdoors for each test (indoors on 2026-11-10 at 13:15
  it had 13 satellites and no corrections).
  KEVIN, 2026-11-10 (after the review): item 1 is withdrawn. USB 3 does
  not need a task. Room for larger streams is sufficient reason, and the
  GPS result is the only gate. Kevin: the repair on the cut extension
  closes the shield fully and it touches the original shield, so that
  cable is not the first suspect (item 2). Kevin's plan: remove the hub
  from the USB 3 path. The OAK cable goes to a USB 3 port of the Pi
  through the cut extension. The camera end of the cut 5 V wire goes to a
  5 V output of the UPS board, through a fuse of approximately 2 A. The
  Pi end of that wire stays open. The Arduino goes directly to a port of
  the Pi. Do this with the X1202, not with the UPS HAT (B): the camera
  load on the HAT (B) cells is the condition of 2026-11-04 (6.34 V).
  NOTE: USB 3 does not increase the depth resolution or the detection
  resolution. The stereo sensors give 640x480 at most (640x400 in use),
  and the detector runs in the camera at 640x352. USB 3 permits more raw
  streams and larger raw streams to the Pi at the same time: VIO together
  with depth, and colour frames larger than 640x480 for the host (the
  colour sensor is 13 MP; the hand detector gets 640x480 today).
  CAUTION: a fast unplug and replug of the OAK during a colour-stall
  watchdog restart hung the vision worker on 3.6.1 (2026-11-06 16:14);
  the hang detector restarted wall-e once the arm switch was off.
  CAUTION: the unplug for the turn-over made depthai 3.3.0 abort at
  14:59:06 (`Resource deadlock avoided` in `PipelineImpl::wait`, the
  self-join that 3.6.1 fixes). systemd restarted wall-e before the camera
  was back, so wall-e ran with "OAK-D Lite not detected" (no obstacle
  avoidance, no follow-me) until a disarmed restart at 15:05. After any
  camera unplug, check for that line and restart wall-e disarmed.
- Main-switch pull in the robot, 2026-11-04 17:27: the HAT current jumped
  from −46 mA to −1105 mA, then −1646 mA at 6.34 V within 5 s, and the
  critical rule halted the Pi on one sample (clean: the LED went red, not
  dark). The extra current is the USB hub: with its own supply off, it
  back-powers from the Pi's ports, so the HAT also carries the hub, the
  OAK and the Arduino. 6.34 V is close to the HY2120 cutoff (5.8 to 6.0 V).
  The shutdown took 5 s (17:27:15 to 17:27:20), and the hook's +600 s alarm
  (the old rule) woke the Pi at 17:37:32 into a `present` boot check.
  Fix: cut the 5 V wire in the Pi-to-hub cable (Kevin is getting a USB 3
  extension to cut). The critical halt's first alarm is now +90 s, not
  +600 s (1aa9621).
  2026-11-05 about 14:40: the cut cable is in (red wire only; ground, both
  data pairs and the braid kept). The hub enumerates on both sides (05e3:0610
  at 480M, 05e3:0626 at 5000M), so it does not need the Pi's 5 V to see the
  host. No over-current trips that boot, EXT5V 5.18 V, `throttled=0x0`, and
  the OAK recovered on its own (camera health HEALTHY at 14:42:56). The
  Arduino is on hub port 2 too (ttyUSB0), so no device draws from the Pi's
  ports. Main-switch pull at 14:52:37 with the cut cable, wall-e running:
  −785 to −850 mA at 6.95 to 6.68 V for 4.5 s, then the normal 5 s absent
  halt (not the critical rule). Before the cut: −1105 to −1646 mA at 6.34 V
  (2026-11-04 17:27) and −1356 mA at 6.39 V (2026-11-05 14:29, critical
  halt). The cut removed about 0.5 to 0.8 A, which is the hub, the OAK and
  the Arduino. The remaining load is the Pi, wall-e and the RTK on the 5 V
  header. 6.68 V keeps about 0.7 V above the HY2120 cutoff with these
  cells.
- The RTK GPS (I2C 0x20) is powered from the Pi's 5 V header pin. The HAT
  feeds that same line, and it stays live while the Pi is halted (only the
  Pi's own rails switch off). The RTK therefore runs, and drains the HAT's
  cells, the whole time the Pi is in standby. Measure its draw. If the
  module accepts 3.3 V (VERIFY on its datasheet), move its power to the
  Pi's 3V3 pin, which switches off at the halt. For storage, switch the HAT
  off either way. 2026-11-13: the DFRobot wiki gives the rover's supply as
  3.3 V to 5 V, so 3.3 V is in range; its draw is still not measured
  (`docs/rc_arduino_gps_wiring.md`, section 8).
- Wake-check lockout, 2026-11-06 08:42-09:13. Kevin switched main power off
  to fit new, charged 18650s (hard halt at 6.68 V, -773 mA on the old
  cells, chain open). After the swap, the 5 V came up slowly and the Pi
  stayed in standby (solid red) until the power button. With main power
  back, the fresh cells sat above the 7.5 V supply and carried the Pi:
  `mains still absent ... discharge 11/11, strong_band 11, chain hard,
  chain_wakes 12, 7.568 V, -448 mA ... powering off`. The two-sided band
  rule (|V - S| > 0.03 V) read a pack above the supply as the outage going
  on. Recovery: Kevin pulled one cell, so the battery leg read about 0 mA
  (`mains present`, chain closed), then put it back. Fix: the strong-band
  count is one-sided, V < S - 0.03 V only (a halt fires at or under
  S - 0.30 V, so a rebounding pack stays under S). Grok second opinion
  (session `01a1b16d`) agreed, and added: (1) when the pack is above S
  during a real outage, the Pi now stays up on the cells until run mode's
  7.2 V line, not on RTC polls; (2) a better test resamples a pack above S
  after 20-30 s (with mains present V falls to S and the current collapses;
  without mains it does not). Item (2) is not built.
- Flat pack and no start, 2026-11-08 to 2026-11-10. Kevin switched main power off on 2026-11-08 at 19:33. The pack fell at once from the 7.5 V float to 6.848 V at -840 mA (about 0.8 Ω, not better than the old cells). The watcher declared a hard loss at 19:33:27 and the halt was clean. Six poll wakes followed (19:35:16 to 19:41:31). Each one read `low 11/11` and powered off again, and the pack fell from 6.732 V to 6.420 V, about 0.06 V for each wake. The seventh wake (alarm at 19:42:31) did not complete: the pack was flat about 9 minutes after the switch-off, the 5 V dropped, and the RTC lost its time. The next boot needed no ext4 recovery. On 2026-11-10 Kevin switched main power on at about 08:30. The Pi stayed in standby until he pressed its power button at 10:30. The boot check then read `present 11/11`, 7.528 V, -22 mA. Conclusions: (1) with these cells, a deliberate power-off of more than some minutes ends in a flat pack; (2) after a flat pack the Pi does not start by itself when main power returns, so press its power button; (3) possible cause, not proven: one weak cell or an unbalanced pair, because the voltage fell about 0.3 V for about 30 mAh. The HY2213 balancers act only near 4.2 V for each cell, so a 7.5 V float never balances the pack. Next check (Kevin): measure each cell with a multimeter. A difference of more than about 0.1 V points at an unbalanced pair. If the two cells are equal, suspect the holder contacts and the HAT's battery path.
- After a cell swap, expect the Pi in standby (solid red): press its power
  button. A slow 5 V ramp leaves a Pi 5 in standby.
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
runtime suspend, `power/control`, `usb3_hardware_lpm_u1`, VIO, ferrite, braid, Grok,
Kevin.
