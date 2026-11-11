# Charger inhibit review, 2026-11-10

Owner: Kevin. This document is a decision memo for audit item 7 in
`docs/status_audit_2026-11-10.md`. It replaces no other document. A
separate commit adds telemetry only (section 7). No change in this review
changes when the robot can arm or move.

Sources: the `wall-e` journal from 2026-10-20 to 2026-11-10 19:30, and all
JSON run logs and arm logs on the robot (2026-10-25 to 2026-11-10 19:30).
The robot was read only. Scripts and extracts are in the session scratch
folder (`charger/scan_logs.py`, `charger/scan.csv`). Each finding is marked
PROVEN (seen in the logs) or INFERRED (the best explanation, not seen
directly).

## 1. Summary

1. The BMS Bluetooth link goes dark when the robot sits on the charger
   after the charge ends. In 5 of 5 cases with data, the link went dark
   45 to 62 minutes after the pack current became 0.0 A. With the Pi and
   the VESCs on the pack, the current is 0.0 A only when the charger holds
   the pack at its float voltage. PROVEN for the timing; INFERRED for the
   cause (the Daly sleep timer, refer to section 2.3).
2. The link is dark now for this reason. Since 11:18 the left VESC bus
   reads 54.4-54.5 V with zero current, the same as every other float
   period. INFERRED: the robot is on the charger.
3. The inhibit was on for about 3.6 h of about 60 h on the charger
   (about 6 %). It engages only in the constant-current phase. It goes off
   when the taper current falls to 2.0 A, and then the link goes dark.
4. The robot was armed 3 times with the link dark (of 55 arm sessions).
   Each time was the start of a field day, directly after the charge
   period. The BMS woke only when the drive current started.
5. A release of the inhibit while armed sends the held stick to the motors
   in one tick (2026-11-05 09:31:19.72: 126 to 0, full reverse). Refer to
   section 4.3.
6. The 42 h dark period in the audit is wrong. The link was dark from
   2026-11-06 18:47 to 2026-11-07 12:17 (17.5 h) and from 2026-11-07 20:09
   to 2026-11-08 12:39 (16.5 h). It was up from 12:17 to 20:09 on
   2026-11-07.

## 2. The link

### 2.1 How the logs show the link

- A failed connect prints a journal WARNING about every 90 s
  (`... was not found. — retrying in 60s`). The retry is 60 s plus the
  30 s connect timeout.
- A good poll prints nothing to the journal (INFO is dropped), except the
  `BMS protection flags active` ERROR line when the pack is near full.
- The run logs carry `bms.connected` on every line. `connected` becomes
  False only when a connect fails. The other `bms` values stay at the last
  polled value.

### 2.2 Timeline

Times are local (CST). "Pi off" means no journal and no log lines.

| From | To | Duration | Link | What happened |
|---|---|---|---|---|
| 10-20 00:00 | 10-24 13:50 | 4.6 days | Dark | Up for about 1 min every 6 h (01:13, 07:12, 01:09, 07:08, 13:07, 19:06, ...), each wake about 4 min earlier than the last. Some wakes were not seen. Pack near full (OVP warning flags). |
| 10-24 13:51 | 10-24 20:28 | 6.6 h | Up | Cause not known. |
| 10-25 10:14 | 11-03 18:19 | 9.3 days | No data | The Pi ran (boot -70), but `wall-e` wrote nothing. Not examined. |
| 11-03 18:19 | 11-03 19:05 | 0.8 h | Up | End of charge, `Cell OVP Trip` (charge FET open). |
| 11-03 19:05 | 11-04 17:06 | 22.0 h | Dark | One 20 s wake at 12:51 (54.3 V, 0.0 A). 9 service restarts did not wake it. |
| 11-04 17:06 | 11-05 12:09 | 19.1 h | Up | Came up 16 s after a service start, with -0.2 A (charger off). Overnight at -0.3 A it stayed up. Charge 08:40-09:31, charge 10:28-11:10. |
| 11-05 12:09 | 11-05 14:56 | 2.8 h | Dark | 60 min after the charge FET opened at 11:09:59 (0.0 A). |
| 11-05 14:56 | 11-06 11:16 | 20.3 h | Up | Overnight at -0.3 A. Charge 08:22 to about 10:14. |
| 11-06 11:16 | 11-06 15:58 | 4.7 h | Dark | 62 min after the current fell to 0.1 A. Bus 54.5 V flat. |
| 11-06 15:58 | 11-06 18:47 | 2.8 h | Up | Came up at the arm, -0.9 A (charger off). Top-up charge 17:22. |
| 11-06 18:47 | 11-07 12:17 | 17.5 h | Dark | About 50 min after 0.0 A. Kevin armed at 12:16:09 and 12:17:45 in the dark. |
| 11-07 12:17 | 11-07 20:09 | 7.9 h | Up | Woke 13 s after the first motion. Charge 18:03. |
| 11-07 20:09 | 11-08 12:39 | 16.5 h | Dark | 61 min after the current fell to 0.1 A. Kevin armed at 12:36:56 in the dark. |
| 11-08 12:39 | 11-08 16:20 | 3.7 h | Up | Woke 98 s after the first motion. Charge 14:30; 0.0 A from 15:35. |
| 11-08 16:20 | 11-08 19:33 | 3.2 h | Dark | 45 min after 0.0 A. |
| 11-08 19:43 | 11-10 10:31 | 38.8 h | No data | Pi off (UPS flat after the main power was switched off). |
| 11-10 10:31 | 11-10 11:18 | 0.8 h | Up | Up at the Pi start. 54.6 V, 0.0 A from the start. |
| 11-10 11:18 | 11-10 19:30 | 8.2 h | Dark | 47 min after the start at 0.0 A. One 30 s wake at 17:15. 12 deploy restarts did not wake it. |

### 2.3 Pattern

PROVEN:

1. Each loss since 11-03 came 45-62 min after the BMS current became
   0.0 A (or 0.1 A) on a full pack. 5 of 5 cases with data.
2. With the charger off, the Pi and the VESCs draw -0.2 to -0.3 A. At that
   current the link stayed up for 13-20 h overnight (11-04 and 11-05).
3. A service restart does not wake the BMS (9 restarts on 11-04, 12 on
   11-10).
4. Drive current wakes it. The link came back 13 s (11-07) and 98 s
   (11-08) after the first motion. The 60 s retry and the 30 s connect
   timeout explain up to 90 s of that delay.

INFERRED:

1. The Daly BMS goes to sleep after about 1 h with no current (Daly
   "sleep waiting time", often 3600 s). Asleep, it stops the Bluetooth
   advertisement. It wakes about every 6 h for about 1 min (October data),
   and it wakes on drive current.
2. The 0.3 A idle load does not wake it within a few minutes (11-07 and
   11-08: dark for 1-4 min after the unplug until the drive started). It
   may wake it after a longer time (11-04 17:06).
3. On this robot, "Pi up, VESCs alive, link dark" almost always means "on
   the charger after the charge". No dark period since 11-03 occurred
   without a charge before it.

Not seen: a link loss during a drive, the phone app (no evidence of use
in this window), or a dark link while charging at constant current.

### 2.4 What the code can use

- The dark link is itself evidence of the charger in this data.
- The link wakes on drive current. A policy that needs a live link before
  the first motion cannot work alone: the BMS does not wake without the
  motion.
- A shorter retry when the link is dark does not help: the BMS does not
  advertise. It can only cut the 90 s delay after a wake.

## 3. Charging episodes

`is_charging()` is True only with the charge FET on and more than 2.0 A
on 2 consecutive polls.

| Episode | Inhibit on | Inhibit off with charger connected | Armed or moved on the charger |
|---|---|---|---|
| 11-05 08:40-09:31, 47.5 to 51.5 V, 3.5 A | 08:40:44-09:31:19 | 10 s after the service restart at 09:10 (debounce) | Armed 09:31:01 and 09:31:17 with the inhibit on. Motors held at 126. The charger came off at 09:31:19 with the stick at full reverse: the next tick sent 0, 0. |
| 11-05 10:28-11:10, to 54.4 V, cell max 4199 mV | 10:28:05-11:03:13 | 11:03:13 (2.0 A taper) to 11:09:59; then the charge FET opened (0.0 A) | 11:22:44: armed, 150 bytes for about 1 s at 3900 ERPM, BMS 0.0 A, charge FET open. Charger state not known. |
| 11-06 08:22 to about 15:58, to 54.6 V, cell max 4205 mV | 08:22:26-09:45:40, less 10 s after each of 2 restarts | 09:45:40 to about 15:58 (6.2 h); link dark from 11:16 | Armed 15:58:07, about when the bus fell from float. |
| 11-06 17:22 to 11-07 12:16, top-up | 17:22:53-17:25:08 | 17:25 to 11-07 12:16 (19 h); link dark from 18:47 | Armed 11-07 12:16:09 and 12:17:45 with the link dark, bus 54.4-54.5 V. Drove at 12:17:58. |
| 11-07 18:03 to 11-08 12:36, 50.9 to 54.6 V | 18:03:50-18:32:24 | 18:32 to 11-08 12:36 (18 h); link dark from 20:09 | Armed 11-08 12:36:56 with the link dark, bus 54.4 V. Drove at 12:37:56. |
| 11-08 14:30 to now, 52.6 to 54.6 V | 14:30:56-14:43:30 | 14:43 to 19:33 and 11-10 10:31 to now; link dark from 16:20 and 11:18 | Armed and disarmed at 14:30:44 and 14:30:51, in the 10 s debounce. No motion. |

Charge FET and cell max: the charge FET stayed on in every episode except
11-03 and 11-05 10:28 (cell OVP trip at about 4196-4200 mV). On the other
episodes the cells ended at 4202-4205 mV with the FET on and 0.0 A.

The Daly charger flag (0x94) was never True (24 logged flag lines, all
"disconnected"). The Daly mode byte (0x93) changes between `charging`,
`idle`, `discharging` and `unknown_20` from poll to poll during a charge.
Do not use either of them.

PROVEN: the robot was armed on the charger with the inhibit on (11-05) and
inside the debounce (11-08 14:30). INFERRED: the 3 dark arms came after an
unplug (the bus fell 0.1-0.2 V just before), so the robot did not drag the
cable. The logs cannot prove the unplug.

## 4. Other evidence of the charger

### 4.1 VESC bus voltage

Both VESCs measure the pack. The right VESC reads about 1.0 V higher than
the left one in every sample. Resolution is 0.1 V.

| State | Left bus | BMS current | VESC bus current |
|---|---|---|---|
| Constant current | rises 47 to 54 V at about 0.1 V/min | +3.5 A | 0.0 A |
| Taper and float | 54.4-54.5 V, flat for hours (11-10: 315 of 322 samples at 54.5 V) | 0.0 to +1.0 A | 0.0 A |
| Full pack, 1 min after unplug | 54.3 V | -0.2 to -0.9 A | 0.0 A |
| Full pack, 16-30 min after unplug | 54.2 V, then 54.1 V | -0.2 A | 0.0 A |
| Driving | 50-55 V (left), sags with load | negative | -14 to +26 A |

Result: the float and a freshly unplugged full pack differ by 0.1-0.2 V.
That is one or two counts. An absolute threshold has no margin. The
constant-current rise is clear, but the BMS already sees that phase. A
regen pulse also raises the bus, but with a negative bus current and
non-zero RPM, so the current and the RPM separate it.

### 4.2 BMS current at rest (no Bluetooth needed only when the link is up)

Parked, link up: with the bus at 54.4 V or more (float), 410 samples read
0.0 to +1.0 A; 2 read -0.1 and -0.2 A (directly after an unplug). Off the
charger, about 1440 samples read -0.2 or -0.3 A; 1 sample read -0.1 A.
This band is the best separation in the data, but it needs a live poll.

### 4.3 Release of the inhibit while armed

`_reset_slew_state` sets `_slew_seen_non_neutral = False`, and
`snap_first_command` is True for MANUAL. The first command after the
release therefore passes the slew limiter without a ramp. The comment on
the pack-low latch says the release ramps from neutral; it does not in
MANUAL. Evidence: 2026-11-05 09:31:19.72, `arm_20261105_093117.log`.

### 4.4 Other measured signals

The UPS HAT measures the Pi supply, not the drive pack. The GPS, the IMU
and the camera carry no charger information.

## 5. Options

The numbers come from sections 2 and 3: 55 arm sessions, 3 with the link
dark, all at the start of a field day (2 field days of 3 in the window).

| Option | What it does | Refusals in Kevin's use | Residual risk |
|---|---|---|---|
| A. Fail open, plus a VESC-voltage inhibit | Inhibit when parked at zero current and the left bus holds at float, or rises at constant current | Unknown. A full pack directly after unplug sits 0.1 V under float, so false holds are likely for some minutes | One count of margin; misses the first minutes of constant current; no escape when it holds wrongly |
| B. Fail closed when the link is dark | No motion while the BMS data is older than 30 s | 3 of 55 arms; each a field-day start. The BMS wakes only on drive current, so B blocks with no way out | No escape. The robot cannot leave the yard if the BMS sleeps or fails |
| C. Charger latch, fail closed only after a charge was seen or at a dark start, with an operator override | Latch on charging evidence; hold it across a dark link and a restart; clear on live off-charger current or a deliberate override | The same 3 arms need the override (about once per field day). All other arms: none | The override is the hazard path: the operator must unplug first. Mitigated by the narrow override rules below |
| D. Staleness limit on the discharge-FET guard | Treat an old FET value as unknown | If "unknown" stops motion: as B. If not: none | As telemetry only, it closes no gap. Committed as telemetry (section 7) |

## 6. Recommendation

Option C, in this form (the second opinion changed items 1, 3 and 4):

1. Set the latch on confirmed charging, and also on 3 consecutive live
   polls while disarmed with the current at -0.05 A or more and the pack
   at 54.0 V or more. This covers a plug-in that starts in taper or float.
2. Hold the latch across a dark link. Persist it to a file, so a restart
   keeps it. Treat a start with the link dark and no file as latched.
3. Clear the latch only on 3 consecutive live polls in the off-charger
   idle band (-0.15 to -1.5 A while disarmed), or on a drive current.
   Never clear on "below zero" alone.
4. Allow the operator override only when no live BMS data exists. Do not
   use a normal re-arm as the override: use a pattern that is not a retry
   (for example a phone button "CHARGER UNPLUGGED", or ch3 off-on-off-on
   in 3 s with both sticks centred). Log it loud. It lasts for one armed
   session. Autonomous modes cannot use it.
5. Fix the slew snap first: after any inhibit or latch release while
   armed, ramp from neutral (do not snap). Ship it as its own commit.
6. While armed and driving, a link loss does not stop the robot. Drive
   current is not the charger.

Runner-up: option A. It needs no Bluetooth, but the float and a freshly
unplugged pack are one ADC count apart. Keep the VESC bus voltage as a
logged corroboration only.

Residual risk with C: an operator who uses the override with the cable
still connected. The override rules make that a deliberate act, not a
retry. A BMS that dies while parked off the charger needs the override
once.

Second opinion: Grok, session `01a1c846-e4a0-7fb1-af8b-1183cdad0a3f`,
effort low. It agreed with C over A and B. Its objections, all accepted:
the set rule on more than 2 A misses a plug-in in taper; the "second ch3
cycle" override is a normal retry and trains a bypass; the clear rule must
use the idle band, not "below zero"; fix the slew snap before any latch
can release while armed. It also noted that A misses the first minutes of
constant current. Correction to the numbers sent to it: the inhibit share
is about 6 %, not 13 %.

### 6.1 Proposed diff (not committed)

- `pi_app/hardware/bms.py`: add `ChargerLatch` (set, clear and override
  rules above), persisted to `logs/charger_latch.json`; `is_charging()`
  stays as it is; a new `charger_latch_active()` reports the latch.
- `pi_app/app/main.py`: `controller.set_charger_inhibit(is_charging() or
  latch_active or _bms_safety_stopped, ...)`.
- `pi_app/control/controller.py`: on an inhibit or pack-low release while
  armed, set `_slew_seen_non_neutral = True` so the slew ramps from
  neutral.
- Override input: phone endpoint (local only) or a ch3 pattern in
  `pi_app/control/safety.py`.
- Tests: latch set in taper; latch survives a dark link and a restart;
  clear only in the idle band; override refused with live on-charger
  data; release ramps from neutral (replay of 2026-11-05 09:31:19).

## 7. Committed (no decision needed)

Telemetry only. The commit changes no inhibit, no guard and no arming.

- `BmsService.get_link_health()`: state `pending` / `up` / `dark` (the
  same 30 s limit as the fail-open check), data age, dark time.
- `BmsService.check_link()`: journal WARNING when the link goes dark,
  every 10 min while it stays dark, and when it comes back.
- Run log `bms` block: `link_state`, `age_s`. `/api/telemetry`:
  `bms_link_state`, `bms_data_age_s`. Status line: `BMS(dark Nm)` while
  dark.
- One WARNING per armed session when the robot is armed with the link
  dark, with the age of the discharge-FET value.

## 8. Decisions for Kevin

1. Approve option C, or choose another option.
2. Choose the override input: a phone button, a ch3 pattern, or both.
3. Approve the slew fix (ramp from neutral after a release while armed)
   as a separate change before the latch.
4. Approve a persisted latch file in `logs/`.
5. Decide whether to change the Daly sleep time in the Daly app (stop
   `wall-e` first; one link only). A longer or disabled sleep keeps the
   link up on the charger, at a small standby current.
6. Confirm your routine: do you unplug the charger before you arm? The
   logs cannot prove it for 11-07 and 11-08.
7. Correct the 42 h statement in the audit and the memory note (17.5 h
   and 16.5 h).

## 9. Technical Names

**Products and parts:** Daly, BMS, VESC, Raspberry Pi (Pi), UPS HAT,
Grok, NMC, XM2703.

**Software, files and fields:** `wall-e`, `BmsService`, `is_charging()`,
`get_link_health()`, `check_link()`, `set_charger_inhibit`,
`_reset_slew_state`, `_slew_seen_non_neutral`, `snap_first_command`,
`ChargerLatch`, `charger_latch_active()`, `bms.connected`, `link_state`,
`age_s`, `bms_link_state`, `bms_data_age_s`, `/api/telemetry`,
`logs/charger_latch.json`, `config.py`, `main.py`, `bms.py`,
`controller.py`, `safety.py`.

**Terms:** Bluetooth, BLE, link, journal, debounce, fail open, fail
closed, latch, override, inhibit, telemetry, float, taper, constant
current, regen, ERPM, ADC, FET, charge FET, discharge FET, OVP, slew
limiter, sleep timer, advertisement, deploy, CST.
