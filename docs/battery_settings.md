# Traction battery: BMS and VESC settings

Status: review of 2026-11-05. The "Now" columns are the measured values.
The "Target" columns are recommendations. Kevin makes the BMS changes in the
Daly phone app. The VESC changes (section 4) can be made in the VESC Tool
phone app or with `tools/vesc_battery_limits.py` (section 4.1).

## 1. The pack and the charger

| Item | Value | Source |
|---|---|---|
| Chemistry | Li-ion NMC ("Ternary") | Cells read 3.872-3.890 V at about 75 % charge |
| Series cells | 13 | BMS cell list |
| Capacity | 7 Ah | Kevin, BMS rated capacity |
| Charger | XM2703, 54.6 V 4 A (4.20 V/cell) | Label |
| BMS | JHB-R24TH-V1.3 (24S 40 A hardware), software 70_250317_01T2, Daly-app compatible | Daly app, version page |
| Internal resistance | about 42 mOhm | Stand test 2026-11-05: 0.5 V rise at 12 A regen |

The cell model is not known. The current targets below assume a typical
7 Ah 2P pack. Replace them with the cell datasheet values when the model is
known.

## 2. Findings

1. **The BMS trips at the end of every full charge.** Cell overvoltage
   protection is 4200 mV and pack overvoltage protection is 54.6 V. Both
   are equal to the charger target. On 2026-11-03 18:19-19:05 the BMS
   reported `Cell OVP Trip` for 45 minutes. A trip opens the charge FET.
   With the charge FET open, regen current has no path into the pack. The
   bus voltage then rises until the VESC over-voltage fault at 90 V, and
   speed-mode braking stops.
2. **The BMS battery type is LTO.** The pack is NMC. The voltage limits were
   edited by hand, so the label and the limits do not agree. The type can
   change the state-of-charge curve: the BMS shows 88 % at 3.89 V/cell,
   which is about 70-75 % for NMC.
3. **The VESC regen limit is far above the BMS charge trip.** Each VESC
   allows -40 A of regen (-80 A total). BMS charge over-current protection
   is 40 A. One hard stop can trip it, which opens the charge FET (finding
   1).
4. **The VESC drive limit is above the BMS discharge trip.** Each VESC
   allows 40 A from the battery (80 A total). BMS discharge over-current
   protection is 40 A. A stall or a climb can open the discharge FET and
   cut all power.
5. **The VESC low-voltage cutoff ends below the BMS cutoff.** VESC battery
   cutoff is 40.8 V start and 36.0 V end. BMS pack under-voltage protection
   is 40.0 V. The BMS cuts power before the VESC finishes its soft ramp.
6. **The regen cutoff is off.** Battery Voltage Regen Cutoff is 1000/1100 V
   (the FW 6.05 default, which means off) on both VESCs.
7. **The right VESC reads the bus about 0.9 V high.** The left VESC reads
   within 0.1 V of the BMS (50.5 V, the sum of the 13 cells). The right VESC
   read 0.9-1.0 V higher in every sample from 50.5 V to 54.5 V. VESC Tool
   has no input-voltage calibration that is known to us, so each voltage
   threshold on the right VESC gets +0.9 V.
8. **The right VESC is set to 12 series cells.** The pack has 13.
9. **Cold weather opens the charge FET.** BMS charging low-temperature
   protection is 1 C. Below it the charge FET opens, and regen braking is
   lost while driving still works (finding 1). No software guard exists
   yet.

## 3. BMS settings (Daly app)

CAUTION: Changing the battery type can load a preset and overwrite every
page. Before the change, keep the screenshots of every page. After the
change, compare each page with them and set each value again.

CAUTION: Do not raise the overvoltage limits before you measure the charger.
Measure the charger output at the end of a charge. If it floats above
54.6 V, keep 4200 mV and get a charger that ends at 54.6 V or lower.

The BMS shares one Bluetooth link with `wall-e.service`. Stop the service
before the app connects, and start it again after.

UPDATE 2026-11-10 (Kevin): every BMS target in this table is applied. The "Now" column shows the values of 2026-11-05. The release values were not confirmed.

| Setting | Now | Target | Reason |
|---|---|---|---|
| Battery type | LTO | Ternary | Finding 2 |
| Cell strings | 13 | 13 | Check after the type change |
| Rated capacity | 7.0 Ah | 7.0 Ah | Check after the type change |
| Cell overvoltage protection | 4200 mV | 4250 mV, release about 4200 mV if the app shows it (section 5) | Finding 1. 4.25 V is the usual NMC trip, 50 mV above the charger |
| Pack overvoltage protection | 54.6 V | 55.3 V, release about 54.6 V if the app shows it (section 5) | Finding 1 (13 x 4.25 V) |
| Cell undervoltage protection | 3000 mV | 3000 mV | Correct |
| Pack undervoltage protection | 40.0 V | 40.0 V | Correct |
| Voltage difference protection | 800 mV | 800 mV | Acceptable |
| Charge over-current protection | 40.0 A | 25 A | Above the VESC regen total (10 A) with margin |
| Discharge over-current protection | 40.0 A | 40.0 A | Above the VESC drive total (30 A) |
| Short-circuit trigger | 280 A | 280 A | Hardware protection |
| Charging temperature | 1-50 C | 1-50 C | Correct for Li-ion; see finding 9 |
| Discharging temperature | -20-50 C | -20-50 C | Acceptable |
| Balancing | 4000-4200 mV start/stop, 10 mV, 1.0 A, active | No change | Acceptable |
| Emergency force start | OFF | OFF | Keep off |

## 4. VESC settings (VESC Tool phone app, both VESCs)

Read the configuration first. Change only these values. Then write it.
Do not run the motor setup wizard after this: it overwrites the cutoffs.
The left VESC is CAN id 2 and the right VESC is CAN id 1.

UPDATE 2026-11-11 20:26: the VESC targets (part A) are applied on both
VESCs with `tools/vesc_battery_limits.py` (section 4.1). Each one was
verified live and again after a reboot (the copy in flash), and
`tools/vesc_info.py` read back all 7 values with no fault. The "Now" column
shows the values before that write. The backups of the old configuration
are in `/home/pi/vesc_backups/` and in `~/Documents/walle/vesc_backups/` on
Kevin's Mac.

| Setting (VESC Tool name, MCCONF field) | Now | Target left (id 2) | Target right (id 1) | Reason |
|---|---|---|---|---|
| Battery Current Max Regen (`l_in_current_min`) | -40 A | -5 A | -5 A | Finding 3. A stop from 1.6 m/s with 70 kg needs about 7 A in total |
| Battery Current Max (`l_in_current_max`) | 40 A | 15 A | 15 A | Finding 4. 30 A total is under the 40 A BMS trip |
| Battery Voltage Cutoff Start (`l_battery_cut_start`) | 40.8 V | 44.2 V | 45.1 V | Finding 5 (3.40 V/cell), finding 7 |
| Battery Voltage Cutoff End (`l_battery_cut_end`) | 36.0 V | 41.6 V | 42.5 V | Finding 5 (3.20 V/cell), finding 7 |
| Battery Voltage Regen Cutoff Start (`l_battery_regen_cut_start`) | 1000 V | 54.4 V | 55.3 V | Finding 6, finding 7 |
| Battery Voltage Regen Cutoff End (`l_battery_regen_cut_end`) | 1100 V | 54.9 V | 55.8 V | Ends 0.4 V under the 55.3 V pack trip; the highest cell stays near 4.233 V |
| Battery Cells Series (`si_battery_cells`) | 13 left, 12 right | 13 | 13 | Finding 8 |
| Battery Capacity (`si_battery_ah`) | 7 Ah | 7 Ah | 7 Ah | Correct |
| Maximum Input Voltage (`l_max_vin`) | 90 V | 90 V | 90 V | No change now; a lower fault does not restore braking |

Order: change the VESC settings first. They are safe now, and they keep the
regen current far under the BMS charge trip. Change the BMS overvoltage
limits after the charger is measured.

Expected result: at a true 100 % charge the regen ramp is already active,
so braking is weak until the pack drops below about 54.4 V. That is the
cost of a full charge. A charger that ends at 4.10-4.15 V/cell removes it.

Read back the VESC values after the change:
`/home/pi/.venv/bin/python3 tools/vesc_info.py` on the robot (read-only).

### 4.1 Software method (2026-11-11)

`tools/vesc_battery_limits.py` writes the same targets over CAN. It uses
the method of VESC Tool: it reads the live motor configuration
(`COMM_GET_MCCONF`), changes only the bytes of the 7 fields in the table,
and writes the configuration back (`COMM_SET_MCCONF`). Then it reboots the
VESC (`COMM_REBOOT`) and reads the configuration again, to prove the copy in
flash. The targets are in the `TARGETS` table of the tool. If you change a
target in this section, change the tool too.

The tool does a dry run unless `--write` is given. The dry run only reads,
and it can run while `wall-e.service` runs.

The firmware ramps regen down between the two regen cutoff values: full
regen below start + 0.1 V, no regen above end - 0.1 V. On the left VESC
that is full regen below 54.5 V and no regen above 54.8 V.

WARNING: The firmware does not check that the motor is stopped, and the
flash store and the reboot release both motors (no brake current). Put the
robot on level ground or chock the wheels before a write.

CAUTION: The firmware does not report a failed flash store. After a bad
store the VESC boots on its old configuration or on the firmware defaults
(99 A, no cutoffs, 3 cells), and GET_VALUES shows no fault. The reboot check
of the tool finds both. Do not drive until the tool reports "VERIFIED: live
and stored" for both VESCs.

Procedure (on the robot, `/home/pi/wall_e-Mini`):

1. Disarm the robot. Put it on level ground or chock it. Disconnect the
   VESC Tool phone app.
2. Do the dry run: `/home/pi/.venv/bin/python3 tools/vesc_battery_limits.py`.
   Examine the field table. Each VESC must show only the 7 fields. While
   wall-e runs, the dry run always reports "a write now would be REFUSED"
   (service active, no deploy hold, command timeout not active). That is
   correct.
3. Get Kevin's permission to stop the service. Then set the auto-deploy hold
   and stop the service:
   `touch /home/pi/wall_e-Mini/.deploy_hold` and
   `sudo systemctl stop wall-e`. (Without the hold, an auto-deploy restarts
   the stopped service.)
4. Do the dry run again. Both VESCs must now show "gates pass".
5. Write: `/home/pi/.venv/bin/python3 tools/vesc_battery_limits.py --write`.
   The tool writes, verifies and reboots the left VESC (id 2), then the
   right VESC (id 1). It stops at the first failure. A VESC that is already
   at target is not written again, but it is rebooted, to prove the copy in
   flash.
6. Read the exit code before you start the service:
   - 0: both VESCs are verified, live and in flash.
   - 1: nothing was changed (a gate refused, or the store did not take).
     Correct the cause and run step 5 again.
   - 2: the CAN bus did not open.
   - 3: a write failed, or a VESC reports a fault. The state is uncertain.
   - 4: the two VESCs have (or may have) different limits.
   - 130: interrupted.
   For 3 and 4, do not drive. Run step 5 again, or use the restore command
   that the tool printed. A restore of one VESC always exits 4: restore the
   other VESC too, or run step 5 for both.
7. Start the service and remove the hold:
   `sudo systemctl start wall-e` and `rm /home/pi/wall_e-Mini/.deploy_hold`.
8. Copy the new backups in `/home/pi/vesc_backups/` off the robot.

Gates (checked first, and again immediately before each write and before
the reboot): the service is "inactive"; the deploy hold file exists; the
VESC reports its command timeout as active (no device sends it motor
commands); no motor turns (100 eRPM or more); the input current is below
1 A; no fault; the bus voltage is between 30 V and 60 V; the hardware is
75_100_V2 with firmware 6.05; the VESC UUID is the one pinned for its CAN
id; each of the 7 fields holds its documented "Now" value or its target;
the live configuration agrees with the newest backup of that VESC outside
the 7 fields (`--accept-live` overrides only this check).

Backup and restore: before each write the tool saves the live configuration
in `/home/pi/vesc_backups/` (a `.bin` file and a `.json` file with its
SHA-256, with the VESC id and UUID in the name). The backup includes the
motor detection values. To put a backup back, use
`tools/vesc_battery_limits.py --restore <file>.bin --id <id> --write`. The
tool refuses a backup with a different VESC UUID or a wrong SHA-256. The
firmware rounds some fields by one step on each write (VESC Tool does the
same), so a restore gives those fields back to within one step.

## 5. Charger measurement (2026-11-05, BMS log)

wall-e logged the BMS during the charge of 10:28-11:10 (`logs/run_20261105_*.log`).

| Time | Pack | Current | Highest cell | State |
|---|---|---|---|---|
| 10:28-10:59 | 50.6 to 54.1 V | 3.5 to 3.6 A | 3899 to 4177 mV | Constant current |
| 11:00-11:09 | 54.2 to 54.4 V | 3.0 A falling to 1.0 A | 4181 to 4196 mV | Constant voltage |
| 11:09:59 | 54.4 V | 0 A | 4193 mV | `Cell OVP Trip`, charge FET off |

Result: the charger holds the pack at about 54.4 V (0.1 V BMS resolution)
in its constant-voltage stage. That is at or below the 54.6 V target, so
the charger is correct and the overvoltage targets in section 3 are safe.
The trip came from the old 4200 mV limit while 1.0 A still flowed. The
charge did not reach its natural end.

After the trip the cells rest at 4170-4186 mV, and the trip stays latched
(charge FET off from 11:09:59 to at least 15:05, with the charger on).
CAUTION: An overvoltage release below the resting cells (for example
4150 mV) keeps the charge FET off after every full charge, and regen then
has no path into the pack. Set the cell release above the resting full
cells, at about 4200 mV, with the 4250 mV trip, and the pack release at
about 54.6 V, with the 55.3 V trip.

## 6. Open items

- Done 2026-11-05: the charger measurement (section 5).
- Cell model and datasheet, to set the current limits from the cell rating.
- A software guard: refuse to arm, or limit speed and deceleration, while
  the BMS reports the charge FET open, a cell overvoltage trip, or a charge
  low-temperature condition.
- Grok review 2026-11-05 (session `01a1ac85-5691-7c90-ad85-46e7244b4b5d`):
  adopted -5 A regen, the charger check, the preset caution, the
  cold-weather finding and offsets on both cutoff pairs. Not adopted: a
  regen ramp that ends at about 54.3 V. With the 55.3 V pack trip, 54.9 V
  keeps a 0.4 V margin and leaves some braking at a full charge.

## Appendix: Technical Names

BMS, VESC, VESC Tool, Daly, JHB, NMC, Ternary, LTO, Li-ion, FET, MCCONF,
CAN, Grok, `wall-e.service`, XM2703, regen, Bluetooth, mOhm, flash,
`COMM_GET_MCCONF`, `COMM_SET_MCCONF`, `COMM_REBOOT`, GET_VALUES, UUID,
SHA-256, eRPM, auto-deploy hold, `tools/vesc_battery_limits.py`, 75_100_V2.
