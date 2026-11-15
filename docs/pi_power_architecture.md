# Pi power for long storage: options and recommendation

Status: decision draft, 2026-11-04. Research by three agents (off-the-shelf
boards, how robot builders do it, an always-on tap from the main pack) and a
Grok second opinion (session 01a1a8bd). Nothing is bought.

UPDATE 2026-11-13: the first bench results of the X1202 are in 7.5 to 7.8.

UPDATE 2026-11-15: the fuse plan from the pack to the Pi is in `docs/fuse_plan_2026-11-15.md`.

UPDATE 2026-11-10: read section 7 first. It proposes a bought UPS board
behind the switched-side converter (option F) in place of the custom
board. Kevin orders one for a bench test.

## 1. The problem

WALL-E can sit switched off for months. The Waveshare UPS HAT (B) cannot
switch its own output off and cannot see its input, so the Pi polls with the
RTC alarm. Each wake costs about 16 s at 0.6 A from the 2S buffer. The
buffer then drains in hours to days, and after that every robot start is a
hard power-up. The halt itself works (four real pulls on 2026-11-04).

Interim rule for storage with the HAT (B): switch the HAT off with its
switch. The pack then sees only its own leakage. Switch the HAT on again
before the robot is used, or the Pi stays off.

## 2. Facts that shape every option

- A Pi 5 halted with `POWER_OFF_ON_HALT=1` is in PMIC standby: about 3 mA
  at 5 V (about 0.01 W). Without it, a halt draws 1 to 1.4 W.
- A Pi 5 wakes only from 5 V applied again, the J2 power-button pads, or the
  RTC alarm while 5 V is present. `WAKE_ON_GPIO` does nothing on a Pi 5.
- A J2 pulse while the Pi runs asks it to shut down, and over 5 s forces it
  off. Use J2 only when the Pi is known to be halted.
- A slow 5 V ramp can leave the Pi 5 in standby instead of booting. The
  supply must rise in tens of milliseconds.
- Robot vendors (Clearpath Jackal, Husarion ROSbot, Duckietown) have a
  controller ask the computer to shut down, then cut power. None treats a
  hard cut as normal. A read-only root is common insurance, not a design.

## 3. Options

| Option | Storage drain | Hold-up | Catch |
|---|---|---|---|
| A. Gated tap on the main pack (recommended) | Converter off-current plus BMS: about a year on 20 Ah (CORRECTION: the pack is 7 Ah, refer to `docs/battery_settings.md`; the storage times here are about one third as long.) | The main pack | Needs a 75 to 100 V module and a hardware force-off timer (section 4) |
| B. Witty Pi 5 on the switched rail plus supercaps | None from the pack | About 20 s of supercap at 15 W | Under the 30 s stop cap; supercap ESR is worse in the cold; timer-based cut |
| C. Latch circuit plus a small buffer | Zero after the latch opens | The buffer | DIY; output capacitance can re-latch it |
| D. Today: HAT (B) plus RTC polling | Hours to days of polling | 2S 18650 | Drains in storage; cannot cut its output |
| E. Hard cut plus read-only root | None | None | Corruption still reported on Pi 5 (boot partition); insurance only |

## 4. Option A in detail

A wide-input buck taps the 13S pack (37.7 to 54.6 V) on the battery side of
the main switch and feeds the Pi 5 at 5.1 V. There is no buffer battery.

- ENABLE comes from the ignition (the switched rail), through an opto whose
  LED is fed only from the switched side, on its own small fuse. No part
  may be able to bridge the main switch, so the switch still makes the
  drivetrain dead.
- When the ignition falls, the opto signals a GPIO. A daemon (the watcher)
  debounces 3 to 5 s and halts the Pi.
- A hardware grace timer, started by the ignition falling, holds ENABLE on
  for 60 to 90 s (longer than the debounce plus the 30 s stop cap). When it
  ends, a transistor pulls ENABLE off even if the Pi's 3V3 is still up.
  This catches a Pi that hangs and never halts. Grok: an `ignition OR 3V3`
  enable alone would let a hung Pi hold the converter on, and drain the
  pack in one to two weeks.
- A bleed drops the 5 V rail fully when ENABLE goes off, so the next
  ignition is a real 5 V application and the Pi 5 boots.
- Tap the BMS pack output (after its discharge FET), not the raw cells, so
  the BMS deep-discharge cutoff still protects the pack. Count every
  leakage path at 55 V: dividers, TVS, the ENABLE pull-down. A 100 kΩ
  divider at 50 V alone is 0.5 mA.
- The pack buffers only an ignition-off halt. A BMS cut, a blown fuse, a
  converter fault or a loose connector is still a hard cut, so keep the
  filesystem tolerant (option E on top).

Converter (Grok): a 60 V module (Pololu D42V55F5) cannot be made safe on 13S
with a TVS: a TVS that stays off at 54.6 V clamps near 90 V (Vishay:
SMBJ58A clamps at 93.6 V). Use a module rated 75 V continuous with a
100 V / 100 ms surge rating, 5 V at 4 to 6 A, a remote-off pin, an output
fuse and a crowbar. The TI LM5013 (100 V) is 3.5 A, non-synchronous, and
too small. Candidates found on 2026-11-04 (all isolated telecom bricks, no
part checked on the bench):

| Part | Rating | Off current | Price, stock |
|---|---|---|---|
| TRACO TEN 30-4811WIN | 18-75 V in (100 V/100 ms), 5.1 V 6 A, 88 % | 3 mA typ | $66.80, none at TME |
| RECOM RP30-4805SFW/N | 18-75 V (100 V/100 ms), 5 V 6 A, trims to 5.1 V; /N fails off | 3 mA max | €91, 11-week lead |
| TRACO TEN 20-4811WIN | 18-75 V, 5 V 4 A, 88 % | 2.5 mA typ | $49.05, 25 at TME |
| Mean Well RSD-60L-5 | 18-72 V, 5 V 12 A, 89 % | no remote pin | $44.18 |

A brick's 3 mA off current is about 70 mAh a day, 2 Ah a month, from the
pack. For true months of storage, put a high-side switch (a 100 V P-FET
or an LTC4368-class controller) in front of the brick, driven by the same
ignition-or-grace logic, so the brick itself is disconnected.

Bus spikes (research, 2026-11-04): no scope capture of a 13S bus was
found. The 60 V-class VESC 6 firmware limits the input to 57 V by default;
a full 13S pack (54.6 V; WALL-E's BMS read 54.10 V at 99.4 % that day)
leaves 2.4 V, and over-voltage is a fault that stops PWM, not a derate.
Trampa reports a 12S board seeing over 60 V on regen. VESC Tool 6.05 and
later have "Battery Voltage Regen Cutoff Start/End", off by default
(1000/1100 V). Setting it near 55.5/57 V is the free first fix.

Measured on WALL-E, 2026-11-04 (read-only CAN query, `tools/vesc_info.py`
on branch `claude/hopeful-euler-drwd8s`, both VESCs):

| Setting | Left (id 2) | Right (id 1) |
|---|---|---|
| Hardware, firmware | 75_100_V2, 6.05 | 75_100_V2, 6.05 |
| `l_max_vin` (over-voltage fault) | 90 V | 90 V |
| `l_battery_regen_cut_start` / `_end` | 1000 / 1100 V (off) | 1000 / 1100 V (off) |
| `l_in_current_min` (regen into the pack) | -40 A | -40 A |
| `l_battery_cut_start` / `_end` | 40.8 / 36 V | 40.8 / 36 V |
| `si_battery_cells` | 13 | **12** |
| `si_battery_ah` | 7 Ah | 7 Ah |

The hardware is 75 V class, so the 57 V fault above does not apply. The
VESC does not fault until 90 V. CAUTION: a regen spike can therefore hold
the bus anywhere up to 90 V without a fault. That is above the 75 V
continuous rating of the option A brick. The regen cutoff is off on both.
Set it before a 75 V-class brick goes on the bus. The right VESC has
`si_battery_cells = 12` on a 13S pack; that setting feeds the VESC's own
battery-level estimate. The two VESCs also read the same bus 1.0 V apart
(52.5 V and 53.5 V).

Stand test, 2026-11-05 09:47 (wheels off the ground, phone teleop, pack at
50.7 V on the left VESC, about 3.9 V a cell; every STATUS frame logged with
`candump`, about 48 Hz a VESC; data on the robot in
`logs/standtest_20261105/`): full speed (20,300 ERPM), a full reverse step,
a pivot, stick releases, a deadman stop and a teleop e-stop. The bus stayed
between 49.1 V and 52.1 V. The highest rise above rest was 0.5 V. Regen
current reached -12.1 A (left, deadman stop) and -8.4 A (right). The
drive draw reached 15.5 A on one VESC (pivot). Stops to 0 RPM took 0.53 s
to 1.01 s; the full reverse took 1.6 s. CAUTION: this is the mild case.
The wheels were free, so the inertia is small, and the pack was not full,
so the BMS took the charge. A 48 Hz sample also cannot see a spike shorter
than about 20 ms. The worst case, a full pack braking a moving robot, is
not measured. Do not use these numbers to size a converter.

Storage estimate (agent, 20 Ah pack at 50 percent, BMS asleep 0.5 mA, cell
leakage about 2 percent a month): about 8 months with the converter idling,
about 12 months with it off. (CORRECTION: the pack is 7 Ah, refer to `docs/battery_settings.md`; the storage times here are about one third as long.) A Daly BMS sleeps only after 3600 s idle.

## 5. Recommendation

1. Now, for testing: keep the HAT (B) and the watcher (they work). For
   storage, switch the HAT off.
2. Next: build option A with a 75 to 100 V converter module, the opto
   ignition sense, the grace timer and the bleed. It removes the 2S
   buffer, the charger and the RTC polling. The watcher becomes the
   ignition daemon.
3. The custom power module (`docs/power_module_spec.md`) then drops its 2S
   buffer, charger and balancers, and keeps the 12 V outputs, the USB
   rails and the MCU (which can be the grace timer).

## 6. Questions for Kevin

1. Which converter: a TRACO TEN 30-4811WIN or RECOM RP30-4805SFW/N brick
   (section 4) behind a high-side switch, or wait for the custom board?
4. Answered 2026-11-04: both are 75_100_V2 on firmware 6.05 (table in
   section 4). Open: set the regen cutoff near 55.5/57 V on both, and set
   `si_battery_cells` to 13 on the right VESC (VESC Tool, Kevin).
2. Is there a point on the robot where the BMS pack output is available on
   the battery side of the main switch?
3. Should the robot ever wake on a schedule while stored? Option A cannot,
   because nothing has 5 V while it is off.

## 7. Review, 2026-11-10: a bought UPS board behind the switched converter (option F)

Status, 2026-11-13: the X1202 is on the bench. The tests without hands are done (7.5). The go/no-go is "not yet" (7.6). The tests with hands are in 7.7.

Status before: Kevin decided on 2026-11-10 to order one X1202 for the bench tests in 7.2. Kevin buys it. No more work on the custom board (`docs/power_module_spec.md`) until the bench result is known.

Kevin, 2026-11-10: "We shouldn't have to build our own." "Other people are using robots that are Pi-powered." His conditions for a bought part:

- The Pi must not wake up to look for power.
- No cells that must stay at 100 percent for balancing. A lower charge state is better.
- One cell is sufficient. An alternative is a supercapacitor with only the energy for a clean shutdown.

Finding: section 8 of the spec rejected each bought board because of its input range (18 V or 30 V maximum). That limit does not apply behind a converter, and the robot already has an adjustable buck on the switched side of the main switch. The polling is a fault of the Waveshare UPS HAT (B) only: it cannot see its input and cannot switch its output off.

### 7.1 Option F

A Geekworm X1202 (about 49 USD, price not verified) replaces the HAT (B). The switched-side converter feeds it with about 12 V.

- Main switch off: the 12 V goes away. The board supplies the Pi from its cells. A script sees GPIO 6 low for about 10 s and halts the Pi. The board then switches its output off. Nothing takes current from the main pack.
- Main switch on: the 12 V comes back. The board starts the Pi. There is no polling and no button.

Facts from the vendor pages (`wiki.geekworm.com/X1202`, `wiki.geekworm.com/X1202_Hardware`, read 2026-11-10):

| Item | Value |
|---|---|
| Input | 6 V to 18 V DC, 3 A or more (5.5 x 2.1 mm jack or a 2-pin connector) |
| Output | 5.1 V, 5 A maximum, through pogo pins under the Pi 5 |
| Cells | four 18650 positions, cells not included, charge to 4.23 V, charge current to about 3 A |
| Start | "Auto power-on when power is applied or restored" |
| Stop | "Automatically cuts off power when the Pi is shut down"; with no Pi 5 found it shuts down in 3 s |
| Power loss signal | GPIO 6: low = input failed, high = input good |
| Charge control | GPIO 16: high = charge disabled, low = charge enabled |
| Fuel gauge | MAX17040 on I2C bus 1, address 0x36 |

Not verified:

- The X1202 cells are in parallel. The X1200 page says that its two positions are in parallel, and the X1202 LED thresholds (3.4 V to 4.2 V) are single-cell voltages. No X1202 page states it.
- Operation with less than four cells.
- The standby drain from the cells with the output off.
- What occurs when the input comes back while a shutdown is in progress.
- A charge temperature limit. None was found.
- One marketplace review says that an X1202 damaged two Pi 5 boards. No primary source confirms it.

How option F answers Kevin's conditions: cells in parallel need no balancing, so they do not need a full charge. A script can set GPIO 16 high to hold the cells near 60 percent. One or two cells are sufficient if the board accepts that.

On the robot (checked 2026-11-10): GPIO 6 and GPIO 16 are not in use. CAN uses SPI0 and GPIO 25. The fuel gauge shares I2C bus 1 with the RTK GPS (0x20).

Software consequences:

- A short script replaces `bin/walle_ups.py`. Remove `walle-ups.service` and `walle-ups-wakecheck.service`, or the old halt logic works against the new board.
- GPIO 16 is driven only while Linux runs. The script must fail to "charge disabled": on an I2C error, and above about 45 C.
- `i2c-bus-guard.service` uses the UPS status file as its witness for a good bus. It needs a new witness.
- The ch5 e-stop halts the Pi with the input still present. Find out on the bench how the board starts the Pi again after that.

### 7.2 Bench tests before the board goes on the robot

1. Power cycles, 20 or more: switch off (GPIO 6 falls, the Pi halts, the 5 V goes to 0 V); switch on (the Pi boots with no button); and the race: switch on again 2 s, 10 s and 30 s into a shutdown. The Pi must boot each time.
2. Drain and charge control: measure the cell current with no input and the output off (microamperes are necessary, not milliamperes). Confirm that GPIO 16 holds the cells near 3.8 V through reboots and halts. Confirm a boot with the cells at about 3.3 V.
3. Full load at 50 C to 55 C on the cells only (Pi 5, NVMe, and each device that the Pi feeds): `vcgencmd pmic_read_adc` shows `EXT5V_V` at 4.9 V or more and there is no undervoltage flag. Record the cell temperature during a charge.

4. Hub power from the board (Kevin's idea, 2026-11-10). The X1202 has two USB Type-A sockets and two 2-pin connectors that are 5 V outputs ("two USB sockets for 5V power output"; no current rating is given). Its USB Type-C socket is an input only. Feed the hub's power input from one Type-A socket, and keep the Pi-to-hub data cable with its 5 V wire cut. Confirm: (a) the sockets go off when the board cuts the Pi's power after a halt, or the hub and the OAK drain the cells while the robot is parked; (b) the voltage at the hub stays at 4.9 V or more with the OAK streaming; (c) the total stays under the board's 5 A. If it passes, the hub's separate supply from the main pack can be removed, and the OAK stays powered during a halt, so the camera teardown is clean.

   Variant without the hub (Kevin, 2026-11-10, for USB 3; refer to the "REVIEW 2026-11-10" entry in `docs/ups_hat_b.md`). The OAK cable goes to a USB 3 port of the Pi through the extension that has its 5 V wire cut. The camera end of that wire goes to one of the 2-pin 5 V outputs, through a fuse of approximately 2 A. The Pi end stays open. The Arduino goes directly to a port of the Pi. Checks (a), (b) and (c) apply to the camera in place of the hub. Then set `OakDetectionConfig.max_usb_speed` to `"SUPER"` and do the GPS test outdoors from a fresh GPS start with the camera streaming.
   Built, 2026-11-13 (Kevin): the hub is no longer used. The OAK-D Lite then took its 5 V from the UPS through the cut wire in its cord, and the Arduino was on a USB port of the Pi. Changed (Kevin, 2026-11-15): the OAK-D Lite is now on a Pi USB port for data and power; the Pi has `usb_max_current_enable=1` (1.6 A for all USB ports). Not recorded: the fuse, the result of checks (a) to (c), and the USB speed setting. `max_usb_speed` is still `"HIGH"` in `config.py`.

Also check first: the mechanical stack (the X1202 goes under the Pi; find out where the NVMe board is), and the RTK fix quality with the board in operation.

### 7.3 If option F fails on the bench

- Option A in its minimum form: a converter with an enable input on the battery side of the main switch, held on by "switched side alive OR the Pi's 3V3", a halt script, the Pi's hardware watchdog, and a bleed on the 5 V rail. The main pack is the only battery. This is a small quantity of hand-wired parts at pack voltage, not a PCB. Its costs: the regen overvoltage conditions of the spec (3.2a) apply, and a BMS cut is a hard cut for the Pi.
- Supercapacitors: 2 x 100 F in series give about 670 J down to 1.5 V. A 60 s halt at 6 W needs about 425 J with the converter loss. The halt must then be short: `wall-e.service` permits 90 s today, and a supercapacitor design needs a kill after about 5 s and about 15 s in total. Few bought boards of this type operate a Pi 5 and start it automatically (the Geekworm UPS Scap was out of stock and does not state it). Use 2.5 V for each cell in the heat.

### 7.4 Second opinions

Grok timed out after 900 s on this question. An independent Opus review agreed with option F and against the custom board ("215 USD and 31 risks to replace a 49 USD board"). It added the race test, the fail-safe for GPIO 16, the heat check, the mechanical check and the retirement of the old watcher. A research agent ranked the minimum option A first and warned that a UPS board "keeps the small-battery design that currently fails"; the answer to that warning is that the present failure comes from the HAT (B)'s missing input sense and output switch, which the X1202 has.

### 7.5 Bench results, 2026-11-12 and 2026-11-13 (tests without hands)

Setup: the robot's Pi 5 is on a bench, off the robot. It is on one Geekworm X1202 with two Moli ICR-18650H cells (identified from a photograph on 2026-11-13: "MOLI ... 06 ICR-18650H MADE IN CANADA"). These are LiCoO2 laptop cells of 2200 mAh when new, with a date code that is probably 2006. They are old cells. The NVMe carrier stays on top of the Pi, and the root file system is on the NVMe drive. Thus the mechanical stack is correct: the X1202 is below the Pi and the NVMe carrier is above it. CORRECTION, 2026-11-13 (Kevin): the supply is the Raspberry Pi USB Type-C supply on the USB Type-C input of the board (5 V; Kevin thinks 3 A). It is not a 12 V supply. The first issue of this section said 12 V; that was an assumption and it was wrong. All results in 7.5 are for the USB Type-C input. The robot uses the 6 V to 18 V input, and that input is not tested yet. The vendor requires 5 A or more on the USB Type-C input. A 3 A supply cannot give the Pi load and the full charge current at the same time. This is a possible cause of the VCELL step at 21:44 and of the EXT5V minimum of 5.05 V during a charge. A Z-Wave dimmer outlet (Hubitat device 98, "Christmas Tree") switches the supply. Kevin was not at the bench. The logger `tools/x1202_bench_logger.py` records GPIO 6, GPIO 16, VCELL, EXT5V, `get_throttled` and the SoC temperature at 4 Hz. The harness `tools/x1202_cycle_test.py` operates the outlet and the Pi. The raw results are in `docs/data/x1202_bench_20261113/results.jsonl`. The logs of the Pi are in `/home/pi/x1202_bench/logs/` (one file for each boot). These logs (69 files, 65 MB) were deleted on 2026-11-15 (Kevin's decision); the results in `docs/data` stay.

Results:

| Test | n | Result |
|---|---|---|
| T0: 38 min with the input on | 1 | No VCELL drop with GPIO 6 high. No GPIO 6 glitch. |
| T1: input off for 20 s, Pi in operation | 3 | 3 of 3 continue on the cells. GPIO 6 falls 0.44 s to 0.60 s after `off` and rises 1.08 s to 1.18 s after `on`. |
| T2: input off, `poweroff` after 10 s, input on 30 s after the network stops | 10 | 10 of 10 boot with no other action. The boot starts 10.9 s to 11.1 s after `on`. ssh answers after 20 s. |
| T3: input on 2 s after `poweroff` | 3 | 0 of 3 boot. |
| T3: input on 10 s after `poweroff` | 3 | 1 of 3 boot. |
| T3: input on 30 s after `poweroff` | 3 | 3 of 3 boot. |
| T4: `poweroff` with the input on | 2 | 0 of 2 start again in 3 min. An input-off of 30 s starts the Pi (2 of 2). |
| T5: `reboot` with the input on | 3 | 3 of 3 come back. |
| T5: `reboot` on the cells | 1 | The board keeps the Pi powered through the reboot. |
| T6: load on the cells | 1 | Aborted at 4 cores plus an NVMe read: EXT5V 4.896 V. |
| T7: Pi halted with the input on, then input off for N s | 4 | N = 3.8 s: 2 of 2 boot. N = 10 s: 2 of 2 boot. |

Findings:

1. The board does not start the Pi when the input returns during a shutdown. The Pi then stays off with the input present. This occurs when the input returns before the board cuts its output, which is approximately 10 s after the `poweroff` command. An input-off of 3.8 s or more starts the Pi.
2. The board does not start the Pi after a halt with the input present. The ch5 e-stop causes this condition on the robot. An input-off of 3.8 s or more starts the Pi. This is the same as the behaviour of the robot today after an e-stop.
3. The board charges the cells each time that Linux does not drive GPIO 16. The board has a pull-down on GPIO 16 that is stronger than the internal pull-up of the Pi. At each boot, GPIO 16 has no function until the logger sets it, 2.5 s after the kernel starts. At the first sample of each boot after a period with the input present and the Pi off, VCELL is 4.20 V. During the tests the rested VCELL increased from 3.78 V to 3.99 V for this reason.
4. `pinctrl set 16 op dh` stops the charge. VCELL decreases from 4.20 V to 3.97 V immediately, and it relaxes to 3.83 V in 5 min. The pin must be an output. The internal pull-up of the Pi is not sufficient.
5. GPIO 6 is clean. In 38 min with the input on it has no low sample at 4 Hz and no kernel edge event. Each input change gives one edge. The board drives the pin (it reads high with the pull-down of the Pi on).
6. With GPIO 16 high and the input on, VCELL drifts by -7 mV in 10 min after the first 5 min. A longer measurement is in finding 13.
7. The cells sag under load. From a rested 3.96 V, VCELL is 3.76 V at idle, 3.64 V with 1 core, 3.55 V with 2 cores and 3.42 V with 4 cores (60 s for each step). EXT5V is 5.12 V, 5.07 V, 5.02 V and 4.96 V. With 4 cores and the NVMe read, EXT5V is 4.896 V and VCELL is 3.385 V after 6 s. `get_throttled` stays 0x0 in all steps. The SoC temperature increases to 67 C. No USB device was connected.
8. The implied resistance of the pack is approximately 0.23 ohm. This value uses an assumed Pi power of 2.7 W at idle and 7.0 W with 4 cores, and a converter efficiency of 90 percent. A good pair of 18650 cells has less than 0.05 ohm. The cells, their holder contacts or the board path have too much resistance.
9. The abort of T6 ended in a clean halt. It was not a power loss. A defect in the harness switched the outlet on 11.7 s after the abort. In that time the bench guard of the logger saw GPIO 6 low for 203 s and halted the Pi, 1.5 s before the input returned. The Pi then stayed off (finding 1). The minimum VCELL was 3.31 V. The next boot shows no ext4 journal replay and no file system error. The harness is corrected. The correction is not tested on the hardware.
10. In the T3 failures with a delay of 2 s, GPIO 6 was high again 3.3 s to 7.0 s before the logger stopped. A check of GPIO 6 late in the shutdown can thus see that the input is back. In the failures with a delay of 10 s, GPIO 6 was still low when the logger stopped.
11. EXT5V is 5.17 V on the input and 5.12 V on the cells at idle. No boot shows an undervoltage line, an I2C error or a file system error in the journal.
12. The fuel gauge answers at 0x36 (version register 0x0002). Its state of charge is not correct: it read 24 percent to 31 percent with the cells at 3.83 V. Use VCELL.
13. Charge hold, 2026-11-13 00:34 to 02:15, GPIO 16 high and the input on: the rested VCELL decreased from 3.981 V to 3.911 V in 1.67 h. The rate decreased continuously: 50 mV in 1 h at the start, 9 mV in 1 h at the end. After the soak the rate was approximately 5 mV in 1 h. Thus the input carries the Pi while the charge is disabled. If the cells carried the Pi, VCELL would be at the loaded value (3.6 V to 3.7 V) and would decrease much faster. The remaining slow decrease is the relaxation after the charges of the night, the self-discharge of these cells, or a small current into the board. The data cannot separate them.
14. Soak, 2026-11-13 02:15 to 06:15: the Pi halted on the cells, and the outlet stayed off for 4 h. The Pi started by itself when the outlet went on (ssh after 36 s). VCELL was 3.911 V before the soak. It was 3.900 V at 2.6 min after the start and 3.891 V at 38 min (GPIO 16 high, after the charge of 13 s that each start gives). The decrease across the soak is thus approximately 18 mV in 4 h. With the input on, the same cells showed 5 mV to 9 mV in 1 h. Thus this test does not show a large drain with the output off, and it cannot show a small one. An upper limit from these numbers is some tens of milliamperes, which is not useful. The meter test in 7.7 is necessary.

Notes:

- The time from `on` to the first ssh has two values, approximately 21 s and 43 s. The boot start is the same in both. The difference agrees with the 20 s ARP hold-down of the Mac. It is probably not a property of the board.
- The dimmer outlet is not a good switch for a power supply. After an input-off of 3.8 s, the dashboard reported the dimmer level as 82 and as 84. The level was 99 before. The harness now sets the level to 99 after each `on`.
- At 21:44 on 2026-11-12, VCELL stepped to 3.65 V one time for less than 2 min with the input present. This did not occur again in 38 min. Its cause is not known. A low dimmer level is a possible cause.
- After a power-off on the bench, the wall clock of the Pi shows 2025-05-29 until the network time is available. The X1202 cuts the 5 V, and the RTC has no cell.

Results with hands, 2026-11-13 morning (Kevin at the bench, a 12 V supply on the 6 V to 18 V input, the USB Type-C cable removed, the same old cells):

- The 5 V output with the Pi in operation on the 12 V input: 5.24 V on Kevin's meter. On the USB Type-C input the Pi read 5.17 V (EXT5V).
- The 5 V output after a halt with the 12 V input present: 0 V on the meter. The board cuts the output.
- The button on the board starts a halted Pi while the input is present. Kevin pushed it approximately 2 min after the halt, and the Pi booted. The board has a 2-pin connector for an external button.
- Kevin set the 12 V supply to off with the Pi in operation. GPIO 6 went low, and the Pi continued on the cells for 66 s until a manual halt: VCELL 3.72 V to 3.77 V, EXT5V 5.10 V to 5.14 V, `get_throttled` 0x0. After the halt, no LED on the board was on. The 5 V output was not measured in this condition.
- Not done: the drain measurement. Kevin does it on 2026-11-14 with new cells.

Results on the 12 V input, 2026-11-13 09:49 to 10:45 (the same harness, a 12 V supply on the 6 V to 18 V input, switched by a plain Zigbee relay outlet, Hubitat device 435; raw results in `docs/data/x1202_bench_20261113/results_12v_input.jsonl`):

| Test | n | Result |
|---|---|---|
| Start when the input is applied | 1 | The Pi booted when Kevin connected the 12 V supply. |
| T1: input off for 20 s, Pi in operation | 3 | 3 of 3 continue on the cells. GPIO 6 falls 1.22 s to 1.25 s after `off` and rises 2.25 s to 2.35 s after `on`. EXT5V minimum 5.05 V. `get_throttled` 0x0. |
| T2: input off, `poweroff` after 10 s, input on 30 s after the network stops | 5 | 5 of 5 boot with no other action. The boot starts 12.0 s to 12.1 s after `on`. ssh answers after 21.6 s. |
| T3: input on 2 s after `poweroff` | 2 | 0 of 2 boot. |
| T3: input on 10 s after `poweroff` | 2 | 1 of 2 boot. |
| T3: input on 30 s after `poweroff` | 2 | 2 of 2 boot. |
| T4: `poweroff` with the input on | 1 | No start in 3 min. An input-off of 30 s starts the Pi. |
| T5: `reboot` with the input on, and on the cells | 3 and 1 | 4 of 4 come back. |
| T7: Pi halted with the input on, then input off for N s | 2 | N = 3.0 s: boots. N = 10 s: boots. |

The 12 V input gives the same behaviour as the USB Type-C input. The one difference is the delay of GPIO 6: 1.2 s after the loss and 2.3 s after the return, because the 12 V supply holds its output for a short time. All failures recovered with one input-off of 30 s.

### 7.6 Status of the tests in 7.2, and the go/no-go

| Test in 7.2 | Status | Result |
|---|---|---|
| Mechanical stack | Done | Correct. The NVMe carrier stays on top. |
| 1. Power cycles | Done without hands, 34 cycles on the USB Type-C input (one after 4 h with the input off) and 17 cycles on the 12 V input | Normal cycle: 10 of 10. Reboot: 4 of 4. Race at 2 s and at 10 s: the Pi stays off. Race at 30 s: 3 of 3. |
| 2. Drain and charge control | Partly done | GPIO 16 stops the charge while Linux drives it. The `gpio=16=op,dh` line in `config.txt` stops the charge during a boot (design document, finding 4). The charge with the Pi off and the input on stays. The boot with low cells is tested (2026-11-14): after VCELL reached 3.30 V and the Pi halted, the input returned and the Pi booted with no other action. The rested VCELL was 3.515 V, EXT5V was 5.147 V, `throttled` was `0x0` and there were no faults. The drain is measured (2026-11-15): 0.85 mA with the Pi off and the input off (design document, bench test 14). |
| 3. Full load on the cells at 50 C to 55 C | Passed at room temperature on 2026-11-14 (7.10, item 11); not done with warm cells | With the new cells, the jumper and the camera in operation: EXT5V is 5.08 V or more with 4 cores and an NVMe read. The limit is 4.9 V. |
| 4. Hub power from the board | Not done | Needs hands. |
| RTK fix quality | Not done | Needs the robot. |

Status, 2026-11-14 evening: the board with the new cells and the jumper passes each test that does not need hands (7.10). The watcher of `docs/x1202_power_watcher_design.md` is installed on the Pi on the test stand since 21:15 and passes its bench tests (section "Implementation and bench status" of that document). One new item: the system journal of a halted boot is lost on this board, because the clock starts at an old date at each boot. Before a go, these items stay open: the standby drain with a meter, the converter of the robot (3 A or more at 12 V), the stand test with the motors, and Kevin's decisions. Before a drive, the watcher must be installed.

Status, 2026-11-15 (night tests): the old watcher is retired. Design tests 6, 7, 11 and 12 pass. The `config.txt` line `gpio=16=op,dh` is installed (Kevin's decision). The clock unit is installed and keeps the system journal of a halted boot (design document, "Clock unit"). The charge hold (test 13) ran for 5.8 h: 14 bursts brought the rested VCELL from 3.585 V to 3.75 V in 56 min, and then one burst in 4.9 h kept it at 3.75 V to 3.77 V. Kevin turned the hold on (2026-11-15): the unit file starts the watcher with `--hold`. The standby drain is measured (Kevin, 2026-11-15): 0.85 mA with the Pi off and the input off. That is above the limit of 0.2 mA, and Kevin accepted a storage rule: more than 6 weeks parked, remove the cells. The 12 V now goes into the barrel jack (Kevin's decision, 2026-11-15): two power cycles and a charge burst with a full CPU load pass on the jack with the bench supply (design document, bench table). Before a go, these items stay open: the converter of the robot (3 A or more at 12 V), the stand test with the motors, the warm-cell test, and Kevin's remaining decisions (the size of the script, tier (a) with a GPIO 6 that cannot be read, the RTC cell).

Status, 2026-11-15 (stand test on the robot): the Pi with the X1202 is on the robot, and the converter of the robot (Kevin: set to approximately 12 V) feeds the barrel jack. Kevin drove the tracks on the stand for 326 s. The motors do not disturb the input: GPIO 6 had no low sample with the motors in operation and no charge burst. A charge burst with the usual load of the robot is correct (120 s, no low sample). A charge burst with a full CPU load is not: three times GPIO 6 went low for less than 0.25 s after 33 s to 38 s, and the watcher stopped the burst (no halt, no undervoltage). The cause is not known: the converter, the wiring to the jack, or the board. This is a fail for the pass condition of the stand test (design document, "Stand test on the robot"). The converter has a rating of 10 A and is on a 5 A fuse (Kevin), thus its rating is not the cause. Meter check, 15:05 to 15:18: the 12 V at the barrel plug was 12.14 V before the test and 12.03 V or more during two bursts with the full load (GPIO 6 went low after 26 s and 22 s). Thus the supply is not the cause, and the limit is in the board (probably a load limit: each low signal came at a CPU temperature of 74 C to 77 C, where the Pi takes its maximum power, and the signal is high again when the watcher stops the charge; not confirmed). A burst with the usual load directly after that was correct. Kevin accepted this limit on 2026-11-15, and the charge policy does not change. Robot checks, 15:46 to 16:11 (design document, "Robot checks after the install"): the main switch off and on, the main switch off for 3 s, the ch5 e-stop and the `/debug` data all pass. One finding: after an e-stop the Pi stayed off for 13.5 min with the main switch on, and the board charged the cells from 3.75 V to 3.92 V (operator rule 2). Kevin's decisions of 2026-11-15 close the remaining items: the watcher script stays as it is, tier (a) stays on with the `nohalt` file, there is no cell temperature rule for now, and the load test with warm cells moves to "before the summer of 2027".

Go/no-go, 2026-11-15: **go** for the X1202 with the `walle-power` watcher on the robot. Basis: the bench tests of the board and of the watcher, the stand test with the motors (with the accepted limit of a charge burst at a very high load), the robot checks after the install, and Kevin's decisions. The go/no-go of 2026-11-13 below is kept as history.

Conditions and follow-up work:

1. Before the first drive: hold the barrel plug in the jack (a cable tie or a right-angle plug). If the plug comes out, the Pi halts 5 s later and the robot stops.
2. This go is for the power board only. The open safety items in `CLAUDE.md` (for example the person stop hold with a person) are not part of it.
3. At the first drive outdoors: the RTK fix quality with the board in operation.
4. After 7 days with no fault: delete the old watcher code (push 2; Kevin's approval). DONE on 2026-11-15, commit `8037d3d`: Kevin did not wait the 7 days. The old files are in the git history (last commit `b0fc480`). The auto-deploy HOLD for that push, its test and the retire script were deleted on the same day (commit `e5bf928`).
5. Before the summer of 2027: the load test with warm cells (50 C to 55 C), and the decision on a cell temperature sensor.
6. When the RTC cell is in the Pi: add `dtparam=rtc_bbat_vchg=3000000` to `config.txt` (Kevin's word).
7. With no date: bench tests 2, 10 and 15 of the design document, the flat-cell LED check, and a charge burst with the usual load while the motors are in operation (an observation, not a test with a pass condition).

The operator rules stay in force (design document, section 4): rule 1 (the Pi does not start: main switch off, wait, on), rule 2 (after an e-stop, set the main switch to off), rule 3 (flat cells) and rule 4 (more than 6 weeks parked: remove the cells).

Go/no-go, 2026-11-13: **not yet**. No result rejects option F. The primary functions are correct: the Pi starts when the input returns, there is no poll wake, the power-loss signal is clean, and a reboot does not cut the power. The board does not go on the robot until these items are closed:

1. The cells. CLOSED on 2026-11-14: two Molicel IHR-18650C are in the board, and the pack resistance is approximately 0.04 ohm (7.10). The open item is now the 5 V path from the board to the Pi (7.10, items 2 and 3). Use two cells with a low internal resistance from a known manufacturer, or four cells. Then do T6 again. The vendor permits 2 to 4 cells. One cell is not permitted.
2. The stay-off condition (findings 1 and 2). Kevin decided on 2026-11-13 that the debounce on GPIO 6 is 5 s, as in the old watcher. The remaining choice is Kevin's decision. Option 1: accept it, with an operator rule: "If the Pi does not start, set the main switch to off, wait 5 s, and set it to on." A debounce of 10 s on GPIO 6 before the halt prevents the condition for a short interruption. Option 2: add a check of GPIO 6 at the end of the shutdown that changes the power-off into a reboot when the input is back. This needs a shutdown hook, and Kevin must approve a bench test of it. The hook cannot close the last seconds before the board cuts its output.
3. The charge level. The software cannot hold the cells at 3.8 V in all conditions (finding 3). Each boot charges for approximately 13 s. A halt on the cells discharges for approximately 20 s. With the Pi off and the main switch on, the board charges to 4.2 V. Possible corrections, not tested: a `gpio=16=op,dh` line in `config.txt` (a shorter charge at each boot), and a longer time on the cells before the halt when VCELL is above the target. Update, 2026-11-15: the `config.txt` line is now tested and installed. What stays: the charge with the Pi off and the input on. The hold is on since 2026-11-15 (Kevin's decision): the watcher keeps the rested VCELL near 3.75 V while the Pi runs.
4. The storage drain. CLOSED on 2026-11-15 with an operator rule: the drain is 0.85 mA (approximately 20 mAh in one day; approximately 3 months from 3.75 V to empty cells), and Kevin accepted the rule "more than 6 weeks parked: remove the cells". Owner reports say that the X1202 has no hardware cutoff for a low cell voltage (7.8). If the drain with the output off is in the milliampere range, months of storage can damage the cells. The soak of 4 h (finding 14) does not answer this. Measure the drain with a meter (7.7), with good cells.
5. The tests with hands in 7.7.
6. A plain relay outlet for the bench supply, not a dimmer.
7. The 6 V to 18 V input. DONE on 2026-11-13 for the power cycles (7.5, "Results on the 12 V input"): the same behaviour as on the USB Type-C input. Still open on this input: the load test with new cells, and the robot's own converter as the supply. Never connect the two inputs at the same time.

The software that replaces `bin/walle_ups.py` is described in `docs/x1202_power_watcher_design.md`. Grok and an independent Opus reviewer examined it in three rounds. The watcher is installed on the Pi on the test stand since 2026-11-14 (Kevin's approval), and the old watcher is retired. The design adds two items to this list: set the robot's converter from 7.5 V to approximately 12 V before the X1202 goes on the robot, and there is no cell temperature, thus the condition "charge disabled above 45 C" needs a sensor on the cells.

### 7.7 Bench procedure for the tests that need hands

Kevin does the physical steps. The harness and the logger continue to record.

1. Record the brand, the model and the age of the cells. Record the voltage and the current rating of the supply.
2. The 5 V after a halt. Done on 2026-11-13 for a halt with the 12 V input present: 0 V. Not measured yet for a halt with the input off. Connect a voltmeter to a USB Type-A socket of the X1202. Remove the input and halt the Pi (`halt-off` in the harness). Read the voltage 15 s after the Pi stops. The correct value is 0 V. Do the same with the input present (`sudo poweroff`).
3. The drain. Halt the Pi and remove the input. Remove one cell. Put an ammeter between the positive end of the other cell and its holder contact. Start on the 10 A range, then go to the microampere range. Record the current after 1 min.
4. The load test with good cells. Do T6 again (`t6` in the harness). Then make the cells warm (50 C to 55 C) and do T6 again. Record the cell temperature during a charge.
5. The 5 V outputs (test 4 in 7.2). Connect the hub, or the camera with the cut 5 V wire, to an output of the board. Check that the output goes off after a halt, that the voltage stays at 4.9 V or more with the camera in operation, and that the total current stays below 5 A.
6. The low-cell boot. Discharge the cells to approximately 3.3 V (Pi in operation, input off, logger guard on). Halt the Pi. Apply the input. The Pi must boot.

The harness commands (the outlet URL is the home dashboard URL of Hubitat device 98):

```bash
python3 tools/x1202_cycle_test.py --pi pi@192.168.86.54 --outlet-url <outlet-url> --results <file> status
python3 tools/x1202_cycle_test.py --pi pi@192.168.86.54 --outlet-url <outlet-url> --results <file> t6
python3 tools/x1202_cycle_test.py --pi pi@192.168.86.54 --outlet-url <outlet-url> --results <file> halt-off
python3 tools/x1202_cycle_test.py --pi pi@192.168.86.54 --outlet-url <outlet-url> --results <file> on-wait
```

The bench logger is a temporary service on the Pi (`x1202-bench-logger`). It operates only while the file `/home/pi/x1202_bench/ENABLED` is there. It holds GPIO 16 high (charge disabled). Its guard halts the Pi when GPIO 6 is low for 180 s, or when VCELL is 3.30 V or less for 5 s with GPIO 6 low. WARNING: remove this service before the Pi goes back on the robot:

```bash
sudo systemctl disable --now x1202-bench-logger && sudo rm /etc/systemd/system/x1202-bench-logger.service /home/pi/x1202_bench/ENABLED
```

NOTE: `walle-ups.service` is still installed on the Pi. Its sensor (the INA219 of the HAT (B)) is not there, thus it records I2C errors and it does not halt the Pi. The I2C bus guard has no witness in this condition and does not recover a locked bus. Do not put the Pi on the robot in this condition.

### 7.9 Test plan with the new cells, on the robot (prepared 2026-11-14)

Condition: Kevin puts the Pi with the X1202 and the new cells back on the robot. The robot's converter feeds the board through the main switch. A camera shows the voltage and the current of that supply. There is no remote outlet: Kevin operates the main switch, and the script reads GPIO 6 to see each change. The robot stays on its stand. It is not armed.

WARNING: the Pi still has the bench logger, the old watcher (which does nothing on this board) and an I2C bus guard with no witness. This is acceptable for tests on the stand. It is not the condition for a drive: on an input loss the Pi halts only after 180 s, and a locked I2C bus does not recover.

Kevin gives these facts before the start: the brand, the model and the number of the new cells; the voltage of the converter; which loads are on the 5 V outputs of the board (the hub, the camera, or nothing); which camera shows the supply.

Stage 0, no hands (5 min): the Pi answers; the logger operates; VCELL of the new cells; the supply voltage and current from the camera at idle; EXT5V; `get_throttled`; the USB devices; the RTK GPS and the fuel gauge on I2C bus 1.

Stage 1, a cold start with low cells (only if the new cells are below 3.45 V at the start): halt the Pi, set the main switch to off for 30 s, set it to on. The Pi must boot. Do this stage before the first charge, because new cells come with a low charge and this condition does not come back.

Stage 2, the first charge, with the camera (no hands): set GPIO 16 low. Read the supply current from the camera each minute. Record VCELL. Kevin feels the cells for heat one time. Stop the charge at a rested VCELL of 3.80 V. Results: the charge current, the supply current with the Pi and the charge together (the converter must carry it), and the pack resistance from the VCELL step when the charge stops.

Stage 3, the load test on the cells (the test that the old cells did not pass): Kevin sets the main switch to off, and to on again after 6 min. The script starts when GPIO 6 goes low. It adds 1, 2 and 4 busy cores and an NVMe read, 60 s for each step, with the camera of the robot in operation if the board feeds it. Pass: EXT5V is 4.9 V or more in each step, and `get_throttled` stays 0x0. The script stops the load if VCELL is below 3.35 V or EXT5V is below 4.95 V.

Stage 4, main-switch cycles, 3 times: Kevin sets the main switch to off. The script halts the Pi 5 s after the loss (Kevin's debounce time). Kevin waits until the Pi is dark, counts to 30, and sets the switch to on. Pass: the Pi boots each time with no other action.

Stage 5, the meter readings (Kevin): the 5 V output after a halt with the main switch off (0 V is correct), and the standby drain (7.7, step 3). The drain is a go/no-go item.

Stage 6, the stand test with the motors (Kevin drives the tracks on the stand for 5 min; the script only records): GPIO 6 must have no low sample; EXT5V and the supply voltage on the camera must stay stable under motor load and during regeneration.

Later, not on this day: the load test with warm cells (50 C to 55 C), and the RTK fix quality outdoors with the board in operation.

Go: stages 3, 4 and 6 pass, the drain in stage 5 is in the microampere range (or Kevin accepts a storage rule), and Kevin accepts the items in 7.6 that the tests cannot change (the stay-off condition, the charge during a boot, no cell temperature). After a go, the next step is the watcher of `docs/x1202_power_watcher_design.md`. The board is not ready for a drive before that watcher is installed.

### 7.10 Results with the new cells, on the robot stack (2026-11-14)

Setup: the Pi with the X1202 is on the robot, on the stand, not armed. The cells are two Molicel IHR-18650C (high-rate cells, approximately 2 Ah each). A bench supply (0 V to 120 V, 0 A to 3 A, set to 12.2 V) feeds the 6 V to 18 V input through the Zigbee relay outlet (Hubitat device 435). WARNING: this supply can give 120 V, and the limit of the board is 18 V. A telephone camera shows the display of the supply. There is no USB hub: the OAK camera and the Arduino are on the USB ports of the Pi. Raw data: `docs/data/x1202_robot_20261114/`.

1. The USB limit. With the default configuration, the Pi limits its USB ports to 600 mA in total, because the X1202 does not negotiate USB power delivery. Result: one over-current event at the boot and 42 USB disconnects in 4 min; the camera and the serial link of the Arduino did not stay connected. With `usb_max_current_enable=1` in `/boot/firmware/config.txt` (added on 2026-11-14, backup `config.txt.bak-20261114-usbcurrent`), the limit is 1.6 A: no over-current event and one disconnect in 67 min.
2. The 5 V at the Pi with the camera on a port of the Pi. The kernel reports an undervoltage two times while the camera starts. EXT5V is 4.97 V at idle (5.12 V to 5.15 V without USB loads). With a CPU load on the 12 V input: 4.89 V with 1 busy core (minimum 4.76 V), and 4.86 V with 2 busy cores with the undervoltage flag on. The test stopped there. The supply stayed at 12.20 V in constant-voltage mode, GPIO 6 stayed high, and VCELL did not move. Thus the supply and the cells are not the cause.
3. The cause is the path from the 5 V of the board to the Pi (the pogo pins), approximately 0.2 ohm. Evidence: the same decrease of EXT5V for each watt occurred on the USB Type-C input, on the 12 V input and on the cells; and on 2026-11-13 the 5 V output of the board was 5.24 V on the meter while the Pi read 5.12 V to 5.15 V. CORRECTION to finding 8 of 7.5: the low EXT5V in the load test of 2026-11-12 came mostly from this path, not from the old cells. Not confirmed with a meter during a load yet.
4. The cells. Ride-through: VCELL goes from 4.186 V to 4.125 V when the input stops (the old cells: 0.20 V). With the full load of the robot application and the camera, VCELL is 0.10 V to 0.14 V below the rested value. The pack resistance is approximately 0.04 ohm (the old cells: 0.23 ohm).
5. Time on the cells. From a full charge (4.17 V rested), the cells carried the Pi, the NVMe drive, the camera, the Arduino and the robot application for 28.0 min until VCELL was 3.72 V under load (3.85 V rested). EXT5V on the cells: median 4.97 V, minimum 4.86 V, no undervoltage flag in 105 samples. Operation on the cells gives the same EXT5V as operation on the input.
6. The charge. With GPIO 16 low for 185 s, the supply current increased from 0.89 A to between 2.2 A and 2.4 A at 12.20 V (10.8 W to between 27 W and 29 W). The charge current into the cells is approximately 3 A (calculated, not measured). VCELL was 3.99 V to 4.03 V during the charge and 3.91 V one second after it. The rested VCELL increased from 3.854 V to 3.893 V. EXT5V decreased to 4.85 V at some moments during the charge.
7. The converter of the robot. The Pi, the camera and a charge together need approximately 29 W. With a full CPU load, estimate 34 W. The converter must supply 3 A or more at 12 V. At 7.5 V the same power is more than 4.5 A.
8. Power cycles with the camera and the Arduino connected: 3 of 3 boot with no other action.
9. I2C bus 1: the fuel gauge (0x36) and the RTK GPS (0x20) answer together.

10. The meter confirms item 3 (Kevin, 2026-11-14 18:05, Pi at idle with the camera in operation): 5.2 V at the 5 V output of the X1202, and 4.95 V on the 5 V rail of the Pi (measured at the screw terminals of the top HAT, the same value as EXT5V). The loss is 0.25 V in the pogo-pin path.
11. The jumper (Kevin, 2026-11-14 19:20): two wires from the 5 V output of the X1202 to the 5 V and ground screw terminals of the top HAT. This is a second path in parallel with the pogo pins. Results with the jumper, the camera still on a USB port of the Pi:

| Load | EXT5V before the jumper (12 V input) | EXT5V with the jumper (12 V input) | EXT5V with the jumper (on the cells, minimum) |
|---|---|---|---|
| Idle | 4.97 V | 5.16 V | 5.14 V |
| 1 busy core | 4.89 V | 5.14 V | 5.12 V |
| 2 busy cores | 4.86 V, undervoltage | 5.12 V | 5.10 V |
| 4 busy cores | not done | 5.10 V | 5.09 V |
| 4 busy cores and an NVMe read | not done | 5.10 V (minimum 5.06 V) | 5.08 V |

   `get_throttled` stays 0x0 from the boot, also while the camera starts. The load test on the cells (T6) is passed at room temperature: the limit is 4.9 V. VCELL goes from 3.94 V rested to 3.69 V in the last step. This was also the first run of the corrected T6 cleanup on the hardware.
12. The halt and the start with the jumper: 2 of 2 power cycles boot with no other action (the boot starts 12.1 s to 12.3 s after the input returns); after a halt with the input present the Pi stays off, and an input-off of 30 s starts it; an input-off of 3.0 s starts a halted Pi. Thus the 5 V output of the board goes off together with the pogo-pin supply, and the jumper does not keep the Pi on after a halt. Not measured yet: the temperature of the 2-pin connector of the board under load, and the standby drain with the jumper.

Consequences:

- The cells are no longer a go/no-go item. The low EXT5V under load is the item: the Pi cannot supply the camera through the pogo pins of this board. Corrections: supply the camera from a 5 V output of the board (the cable that Kevin planned), or add a second 5 V path from a 5 V output of the board to the 5 V pins of the header. DONE on 2026-11-14 with the second path (item 11): the load test passes, and the camera stays on a USB port of the Pi with a standard cable. The jumper is now a necessary part of the installation. A Grok second opinion (session cdb2a75e) agreed with the second path, and gave the conditions that items 11 and 12 examine.
- `usb_max_current_enable=1` stays in `config.txt` while a USB device takes its power from the Pi. To remove it, delete the last three lines of the file or restore the backup.
- The harness aborts only on an undervoltage that is present, not on the history flag. This change is in the work tree and is not committed yet, because a commit under `tools/` restarts `wall-e.service` on the robot.

### 7.8 Facts from the vendor and from other owners (read 2026-11-12)

These facts are not from our tests. "Reported" means one owner report.

- The manufacturer (SupTronics) requires 2 to 4 cells, unprotected, with a flat top. The cells are in parallel.
- The board finds the Pi through the 3.3 V rail on a pogo pin. With `POWER_OFF_ON_HALT=1` it cuts its output approximately 3 s after the halt.
- The charger is probably an Injoinic IP2312 (reported). This part is a charger only. It has an input for a temperature sensor, but no page says that the board has a sensor. Assume that there is no protection for the cell temperature.
- No hardware cutoff voltage is published. One X1202 owner says that the board does not go off below 3.4 V (reported). On a larger board of the same family, the cells went down to 2.54 V and were destroyed (reported). The only vendor cutoff is a script at 3.20 V.
- Connectors (vendor wiki, read 2026-11-15): the 6 V to 18 V input is a DC 5521 jack and an "XH2.54-2P" socket; the two 5 V outputs are "XH2.54 2-pin" sockets; the external button is an "XH2.54-2P" socket. No current rating is given for one socket; the output of the board is 5.1 V, 5 A maximum. WARNING: the 12 V input socket, the 5 V output sockets and the button socket take the same plug. A 12 V lead in a 5 V socket puts 12 V on the 5 V rail of the Pi.
- No standby current is published. A larger board of the same family used approximately 13 mW with the charger idle (reported).
- An owner of the X1201 reports the stay-off condition of finding 1: "the Raspberry Pi remains powered off".
- The vendor scripts and the third-party drivers stop the charge with the internal pull-up of the Pi. On our board this does not work (finding 4). One X1202 owner reports that a high GPIO 16 did not stop the charge on his board. Our board stops.
- The vendor script halts the Pi 2 min to 3 min after the input fails. It has no debounce in seconds.
- Owner reports of bad pogo contact: GPIO 6 read "good" until the Pi was pushed down, and one board had no I2C. One owner reports undervoltage (4.9 V to 5.05 V) under load.
- The fuel gauge needs a quick-start (0x4000 to register 0x06) after 30 min with no cell current. The vendor scripts do not send it.

Pages: `suptronics.com/Raspberrypi/Power_mgmt/x1202-v1.1.html`, `suptronics.com/Raspberrypi/Power_mgmt/x1202-v1.1_hardware.html`, `wiki.geekworm.com/X120X_Software`, `wiki.geekworm.com/Template:X120x-FAQ`, `github.com/suptronics/x120x`, `github.com/mor-lock/x120x-dkms` (issues 5 and 6, incident notes), `github.com/ArjenR/x120x_upsd`, `geekworm.com/community/forum/topic/200327/x1202-raspi-does-not-switch-back-on`, the IP2312 data sheet.

## Sources

Raspberry Pi documentation (`POWER_OFF_ON_HALT`, `WAKE_ON_GPIO`, power
supplies); CM5 datasheet RP-008180 (PWR_BUT, standby current); Jeff
Geerling, "Reducing Raspberry Pi 5's power consumption 140x"; UUGear Witty
Pi 5 user manual; AQEX qUPS and Geekworm UPS_Scap product pages; Pololu
D42V55F5 and pushbutton switch pages; TI LM5013 and LM5164 datasheets; Daly
smart BMS control strategy; Clearpath Jackal, Husarion ROSbot and Duckietown
manuals; Raspberry Pi forum t=388108; Volumio forum (Pi 5 slow ramp).

## Appendix: Technical Names

Raspberry Pi 5, PMIC, RTC, J2, PWR_BUT, `POWER_OFF_ON_HALT`, `WAKE_ON_GPIO`,
3V3, GPIO, Waveshare UPS HAT (B), Witty Pi 5, UUGear, AQEX qUPS, Geekworm
UPS_Scap, Pololu D42V55F5, TI LM5013, LM5164, TPL5110, Daly BMS, VESC,
OAK-D, 13S, 2S, 18650, supercap, ESR, TVS, opto, crowbar, buck, ENABLE,
overlayfs, read-only root, Geekworm X1202, SupTronics, MAX17040, Injoinic IP2312, VCELL, EXT5V, `get_throttled`, `pinctrl`, `poweroff`, `reboot`, ssh, ARP, Z-Wave, Hubitat, NVMe, SoC, ext4, pogo pin, pull-up, pull-down, debounce, harness, logger, Clearpath Jackal, Husarion ROSbot, Duckietown,
TRACO TEN 30-4811WIN, TEN 20-4811WIN, RECOM RP30-4805SFW/N, Mean Well RSD-60L-5,
SMBJ58A, LTC4368, P-FET, VESC Tool, Trampa, Grok, Kevin, WALL-E.
