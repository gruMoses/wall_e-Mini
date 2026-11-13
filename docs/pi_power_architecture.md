# Pi power for long storage: options and recommendation

Status: decision draft, 2026-11-04. Research by three agents (off-the-shelf
boards, how robot builders do it, an always-on tap from the main pack) and a
Grok second opinion (session 01a1a8bd). Nothing is bought.

UPDATE 2026-11-13: the first bench results of the X1202 are in 7.5 to 7.8.

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

Also check first: the mechanical stack (the X1202 goes under the Pi; find out where the NVMe board is), and the RTK fix quality with the board in operation.

### 7.3 If option F fails on the bench

- Option A in its minimum form: a converter with an enable input on the battery side of the main switch, held on by "switched side alive OR the Pi's 3V3", a halt script, the Pi's hardware watchdog, and a bleed on the 5 V rail. The main pack is the only battery. This is a small quantity of hand-wired parts at pack voltage, not a PCB. Its costs: the regen overvoltage conditions of the spec (3.2a) apply, and a BMS cut is a hard cut for the Pi.
- Supercapacitors: 2 x 100 F in series give about 670 J down to 1.5 V. A 60 s halt at 6 W needs about 425 J with the converter loss. The halt must then be short: `wall-e.service` permits 90 s today, and a supercapacitor design needs a kill after about 5 s and about 15 s in total. Few bought boards of this type operate a Pi 5 and start it automatically (the Geekworm UPS Scap was out of stock and does not state it). Use 2.5 V for each cell in the heat.

### 7.4 Second opinions

Grok timed out after 900 s on this question. An independent Opus review agreed with option F and against the custom board ("215 USD and 31 risks to replace a 49 USD board"). It added the race test, the fail-safe for GPIO 16, the heat check, the mechanical check and the retirement of the old watcher. A research agent ranked the minimum option A first and warned that a UPS board "keeps the small-battery design that currently fails"; the answer to that warning is that the present failure comes from the HAT (B)'s missing input sense and output switch, which the X1202 has.

### 7.5 Bench results, 2026-11-12 and 2026-11-13 (tests without hands)

Setup: the robot's Pi 5 is on a bench, off the robot. It is on one Geekworm X1202 with two Moli ICR-18650H cells (identified from a photograph on 2026-11-13: "MOLI ... 06 ICR-18650H MADE IN CANADA"). These are LiCoO2 laptop cells of 2200 mAh when new, with a date code that is probably 2006. They are old cells. The NVMe carrier stays on top of the Pi, and the root file system is on the NVMe drive. Thus the mechanical stack is correct: the X1202 is below the Pi and the NVMe carrier is above it. CORRECTION, 2026-11-13 (Kevin): the supply is the Raspberry Pi USB Type-C supply on the USB Type-C input of the board (5 V; Kevin thinks 3 A). It is not a 12 V supply. The first issue of this section said 12 V; that was an assumption and it was wrong. All results in 7.5 are for the USB Type-C input. The robot uses the 6 V to 18 V input, and that input is not tested yet. The vendor requires 5 A or more on the USB Type-C input. A 3 A supply cannot give the Pi load and the full charge current at the same time. This is a possible cause of the VCELL step at 21:44 and of the EXT5V minimum of 5.05 V during a charge. A Z-Wave dimmer outlet (Hubitat device 98, "Christmas Tree") switches the supply. Kevin was not at the bench. The logger `tools/x1202_bench_logger.py` records GPIO 6, GPIO 16, VCELL, EXT5V, `get_throttled` and the SoC temperature at 4 Hz. The harness `tools/x1202_cycle_test.py` operates the outlet and the Pi. The raw results are in `docs/data/x1202_bench_20261113/results.jsonl`. The logs of the Pi are in `/home/pi/x1202_bench/logs/` (one file for each boot).

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

### 7.6 Status of the tests in 7.2, and the go/no-go

| Test in 7.2 | Status | Result |
|---|---|---|
| Mechanical stack | Done | Correct. The NVMe carrier stays on top. |
| 1. Power cycles | Done without hands, 34 cycles (one after 4 h with the input off) | Normal cycle: 10 of 10. Reboot: 4 of 4. Race at 2 s and at 10 s: the Pi stays off. Race at 30 s: 3 of 3. |
| 2. Drain and charge control | Partly done | GPIO 16 stops the charge while Linux drives it. It cannot stop the charge during a boot or with the Pi off. The drain is not measured. The boot with the cells at 3.3 V is not tested. |
| 3. Full load on the cells at 50 C to 55 C | Partly done, at room temperature | Not passed with these cells: EXT5V is 4.896 V with 4 cores and an NVMe read. The limit is 4.9 V. |
| 4. Hub power from the board | Not done | Needs hands. |
| RTK fix quality | Not done | Needs the robot. |

Go/no-go, 2026-11-13: **not yet**. No result rejects option F. The primary functions are correct: the Pi starts when the input returns, there is no poll wake, the power-loss signal is clean, and a reboot does not cut the power. The board does not go on the robot until these items are closed:

1. The cells. The pack resistance is too high (finding 8). The cells are old Moli ICR-18650H laptop cells (7.5, setup). They are sufficient for function tests on the bench with the charge disabled. Do not put them on the robot. Use two cells with a low internal resistance from a known manufacturer, or four cells. Then do T6 again. The vendor permits 2 to 4 cells. One cell is not permitted.
2. The stay-off condition (findings 1 and 2). This is Kevin's decision. Option 1: accept it, with an operator rule: "If the Pi does not start, set the main switch to off, wait 5 s, and set it to on." A debounce of 10 s on GPIO 6 before the halt prevents the condition for a short interruption. Option 2: add a check of GPIO 6 at the end of the shutdown that changes the power-off into a reboot when the input is back. This needs a shutdown hook, and Kevin must approve a bench test of it. The hook cannot close the last seconds before the board cuts its output.
3. The charge level. The software cannot hold the cells at 3.8 V in all conditions (finding 3). Each boot charges for approximately 13 s. A halt on the cells discharges for approximately 20 s. With the Pi off and the main switch on, the board charges to 4.2 V. Possible corrections, not tested: a `gpio=16=op,dh` line in `config.txt` (a shorter charge at each boot), and a longer time on the cells before the halt when VCELL is above the target.
4. The storage drain. Owner reports say that the X1202 has no hardware cutoff for a low cell voltage (7.8). If the drain with the output off is in the milliampere range, months of storage can damage the cells. The soak of 4 h (finding 14) does not answer this. Measure the drain with a meter (7.7), with good cells.
5. The tests with hands in 7.7.
6. A plain relay outlet for the bench supply, not a dimmer.
7. The 6 V to 18 V input. The robot uses this input, and all tests so far used the USB Type-C input. Do T1, T2, T3, T4 and T7 again with a 12 V supply of 3 A or more on the DC input. Never connect the two inputs at the same time.

A draft design for the software that replaces `bin/walle_ups.py`, with the install procedure, is in `docs/x1202_power_watcher_design.md`. Grok and an independent Opus reviewer examined it in three rounds. It is not approved, and nothing from it is installed. It adds two items to this list: set the robot's converter from 7.5 V to approximately 12 V before the X1202 goes on the robot, and there is no cell temperature, thus the condition "charge disabled above 45 C" needs a sensor on the cells.

### 7.7 Bench procedure for the tests that need hands

Kevin does the physical steps. The harness and the logger continue to record.

1. Record the brand, the model and the age of the cells. Record the voltage and the current rating of the supply.
2. The 5 V after a halt. Connect a voltmeter to a USB Type-A socket of the X1202. Remove the input and halt the Pi (`halt-off` in the harness). Read the voltage 15 s after the Pi stops. The correct value is 0 V. Do the same with the input present (`sudo poweroff`).
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

### 7.8 Facts from the vendor and from other owners (read 2026-11-12)

These facts are not from our tests. "Reported" means one owner report.

- The manufacturer (SupTronics) requires 2 to 4 cells, unprotected, with a flat top. The cells are in parallel.
- The board finds the Pi through the 3.3 V rail on a pogo pin. With `POWER_OFF_ON_HALT=1` it cuts its output approximately 3 s after the halt.
- The charger is probably an Injoinic IP2312 (reported). This part is a charger only. It has an input for a temperature sensor, but no page says that the board has a sensor. Assume that there is no protection for the cell temperature.
- No hardware cutoff voltage is published. One X1202 owner says that the board does not go off below 3.4 V (reported). On a larger board of the same family, the cells went down to 2.54 V and were destroyed (reported). The only vendor cutoff is a script at 3.20 V.
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
