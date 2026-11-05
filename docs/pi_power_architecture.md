# Pi power for long storage: options and recommendation

Status: decision draft, 2026-11-04. Research by three agents (off-the-shelf
boards, how robot builders do it, an always-on tap from the main pack) and a
Grok second opinion (session 01a1a8bd). Nothing is bought.

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
| A. Gated tap on the main pack (recommended) | Converter off-current plus BMS: about a year on 20 Ah | The main pack | Needs a 75 to 100 V module and a hardware force-off timer (section 4) |
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

Storage estimate (agent, 20 Ah pack at 50 percent, BMS asleep 0.5 mA, cell
leakage about 2 percent a month): about 8 months with the converter idling,
about 12 months with it off. A Daly BMS sleeps only after 3600 s idle.

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
   section 5). Open: set the regen cutoff near 55.5/57 V on both, and set
   `si_battery_cells` to 13 on the right VESC (VESC Tool, Kevin).
2. Is there a point on the robot where the BMS pack output is available on
   the battery side of the main switch?
3. Should the robot ever wake on a schedule while stored? Option A cannot,
   because nothing has 5 V while it is off.

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
overlayfs, read-only root, Clearpath Jackal, Husarion ROSbot, Duckietown,
TRACO TEN 30-4811WIN, TEN 20-4811WIN, RECOM RP30-4805SFW/N, Mean Well RSD-60L-5,
SMBJ58A, LTC4368, P-FET, VESC Tool, Trampa, Grok, Kevin, WALL-E.
