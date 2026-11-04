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
with a TVS: a TVS that stays off at 54.6 V clamps near 90 V. Use a module
rated 75 to 100 V continuous that gives 5 V at 5 A after derating at 50 V,
with a measured off-state current at 55 V, an output fuse and a crowbar.
The TI LM5013 (100 V) is 3.5 A, non-synchronous, and too small for a Pi 5
with the OAK-D. Not chosen yet; see question 1.

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

1. Which converter module: a bought 75 to 100 V module (to be found), or
   wait for the custom board?
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
Grok, Kevin, WALL-E.
