# Pi 5 power module: hardware design specification

Status: research draft, 2026-11-04. Nothing is ordered. No part is
sampled. Each part number below was checked on the manufacturer's page
or a distributor page on 2026-11-04 unless the text says VERIFY. A VERIFY
mark means the number or the status comes from memory or from a
third-party listing, and Kevin must check it in the datasheet before he
buys the part.

This document uses Simplified Technical English as far as is reasonable.
The appendix lists the Technical Names.

## 0. Summary

The module takes the robot's 13S traction pack (37.7 V to 54.6 V) at one
XT60 input and makes four things from it:

1. A 14.0 V rail, 150 W, with two switched outputs that the Pi controls.
2. A 5.1 V rail for the Raspberry Pi 5 through the GPIO header, 6 A.
3. A 5.0 V rail for the USB devices, with four current-limited ports.
   The Pi's own USB ports do not carry the device power.
4. A 2S 18650 buffer pack that carries the Pi through a clean halt when
   the input is lost. The pack floats at 7.60 V (3.80 V per cell). The
   module cuts the Pi's 5 V after the halt and applies it again when the
   input returns. The Pi then boots. No RTC polling is necessary.

A small microcontroller (STM32G071) runs the state machine. It talks to
the Pi over UART0 (GPIO 14 and 15) and two GPIO lines. It is not on the
Pi's I2C bus 1.

Section 11 (the Grok appendix) records the three second opinions and
what changed because of them. The first spin uses a bought 150 W DC-DC
brick for the 14 V rail. A discrete 100 V buck is the second spin.

## 1. Requirements

Assumptions are marked (A). Kevin's words are quoted.

| # | Requirement | Source |
|---|---|---|
| R1 | Input: 13S Li-ion pack, 37.7 V (BMS cut) to 54.6 V (full). Design working input 60 V. | Kevin: "up to 60 V straight from the battery". `config.py` lines 225-242. |
| R2 | Input survives a hot-plug ring above the pack voltage, VESC regen and load transients, and reverse polarity. | (A) from the two VESCs on the pack and the XT connector. |
| R3 | 14.0 V rail, 150 W. Two switched outputs, each 10 A peak. 8 A combined continuous while the Pi runs. | Kevin: "a tap for 10 A at 14 V", "a relay to control the 12V output, or better yet, two". (A) the 14 V tap and the 12 V output are the same rail. See open question 1 and 2. |
| R4 | 5.1 V rail for the Pi 5, 6 A continuous, through the header. | (A) Pi 5 needs 5 V 5 A to lift its USB limit. |
| R5 | 5.0 V rail for USB devices, 4 ports, each current-limited, no backfeed into the Pi. | Kevin: "power everything we need to power over USB". `docs/NEXT_SESSION.md`: 167 USB over-current trips from hub backfeed. |
| R6 | 2S 18650 buffer with a charger that floats at 7.60 V, a protector, balancing that works, and a pack monitor. | Kevin: "whatever we need for a custom BMS with two 18650s", 3.80 V per cell. |
| R7 | Input loss is detected on the 60 V side, not from battery current. | Lesson from `docs/ups_hat_b.md` section 4. |
| R8 | On input loss: request a clean halt, wait for the halt, cut the Pi's 5 V, then idle. On input return: apply 5 V, the Pi boots. | Kevin: "safely shut down the Pi", "powers off the Pi and probably the HAT", "the Pi should boot automatically". |
| R9 | A Pi that does not halt is cut after a timeout (120 s). | (A) field recovery over data safety. See risk R-7. |
| R10 | The module interface cannot hang the Pi's I2C bus 1. | `docs/i2c_bus_guard.md`. |
| R11 | Idle current from the buffer pack while the input is absent: under 0.3 mA. | (A) weeks of storage without a flat pack. |
| R12 | The module tells the Pi its state, voltages and currents, and accepts output commands. | (A) replaces `bin/walle_ups.py`. |
| R13 | The module's safe state with a dead microcontroller: charger at 7.60 V, 5 V on when the input is present, switched outputs off. | Grok objection, section 11. |
| R14 | Outdoor enclosure, ambient -10 C to +45 C, no forced air. | (A) WALL-E lives outside in Texas. |
| R15 | One person hand-assembles the first boards. Four-layer PCB, stencil, hot plate or oven. | Kevin builds it himself. |

Not requirements: galvanic isolation of the 14 V rail (the pack
negative is the common ground), ride-through of the robot's loads,
USB Power Delivery to the Pi.

## 2. Block diagram

```
 XT60 in          +-----------+   +---------+
 37.7-54.6 V ---->| F1 fuse   |-->| Surge   |---+---> VIN_OK
 (60 V work.)     | TVS, RPP  |   | stopper |   |     (divider +
                  +-----------+   | LT4363  |   |      comparator)
                                  +---------+   |
                                                v
                                     +-------------------+
                                     | 14 V converter    |
                                     | TEP 150-4815WI    |
                                     | trim 14.0 V, 150 W|
                                     +---------+---------+
                                               | 14 V rail
            +------------------+---------------+-------------+
            |                  |                             |
   +--------v-------+  +-------v--------+          +---------v--------+
   | OUT1 / OUT2    |  | Charger        |          | Ideal diode      |
   | TPS1HB08-Q1 x2 |  | BQ24610        |          | LTC4359 (SHDN)   |
   | 10 A peak each |  | CV 7.60/8.40 V |          +---------+--------+
   +----------------+  | CC 1.0 A       |                    |
                       +-------+--------+                    | SYS bus
                               |                 +-----------+------+
                    +----------v----------+      |                  |
                    | 2S pack             |      |      +-----------v------+
                    | protector S-8252    +------+      | 5.1 V Pi buck    |
                    | balancers at 4.20 V | ideal       | LM61495, 6 A     |
                    | INA228 monitor      | diode       +--------+---------+
                    +---------------------+ LM74700              |
                                                        +--------v---------+
                                                        | eFuse TPS25982   |
                                                        | + discharge FET  |
                                                        +--------+---------+
                                                                 | header
                                                                 v pins 2,4
                    +------------------+      +---------------+  Pi 5
      SYS bus ----->| 5.0 V USB buck   |----->| TPS25221 x4   |--> OAK, hub,
                    | LM61495, 5 A     |      | per-port limit|    Arduino,
                    +------------------+      +---------------+    spare
                                                     (USB 3 injectors
                                                      pass data through,
                                                      Pi-side VBUS open)

      MCU STM32G071: VIN_OK, 3V3 sense, UART0, SHUTDOWN_REQ,
      J2 PWR_BUT pulse, OUT1/OUT2 enable, port enables,
      charger top-up, INA228 (own I2C), LED, button.
```

## 3. Blocks and parts

Each block gives a recommended part and one alternative. Prices are
single-unit distributor estimates (section 7).

### 3.1 Input protection and surge stopper

Problem. A 54 V pack hot-plugged through an XT60 into the module's input
capacitors rings above the pack voltage. The VESCs push the bus up on
regen. A hot-swap controller that opens on overvoltage drops the Pi
during a brake event. A TVS that stays off at 60 V clamps near 94 V,
which is above 100 V parts after derating (Grok, section 11.1).

Design. A surge stopper clamps the output to a set voltage during a
surge and keeps the loads powered. It also limits inrush and current,
and it blocks reverse polarity. The pass FET is rated 150 V so that the
TVS can have a 70 V standoff.

| Item | Part | Key ratings |
|---|---|---|
| Surge stopper | ADI LT4363-2 (auto-retry) | 4 V to 80 V operating. Overvoltage, reverse input, current limit with foldback, circuit breaker, UV lockout. SOIC-16 or MSOP. Datasheet: https://www.analog.com/en/products/lt4363.html . VERIFY: output clamp set point, reverse input limit (-60 V from memory), VIN pin series resistor for transients above 80 V. |
| Alternative | TI LM5069-2 | 9 V to 80 V operating, 100 V absolute maximum, power limiting, UVLO/OVLO. It opens on overvoltage. It does not clamp. Use it only if the measured ring stays under 70 V. https://www.ti.com/product/LM5069 |
| Pass FET | Infineon IPB072N15N3 G | 150 V, 7.2 mOhm, D2PAK. VERIFY current rating and SOA for a 100 W, 10 ms clamp pulse. |
| Alternative FET | Infineon BSC160N15NS5 | 150 V, 16 mOhm, SuperSO8. VERIFY. Smaller, less SOA. |
| TVS | Littelfuse SMCJ70A | 70 V standoff, 1500 W. Clamp about 113 V at rated pulse (VERIFY). Stays off through regen; clamps under the 150 V FET. |
| Sense resistor | 8 mOhm, 2512, 2 W | Breaker at 50 mV = 6.25 A (VERIFY LT4363 threshold). |
| Fuse F1 | Littelfuse 0895020.Z LP-JCASE 20 A, 58 V DC | Fire backstop only. The stopper is the breaker. 58 V is above the 54.6 V pack maximum but below the 60 V design input (R1). VERIFY: for a 14S pack or for margin, use a fuse with a DC rating of 80 V or more. https://www.littelfuse.com/products/fuses/automotive-passenger-car/cartridge-fuses/895.aspx |
| Alternative fuse | Littelfuse NANO2 0454010.MR 10 A 125 V AC/DC | VERIFY that the 10 A rating exists in that series. |
| Input capacitors | 2 x 47 uF 100 V polymer + 2 x 2.2 uF 100 V X7R | ESR keeps the stopper stable. |

Set points: UV lockout 30 V. Output clamp 68 V (VERIFY range).
Current limit 6.25 A. Fault timer long enough for 100 uF of input
capacitance at 55 V.

Grok asked for one scope capture of a hot-plug and one of a hard brake
at this input before the clamp values are final. That is bench test
T-1 in section 9.

### 3.2 14 V converter

Two paths. The first spin buys the converter. The second spin builds it.

Why a brick first (Grok, section 11.1): the 100 V margin of a discrete
buck does not help when the front end is the real limit; a shorted
high-side FET puts the pack on the 14 V rail; a first hand-built buck
runs 90 to 93 percent, not 95; layout is the product. A brick is
potted, surge-rated, isolated, and known.

| Item | Part | Key ratings |
|---|---|---|
| Converter, spin 1 | TRACO TEP 150-4815WI | 18 V to 75 V input, 15 V 10 A, 150 W, isolated 1.5 kV. Efficiency 86 to 87 percent (listing). Operating -40 C to +75 C, case maximum 95 to 100 C. Metal case with flanges. EN 61000-4-5 surge 1 kV. https://www.tracopower.com/products/tep150wi . VERIFY: trim range reaches 14.0 V (needs -6.7 percent; the series is "adjustable", the exact range is in the datasheet), derating curve, case-to-flange thermal resistance. |
| Alternative brick | Mean Well SD-150C-12 | 36 V to 72 V input, 12 V 12.5 A, 77 percent. Rejected: 36 V minimum sits at the 37.7 V BMS cut, 12 V is the wrong rail, 35 W of heat. https://www.meanwell.com/productDetail.aspx?i=361 |
| Converter, spin 2 | TI LM5146-Q1 controller | 5.5 V to 100 V input, 0.8 V to 60 V output, synchronous buck. https://www.ti.com/product/LM5146-Q1 |
| FETs, spin 2 | Infineon BSC098N10NS5 (low side), BSC160N10NS3 or similar (high side) | 100 V. Low side 9.8 mOhm. High side smaller: duty is 0.26, so switching loss dominates there (Grok). VERIFY high-side part. |
| Alternative controller | TI LM5145 | 6 V to 75 V. Less margin. |

Rules for spin 2, from the review: sense-resistor current limit, not
RDS(on); bias (VCC) back-fed from the output through a small LDO so
the controller does not burn 2 W from 54 V; copy the evaluation
module layout; an output overvoltage disconnect (a second LM5069 with
OVLO at 16.5 V) between the buck and the rail; fixed 14.0 V, not a
range.

14 V rail protection on spin 1: SMBJ18A TVS on the rail, 16.5 V
comparator that opens the LTC4359 (3.5) and OUT1/OUT2.

Thermal case, section 4.

### 3.3 Switched 14 V outputs (OUT1, OUT2)

Kevin said "relay". Comparison:

| | Mechanical relay | Smart high-side FET |
|---|---|---|
| Isolation | Yes | No |
| Coil or drive power | 0.5 to 1 W each | 1 mW |
| Inrush into a capacitive load | Contact welding risk | Current limit, soft on |
| Short to ground | Needs a fuse per channel | Internal limit, retry |
| Current telemetry | No | Yes (sense pin) |
| Wear | Contacts | None |
| Fail state | Open on loss of drive | Open on loss of drive |

Recommendation: smart high-side FETs. Nothing on the robot needs
isolation. The sense pin gives per-channel current to the Pi, which
also catches a shorted light bar before it resets the computer.

| Item | Part | Key ratings |
|---|---|---|
| Switch, 2 pieces | TI TPS1HB08-Q1 | 6 V to 18 V operating, 40 V load dump, 8 mOhm, 10 A nominal, limit 6.4 A to 70 A, analog current sense, open-load and short detection, reverse battery. HTSSOP-16. Active. https://www.ti.com/product/TPS1HB08-Q1 |
| Alternative | Infineon BTS7008-2EPA | Dual channel, 8 mOhm, 7.5 A nominal per channel, PROFET+2. One package for both outputs. https://www.infineon.com/cms/en/product/power/smart-power-switches/high-side-switches/profet-plus-2-12v-automotive-smart-high-side-switch/bts7008-2epa/ |
| Relay, if isolation is required | TE T9AS1D12-12 or Omron G8P-1A4P 12 V | 30 A contacts. Coil at 14 V is inside the 130 percent limit (VERIFY coil rating). |

Each output: 2-pin Molex Micro-Fit 3.0 (8.5 A per pin, VERIFY) or
5.08 mm screw terminal for 10 A. Current limit set to 12 A. Default
off. The Pi turns an output on over UART. The module turns both off
on input loss, on hard cut and on an overvoltage on the rail.

### 3.4 5 V rails

Two independent bucks from the SYS bus (3.5). The Pi rail is 5.1 V.
The USB rail is 5.0 V. A USB fault cannot brown out the Pi.

| Item | Part | Key ratings |
|---|---|---|
| Pi buck | TI LM61495 (LM61495-Q1) | 3 V to 36 V input, 10 A, synchronous, 400 kHz, VQFN-HR 4.5 x 3.5 mm. https://www.ti.com/product/LM61495-Q1 . Set 5.10 V. Rated 6 A continuous in this design. |
| USB buck | TI LM61495 | Same part. Set 5.00 V. 5 A. |
| Alternative | TI LMQ61460 | 36 V, 6 A. Pi rail only. |
| Pi eFuse | TI TPS25982 | 2.7 V to 24 V, 15 A, 2.7 mOhm, adjustable current limit and slew, current monitor. https://www.ti.com/product/TPS25982 . Limit 8 A, slew about 2 ms. VERIFY output discharge feature; the design adds its own discharge FET (3.8). |
| USB port switches, 4 pieces | TI TPS25221 | 2.5 V to 5.5 V, 70 mOhm, limit 275 mA to 2.7 A by resistor, reverse current blocking, FAULT pin. https://www.ti.com/product/TPS25221 |
| Alternative port switch | TI TPS2561 (dual) | Two ports per package. VERIFY reverse blocking. |

Port plan and limits:

| Port | Device | Limit |
|---|---|---|
| P1, USB 3 injector | OAK-D Lite | 2.5 A |
| P2, USB 3 injector | Hub upstream | 2.5 A |
| P3, USB 2 power | Arduino RC reader | 1.0 A |
| P4, USB 2 power | Spare (RC receiver, lidar) | 1.5 A |

OAK-D Lite draw: 4 W running, 4.5 W maximum, spikes of 2 W, 1.5 A
maximum input current per Luxonis. Source:
https://docs.luxonis.com/hardware/products/OAK-D%20Lite .

Injector. A USB 3 injector is two connectors on the module: a USB 3
Type-A receptacle for the device and a short USB 3 cable to the Pi.
D+, D-, SSTX and SSRX pass straight through. The device VBUS comes
from the module's port switch. The Pi-side VBUS pin is not connected.
The device cannot backfeed the Pi. The Pi's port sees no load. The
SuperSpeed pairs need 90 ohm differential routing, under 50 mm, on
the four-layer board. VERIFY: enumerate the OAK at 5 Gbps through the
injector (bench test T-10).

Alternative for the product spin: a Microchip USB5744 four-port USB 3
hub on the module, which removes the external hub.

Pi firmware settings that this rail enables (section 6.3): with a 5 A
capable rail, `PSU_MAX_CURRENT=5000` in the EEPROM is correct. The
repository rule "do NOT set `usb_max_current_enable=1`" exists only
because the HAT (B) could not supply 5 A. That rule ends when this
module is installed.

### 3.5 Power path (SYS bus)

The SYS bus feeds both 5 V bucks. It is the OR of the 14 V rail and
the buffer pack.

| Item | Part | Key ratings |
|---|---|---|
| 14 V to SYS ideal diode | ADI LTC4359 | 4 V to 80 V, 9 uA shutdown, SHDN pin with internal pull-up (floats on). https://www.analog.com/en/products/ltc4359.html |
| Pack to SYS ideal diode | TI LM74700-Q1 | 3.2 V to 65 V, 20 mV forward drop, 80 uA. https://www.ti.com/product/LM74700-Q1 |
| Both FETs | Infineon BSC0902NS or similar 30 V, 2 mOhm | VERIFY. |
| SYS holdup | 2 x 220 uF 25 V polymer | Covers the 14 V to 7.6 V hand-over. |

Behaviour. Input present: SYS = 14 V, the pack diode is reverse
biased, the pack only sees the charger. Input lost: the 14 V rail
falls, the pack diode conducts at about 7.5 V, SYS follows. The bucks
accept 3 V to 36 V, so the 5 V rails do not see the step. There is no
NVDC charger and no gap.

Drain mode. The MCU pulls LTC4359 SHDN low. SYS falls to the pack.
The Pi runs from the pack with the charger disabled. The module uses
this to bring the pack from 8.40 V back down to 7.60 V after a top-up
(3.6), without a bleed resistor. If the MCU dies, SHDN floats high and
the 14 V path returns.

### 3.6 Charger, protector, balancing, monitor (the "custom BMS")

Lessons from the HAT (B) and how each is closed:

| Lesson | Fix |
|---|---|
| No charger IC; the supply set point was the charge limit. | A real CC/CV charger. Its float is set by resistors, 7.60 V. |
| Mains presence came from battery current. | Input presence comes from the 60 V side (3.7). |
| Shunt labelled 0.1R, really 10 mOhm. | One 10 mOhm 1 percent shunt, measured by the INA228, calibrated on the bench (T-14). |
| Nothing cut the Pi after a halt, nothing woke it. | 3.8. |
| Cells with about 1 ohm. | Specify cells with 35 mOhm or less (Samsung INR18650-35E, initial impedance 35 mOhm AC maximum). |
| Balancers only act near 4.2 V, so at 3.8 V they never balance. | Keep passive 4.20 V balancers and run a scheduled top-up to 8.40 V so that they get their chance. Host balancing at 3.80 V was rejected (section 11.2). |

Why not a programmable charger (BQ25713 or BQ25792). Grok's objection
(11.2) holds: the dead-MCU state of a register charger is whatever the
power-on defaults say, and no default means "hold 7.60 V". With
resistors the dead-MCU state is 7.60 V.

| Item | Part | Key ratings |
|---|---|---|
| Charger | TI BQ24610 | Stand-alone synchronous buck charge controller, 1 to 6 cells, 5 V to 28 V input (33 V absolute), up to 10 A, CV by resistor divider, CE pin, TS pin for an NTC, STAT pins. https://www.ti.com/product/BQ24610 . VERIFY lifecycle: one distributor lists the RGET as obsolete, TI lists the part as active. |
| Alternative charger | TI BQ24650 | Same family, solar MPPT input. Or TI BQ25713 for a product spin with a watchdog-safe firmware (11.2). |
| Charger FETs | 2 x 30 V, 10 mOhm, SuperSO8 | VERIFY. |
| Charger inductor | 10 uH, 3 A saturation | 1 A charge current. |
| CV set | FB divider: 7.60 V | A 2N7002 switches a third resistor in parallel with the lower resistor. That lowers the lower leg and raises the CV to 8.40 V while the MCU drives the gate. Gate pulled down on the PCB. WARNING: do not put the FET across the whole lower resistor. That pulls FB to ground, and the charger then charges with no voltage limit until the protector opens at 4.25 V per cell. With the parallel resistor, a FET that fails short gives 8.40 V and never more. |
| CC set | ISET: 1.0 A | 2 x 3500 mAh at 0.3 C. |
| Protector | ABLIC S-8252 series (2 cells) | Fixed OV 4.25 V, UV 2.70 V, OCD, short. SOT-23-6. Low-side NFETs. https://www.ablic.com/en/semicon/products/battery-protection/battery-protection-ic/s-8252/ . Choose the suffix for 4.25 V OV and 2.7 V UV. Lineage note: the HAT (B) uses the HY2120, which is the same class of part. |
| Alternative protector | TI BQ76905 | 2S to 5S, programmable, I2C, coulomb counter, host-controlled balancing. Rejected for spin 1: its ground is cell-negative, below the low-side FETs, so its I2C splits from the MCU ground when the FETs open. Product spin option with an MCU referenced to cell-negative. https://www.ti.com/product/BQ76905 |
| Protector FETs | 2 x 30 V, 5 mOhm, in series back to back | VERIFY. |
| Balancers | HYCON HY2213-BB3A, one per cell | Shunt balancer at 4.20 V, 50 mA class. Used on the HAT (B). VERIFY part suffix and current. Alternative: TL431 + resistor shunt at 4.20 V. |
| Pack monitor | TI INA228 | 85 V, 20-bit, I2C, current, bus voltage, temperature, charge accumulator. https://www.ti.com/product/INA228 . On the module's own I2C bus, referenced to pack positive side of the protector (system ground). |
| Cell temperature | 10 k NTC on the cell holder | Into BQ24610 TS and the MCU ADC. |
| Cells | 2 x Samsung INR18650-35E | 3500 mAh, 35 mOhm maximum initial impedance. Alternative Molicel P28A. VERIFY the holder contact resistance. |

Top-up schedule (firmware). Every 8 weeks, if the input is present,
the cell temperature is above 10 C and the Pi is not armed (the Pi
tells the module), the MCU raises CV to 8.40 V. The charger runs to
taper. The balancers act above 4.20 V. The MCU then disables the
charger, enters drain mode (3.5) until the pack reads 7.60 V at rest,
and returns to float. An input loss during the top-up ends the top-up
at once. The pack is fuller than usual at that moment, which is
harmless.

State of charge. The INA228 charge accumulator drifts (Grok). The MCU
reports SOC from a resting-voltage table and reports the accumulator
as a separate number. The number that matters is the 10 s load test:
the MCU records the pack sag at the Pi's boot current once per day and
reports the cell resistance. Above 150 mOhm per cell it raises a
warning. That is the HAT (B) failure seen early.

Chemistry. This design is for NMC or NCA cells. A LiFePO4 cell at
3.80 V is already over its full charge, and 4.20 V damages it. Do not
fit LiFePO4 cells.

### 3.7 Input-present detection

A resistor divider on the 60 V input (1 MOhm and 43 kOhm, 80 V reads
3.3 V) feeds the MCU ADC and a comparator with hysteresis (TI
TLV7031, 335 nA). Thresholds: present at 36.0 V rising, lost at
32.0 V falling. The divider draws from the input only. With no input it
draws nothing from the pack. The comparator edge wakes the MCU from
Stop mode. The LT4363 UV lockout (30 V) sits below the "lost" line so
that the module declares the loss before the stopper opens.

Debounce is in firmware (section 5): 2 s for a loss, 5 s for a
return, longer after repeated flaps.

### 3.8 Pi control path: halt detection, cut, wake

Three signals and one pulse.

| Signal | Direction | How |
|---|---|---|
| SHUTDOWN_REQ | Module to Pi, GPIO 27 | Open drain on the module. Pull-up on the Pi only. `dtoverlay=gpio-shutdown,gpio_pin=27,active_low=1,gpio_pull=up`. The kernel raises a power-key event, logind runs poweroff. No daemon is in the path. |
| PI_HALTED | Pi to module, header pin 1 (3V3) | 100 k / 47 k divider into an MCU input. The module sources nothing into the pin. With `POWER_OFF_ON_HALT=1` and `WAKE_ON_GPIO=0`, a halt puts the PMIC in STANDBY and "all outputs are switched off" (Raspberry Pi documentation, bootloader properties). 3V3 falls to 0 V. That is the halt witness. |
| PI_5V_SENSE | Pi side of the eFuse to module | Divider into an MCU input. Used for the collapse check and the apply check. |
| PWR_BUT pulse | Module to Pi J2 pads | 2N7002 open drain across the J2 pads, 200 ms (VERIFY the width on the bench, T-9). "Briefly closing this switch will perform the same actions as the onboard power button" (Raspberry Pi documentation). Used only when the input is present and 3V3 is already down. Never push-pull. |

Sequence on input loss (full state machine in section 5):

1. Loss debounced. Assert SHUTDOWN_REQ. Send `SHUTDOWN_PENDING` on
   UART. Start the 120 s timer. Keep 5 V and USB on.
2. The Pi halts. 3V3 falls. Deassert SHUTDOWN_REQ. Open the Pi eFuse
   and the USB port switches. Close the discharge FET (47 ohm, 1 W)
   across the Pi-side rail for 200 ms.
3. Read PI_5V_SENSE. Wait until it is below 1 V. If it does not fall
   within 5 s, something feeds the Pi (a USB-C supply, a HAT). Log a
   fault and keep waiting; never re-apply 5 V onto a rail that did not
   collapse (Grok, 11.3).
4. Module goes dark: outputs off, charger as the input allows, MCU in
   Stop. The comparator wakes it.
5. Input returns, debounced. SHUTDOWN_REQ is deasserted. Close the Pi
   eFuse with slew control. Read PI_5V_SENSE within 100 ms. If it did
   not rise, open, wait 1 s, retry, up to 5 times, then raise a fault
   LED pattern. The Pi boots from a cold power-on. No J2 pulse on this
   path.

Sequence on a halt while the input is present (operator `poweroff`,
ch5 e-stop): leave 5 V on. The Pi sits in PMIC standby at about
3 mA. Wake only by the Pi's own button, the module's button, or a
UART-set one-shot auto-wake that is off by default and is never
armed after an e-stop (Grok, 11.3). See open question 8.

Hard cut: if 3V3 is still up 120 s after SHUTDOWN_REQ, open the
eFuse anyway, then run the same collapse check and the same cold apply.

Dead-MCU hardware default: `EN_PI5V = MCU_EN OR (VIN_OK AND NOT
HOLD)`. HOLD is a latch (74LVC1G74 or the MCU's own output with a
300 s RC timeout) set at step 1 and cleared after step 3. A crashed
MCU with the input present therefore leaves the Pi powered. A crashed
MCU during a shutdown holds the cut until the timeout, then the
hardware applies 5 V again. A powered, disarmed Pi is recoverable. A
dark Pi is not.

RTC. A cold apply loses the RTC time unless a cell sits on the Pi's J5
connector. GPS or NTP sets the clock at boot. See open question 7.

### 3.9 Microcontroller

| Item | Part | Why |
|---|---|---|
| MCU | ST STM32G071CBT6 | 64 pin LQFP, 128 kB flash, UART x4, I2C x2, 12-bit ADC, Stop mode in the microamp range, independent watchdog, 2 V to 3.6 V. Hand-solderable. |
| Alternative | Raspberry Pi RP2040 | Familiar to Pi users. Needs external flash and has higher sleep current. Firmware in MicroPython or C. |
| 3.3 V housekeeping LDO | TI TPS7B81-Q1, from SYS | 40 V input, 150 mA, 2.7 uA quiescent. VERIFY part number. Alternative: TI TPS7B69. |
| Status | 1 RGB LED, 1 button, 1 buzzer | Button short: wake the Pi (J2 pulse). Button 5 s: force cut (bench only, confirmed on UART). |

The MCU's I2C bus carries only the INA228 and the optional BQ76905. It
never touches the Pi. The UART to the Pi is UART0 on GPIO 14 and 15,
3.3 V, 115200 8N1, through a 1 kOhm series resistor each way so that a
live MCU cannot push current into a dead Pi.

Firmware watchdog: the independent watchdog resets the MCU after 2 s
without a kick. All pin defaults after a reset are the safe states in
section 5.4.

### 3.10 Connectors and fusing

| Connector | Type | Rating |
|---|---|---|
| Pack input | XT60 (panel or PCB) | 30 A continuous, 60 A peak, 500 V. Open question 4: XT90 if the robot harness is XT90. |
| OUT1, OUT2 | 5.08 mm screw terminal, 2 pin | 15 A. |
| Pi power | 6-wire harness to header pins 2, 4 (5 V) and 6, 9, 14, 20 (GND) | 3 A per pin, 6 A on 2 pins. Alternative: pogo pins to the Pi 5 bottom pads as the HAT (B) does. |
| Pi signals | 6-wire harness: 3V3 sense (pin 1), GPIO 14, 15, 27, 22, GND | Under 150 mm. |
| J2 | 2-wire to the Pi's J2 pads | Solder or 2-pin 1.0 mm JST. |
| USB injectors | 2 x USB 3 Type-A receptacle + 2 x USB 3 Type-A plug cable, 150 mm | 90 ohm pairs. |
| USB power outputs | 2 x USB Type-A receptacle, power only | D+ D- open. |
| Cells | 2 x 18650 holder, Keystone 1043 or spring holder | VERIFY contact resistance under 10 mOhm. Alternative: solder tabs on protected cells. |
| Fan (optional) | 2 pin JST-PH, 5 V | For the enclosure. |

Fusing: F1 on the input (3.1). The 14 V outputs are protected by the
high-side switches. The 5 V rails are protected by the eFuse and the
port switches. The pack path is protected by the protector FETs and a
5 A polyfuse in the cell lead as a second layer.

## 4. Power budget, efficiency, thermal

### 4.1 Loads

| Load | Typical | Peak |
|---|---|---|
| Pi 5 + NVMe, control stack | 6 W | 12 W (boot, 2.5 A) |
| OAK-D Lite | 4 W | 6.5 W |
| Hub, RC receiver, Arduino | 2 W | 3 W |
| Spare USB port | 0 | 7.5 W |
| 5 V side total | 12 W | 29 W |
| At 14 V after 92 percent bucks | 13 W | 32 W |
| Charger at 1 A into 8.4 V, 90 percent | 0 (float) | 9.5 W |
| OUT1 + OUT2 (future lidar, lights) | 0 today | 84 W (6 A) |
| Module housekeeping | 1 W | 1 W |
| 14 V rail total | 14 W (1.0 A) | 127 W (9.1 A) |
| Input at 87 percent, 37.7 V | 16 W (0.43 A) | 146 W (3.9 A) |

The 10 A tap peak (140 W on the outputs alone) exceeds 150 W when
the Pi and the charger run at the same time. The module meets it like
this: the charger pauses when the rail current passes 9 A, and the
outputs may draw 10 A combined for 60 s, 8 A continuous. Open question
2 asks whether Kevin wants a 200 W converter instead.

### 4.2 Heat

| Source | Typical | Worst |
|---|---|---|
| TEP 150 at 87 percent | 2 W | 22 W |
| Two 5 V bucks at 92 percent | 1 W | 2.5 W |
| Charger | 0 | 1 W |
| OUT1 + OUT2, 8 mOhm each | 0 | 0.4 W |
| Surge stopper FET + sense | 0.05 W | 0.3 W |
| Total | 3 W | 26 W |

Typical WALL-E operation is near the 3 W column. The 26 W column is
the future-loads case.

Thermal path. The TEP 150 case has flanges. Bolt it to an aluminium
plate that is part of the robot chassis, with a thermal pad. At 45 C
ambient and 22 W, the case stays under 95 C only if the path from case
to ambient is 2.2 K/W or better. A 3 mm aluminium plate of 150 x 150
mm in still air is roughly 3 K/W (VERIFY on the bench, T-3). Therefore:
either derate the rail to 100 W above 40 C ambient (firmware reads a
case thermistor and trims the output limits), or give the plate a path
to the chassis skin. The 5 V bucks dissipate 1 to 2.5 W on the PCB
copper; a 4-layer board with 2 oz outer copper and thermal vias is
enough.

### 4.3 Buffer energy

Worst shutdown: Pi 12 W + USB 10 W + module 1 W for 120 s = 2.8 kJ
= 0.77 Wh = 104 mAh at 7.4 V. Two 3500 mAh cells at 50 percent hold
about 1750 mAh. The buffer has 16 times the worst case.

Sag at 2.6 A with 35 mOhm cells: 0.18 V on the pack. With the HAT (B)
cells at 1 ohm: 5.2 V, which trips UV at once. The cell specification
is the fix, not the circuit.

Idle after the cut, input absent: MCU Stop 10 uA, LDO 3 uA, INA228
shutdown 2 uA, LM74700 80 uA, two bucks disabled 10 uA, protector
5 uA, measurement dividers 10 uA. About 120 uA. 1750 mAh / 120 uA is
about 600 days. Requirement R11 is met with margin.

## 5. State machine

### 5.1 States

| State | 5 V Pi | USB | OUT1/2 | Charger |
|---|---|---|---|---|
| OFF_NO_INPUT | off | off | off | off |
| APPLYING | closing, check | off | off | on |
| RUNNING | on | on | per Pi | on |
| SHUTDOWN_REQ | on | on | off | on |
| HALT_REQUESTED | on | on | off | on |
| COLLAPSE_CHECK | off | off | off | as input |
| OFF_LATCHED | off | off | off | on |
| HALTED_INPUT_PRESENT | on (standby) | off | off | on |
| HARD_CUT | off | off | off | as input |
| FAULT | per fault | off | off | off |

### 5.2 Transitions

| From | Event | To |
|---|---|---|
| OFF_NO_INPUT | VIN_OK for 5 s (or the backoff time) | APPLYING |
| APPLYING | PI_5V_SENSE rose within 100 ms | RUNNING |
| APPLYING | no rise, 5 tries | FAULT (apply) |
| RUNNING | VIN lost for 2 s | SHUTDOWN_REQ |
| RUNNING | 3V3 fell with input present | HALTED_INPUT_PRESENT |
| RUNNING | UART `HALT_NOW` from the Pi (operator poweroff, ch5 e-stop) | HALT_REQUESTED |
| HALT_REQUESTED | 3V3 fell, input present | HALTED_INPUT_PRESENT (no cut, no re-apply) |
| HALT_REQUESTED | VIN lost for 2 s | SHUTDOWN_REQ |
| HALT_REQUESTED | 120 s elapsed, 3V3 still up | HARD_CUT (latched) |
| SHUTDOWN_REQ | 3V3 fell | COLLAPSE_CHECK |
| SHUTDOWN_REQ | 120 s elapsed, 3V3 still up | HARD_CUT |
| HARD_CUT | eFuse opened | COLLAPSE_CHECK |
| COLLAPSE_CHECK | PI_5V_SENSE under 1 V and VIN lost | OFF_NO_INPUT |
| COLLAPSE_CHECK | PI_5V_SENSE under 1 V and VIN_OK, after an input-loss shutdown | APPLYING |
| COLLAPSE_CHECK | PI_5V_SENSE under 1 V and VIN_OK, after a latched hard cut | OFF_LATCHED |
| OFF_LATCHED | module button | APPLYING |
| OFF_LATCHED | VIN lost for 2 s | OFF_NO_INPUT |
| COLLAPSE_CHECK | rail did not fall in 5 s | FAULT (sneak feed), keep checking |
| HALTED_INPUT_PRESENT | button, J2 request, or auto-wake | pulse J2, then RUNNING |
| HALTED_INPUT_PRESENT | VIN lost for 2 s | COLLAPSE_CHECK (open eFuse first) |
| any | 14 V rail over 16.5 V | FAULT (OV): open LTC4359, OUT1/2 |

### 5.3 Flapping input, brown-out, a Pi that never halts

Flapping. A loss shorter than 2 s is ignored. Once SHUTDOWN_REQ is
asserted, the shutdown completes; an input return does not cancel it.
The return debounce grows: 5 s normally, 60 s after the third loss
within 10 minutes, 300 s after the sixth. The counter clears after 1 h
of stable input. The boot window (first 30 s after APPLYING) requires
10 s of loss instead of 2 s, so that boot inrush sag cannot start a
shutdown.

Brown-out. Between 32 V and 36 V the module logs `degraded` and does
nothing else. The converter runs to 18 V. The VESC pack-low logic in
`config.py` (39 V for 30 s) cuts the motors long before this.

A Pi that never halts. 120 s after SHUTDOWN_REQ the eFuse opens. The
NVMe root may be dirty. Section 6.3 sets the systemd timeouts so that
the Pi gives up on a stuck camera driver before the module gives up on
the Pi. The timeout is a UART-settable register (60 s to 600 s).

Input returns during SHUTDOWN_REQ. The shutdown completes, the rail
collapses, and the module applies 5 V again. The Pi reboots once. This
is the designed behaviour for a 3 s to 120 s outage.

A halt that the Pi requests (HALT_NOW). This path never re-applies
5 V by itself. The ch5 e-stop must leave the Pi down until a person
acts (`CLAUDE.md`, "ch5 e-stop powers the Pi off"). After a normal
halt the Pi sits in PMIC standby with 5 V on. After a hard cut the
module holds 5 V off in OFF_LATCHED. In OFF_LATCHED the MCU re-arms
the HOLD latch (3.8) every 60 s. If the MCU dies, the hardware applies
5 V after 300 s. That is the dead-MCU rule in 5.4, and it wins on
purpose. A pack switched off and on again is a deliberate power cycle,
and the Pi boots.

### 5.4 Safe states with a dead MCU

| Line | PCB default | Effect |
|---|---|---|
| EN_PI5V | hardware OR with VIN_OK | Pi powered when input present |
| HOLD latch | cleared after 300 s | cut cannot stick |
| SHUTDOWN_REQ | released (open drain) | no shutdown loop |
| OUT1, OUT2 | pull-down, off | loads off |
| USB port enables | pull-down, off | USB off until the MCU runs |
| Charger CE | pull-up, on | charger runs at 7.60 V |
| CV bump FET | pull-down | 7.60 V, not 8.40 V |
| LTC4359 SHDN | floats high | 14 V path on |
| J2 pulse FET | pull-down | no pulse |

The USB default "off" means that a dead MCU with a running Pi has no
camera. The Pi reports this on the dashboard (`module_alive` false in
6.2). Open question 10 asks whether USB should default on instead.

## 6. Pi-side software interface

### 6.1 What replaces `bin/walle_ups.py`

A new daemon `bin/walle_power.py` (`walle-power.service`, root) owns
`/dev/ttyAMA0`. It:

- reads the module status at 2 Hz and writes `/tmp/ups_status.json`
  with the schema `walle-power/1` (fields in 6.2), so the `/debug` page
  and the dashboard keep working with small changes;
- forwards output commands from the web UI (`POST /api/power/out`);
- on `SHUTDOWN_PENDING`: sends SIGKILL to `wall-e.service` and stops
  it (the camera teardown can take 90 s), then runs `sync`. It does not
  call `poweroff`; the gpio-shutdown overlay already did. The daemon is
  a head start, not the thing the halt depends on;
- sends `ARMED` / `DISARMED` so the module never starts a top-up on an
  armed robot;
- sends a 1 Hz heartbeat. The module uses it only to know that user
  space is up before it asserts SHUTDOWN_REQ (a request during the
  kernel boot would loop the Pi).

`walle-ups-wakecheck.service` is deleted. There is no RTC polling.
`i2c-bus-guard.service` loses its UPS witness (the UPS status age);
it must use the RTK GPS reader's status age instead (open question
12).

### 6.2 Protocol

ASCII lines, 115200 8N1, `\n` terminated, CRC-8 as the last field.

Module to Pi, every 500 ms and on change:

```
S,<state>,<vin>,<v14>,<i14>,<vsys>,<vpack>,<ipack>,
  <soc>,<cell_mohm>,<t_cell>,<t_case>,<out1>,<i_out1>,
  <out2>,<i_out2>,<usb_mask>,<flags>,<uptime>,<crc>
```

Events: `E,SHUTDOWN_PENDING,<timeout_s>`, `E,INPUT_LOST`,
`E,INPUT_BACK`, `E,TOPUP_START`, `E,TOPUP_END`, `E,FAULT,<code>`.

Pi to module:

| Command | Effect |
|---|---|
| `HB` | heartbeat |
| `OUT,1,ON` / `OUT,2,OFF` | switch an output |
| `USB,<port>,ON/OFF` | switch a port (for the camera restart case) |
| `ARMED` / `DISARMED` | top-up gate |
| `HALT_NOW` | start a Pi-requested halt (replaces the ch5 `shutdown -h now`). The module never re-applies 5 V by itself after it (5.3). |
| `SET,timeout,<s>` | hard-cut timeout |
| `SET,autowake,<s>` | one-shot auto-wake after a halt with input present, 0 = off |
| `SET,topup_weeks,<n>` | top-up period, 0 = never |
| `TOPUP` | start a top-up now |
| `GET,cfg` | dump configuration |
| `ID` | firmware version, board revision |

The module ignores a command with a bad CRC. It never trusts the Pi
for a safety decision: the Pi cannot disable the hard cut, cannot
raise the CV above 8.40 V, cannot keep an output on through an
overvoltage.

### 6.3 Pi settings that change

EEPROM (`rpi-eeprom-config -e`):

```
POWER_OFF_ON_HALT=1     # already set 2026-11-03
WAKE_ON_GPIO=0          # already set 2026-11-03
PSU_MAX_CURRENT=5000    # new: the module's Pi rail is 5 A capable
```

`PSU_MAX_CURRENT` "instructs the firmware to skip USB power-delivery
negotiation and assume that it is connected to a power supply with the
given current rating" (Raspberry Pi documentation). With 5000 the
firmware sets the USB limit to 1.6 A on its own, so
`usb_max_current_enable` is not needed. Condition: the module's Pi
rail must be rated 5 A continuous at the header, which 3.4 gives.

`/boot/firmware/config.txt`:

```
dtoverlay=uart0-pi5                 # /dev/ttyAMA0 on GPIO 14, 15
dtoverlay=gpio-shutdown,gpio_pin=27,active_low=1,gpio_pull=up
# do NOT add gpio-poweroff: it interferes with the PMIC standby path
```

Checked on the Pi on 2026-11-04: the CAN board is an MCP2515 on SPI0
(`dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25`), so SPI0
and GPIO 25 are in use. GPIO 14, 15, 22 and 27 are unclaimed. UART0 is
free: `serial0` points to the debug UART `ttyAMA10`.

systemd: `TimeoutStopSec=20` on `wall-e.service`, and
`DefaultTimeoutStopSec=30s` in `/etc/systemd/system.conf`, so that the
Pi finishes a poweroff inside the 120 s window even with a camera
driver in state D. `HandlePowerKey` is not set in logind.conf, so the
default `poweroff` applies (checked 2026-11-04).

The ch5 e-stop path in `controller.py` changes from `sudo shutdown -h
now` to `HALT_NOW` over the daemon, so that the module knows the halt
was requested and goes to HALTED_INPUT_PRESENT with no auto-wake.

## 7. Rough bill of materials (research only)

Single-unit prices, estimated from 2026 distributor listings. Not
quotes. The TEP 150 price is from memory and needs a check.

| Block | Parts | Estimate USD |
|---|---|---|
| Input: LT4363, FET, TVS, fuse, holder, caps | | 22 |
| 14 V: TEP 150-4815WI | | 160 |
| OUT1/2: 2 x TPS1HB08-Q1, terminals | | 10 |
| 5 V: 2 x LM61495, inductors, caps, TPS25982 | | 16 |
| USB: 4 x TPS25221, 2 USB 3 receptacles, 2 cables, 2 USB-A | | 22 |
| Power path: LTC4359, LM74700, FETs, polymer caps | | 10 |
| Charger: BQ24610, FETs, inductor, sense | | 10 |
| Protector, balancers, FETs, polyfuse | | 5 |
| INA228, NTC, shunt | | 6 |
| MCU, LDO, comparator, LED, button, buzzer, latch | | 8 |
| Connectors: XT60, Micro-Fit, harnesses, JST | | 15 |
| Passives, TVS on rails | | 12 |
| 4-layer PCB, 5 pieces, 100 x 120 mm | per board | 15 |
| Cells, 2 x INR18650-35E, holders | | 14 |
| Total, spin 1 | | about 325 |
| Spin 2 delta: LM5146 + FETs + inductor replaces the brick | | -135 |
| Total, spin 2 | | about 190 |

A product at 100 pieces lands near 120 to 140 USD in parts with the
discrete buck. The brick alone is more than that.

## 8. Build or buy

Products checked on 2026-11-04 (vendor pages or retailer listings):

| Product | Input | Battery | Pi 5 | Gap |
|---|---|---|---|---|
| UUGear Witty Pi 5 HAT+ | 6 to 30 V DC or 5 V USB-C, 5 A out | none | yes | No battery, 30 V maximum, no 14 V tap. Has voltage-triggered safe power-off and RP2350. Closest "wide input + safe shutdown" board. |
| Joy-IT StromPi 3 | 6 to 61 V, 3 A out | 1000 mAh LiFePO4 option | not listed | 61 V input is right. 3 A cannot run a Pi 5. Pi 5 not in the compatibility list. |
| Geekworm X1202 / DFRobot FIT0992 | 6 to 18 V | 4 or 2 x 18650 | yes | 18 V maximum. 5 A. Cuts power after halt. No wide input. |
| Waveshare UPS HAT (E) | USB-C PD 5 to 20 V | 21700 | yes | PD input only. |
| Waveshare UPS HAT (B), current | 8.4 V jack | 2 x 18650 | yes | The lessons in 3.6. |
| SunFounder PiPower 5 | USB-C PD 5 to 15 V | internal | yes | PD only. |
| Geekworm UPS_Scap | 9 to 24 V | supercapacitor | yes | Seconds of hold-up, 24 V maximum. |
| Sequent Super Watchdog | 5 V | 1 or 2 x 18650 | partial (3 A) | 5 V input. |
| PiJuice HAT | 5 V | LiPo | unverified | 5 V input. |
| Sfera Labs Strato Pi UPS | 9 to 28 V | 12 V lead-acid | no (Pi 3, 4) | 28 V maximum, industrial case, no Pi 5. |
| mjbots, VBCores power boards | up to 44 V | none | n/a | Robot power distribution, no Pi UPS logic, no halt coordination. |

How close the best one comes. The StromPi 3 takes the pack voltage but
cannot run a Pi 5 and is not supported on it. The Witty Pi 5 has the
right safe-shutdown logic and a 5 A rail but stops at 30 V and has no
battery. Nothing found takes 36 to 60 V, runs a Pi 5 at 5 A, buffers
the halt, cuts and re-applies 5 V, powers the USB devices, and gives
switched accessory rails. The usual substitute is a Mean Well brick
plus a Geekworm UPS, two boxes with no coordination and the 18 V cap.

Market note. 24 V and 48 V packs are standard on ROS robots, mowers
and AGVs, and the Pi 5 is the common computer. The gap is real but
narrow: the buyers are robot builders, not makers with a bench supply.
A board at 150 to 250 USD with a documented UART protocol and a Pi
daemon would sell in the hundreds, not the thousands. The 14 V tap and
the switched outputs are what the competitors lack most.

## 9. Risks and bench test plan

### 9.1 Risks

| # | Risk | Mitigation |
|---|---|---|
| R-1 | Hot-plug ring exceeds the clamp design. | T-1 measures it before the clamp values are final. 150 V FET and 70 V TVS give room. |
| R-2 | Regen pushes the bus above the clamp during a brake. | The stopper clamps and holds the rail up; it does not open. T-2. |
| R-3 | TEP 150 trim does not reach 14.0 V. | VERIFY the datasheet. Fallback: 13.5 V, or open question 1 (12 V). |
| R-4 | 22 W of brick heat in a closed box at 45 C. | T-3 thermal soak. Firmware derate from the case thermistor. |
| R-5 | The Pi rail does not collapse (sneak feed) and a re-apply does not boot the Pi. | Discharge FET, collapse check, apply check, 5 retries. T-6. |
| R-6 | SHUTDOWN_REQ asserted during boot loops the Pi. | Deassert before apply; assert only after the heartbeat. T-7. |
| R-7 | Hard cut corrupts the NVMe root. | systemd timeouts; consider a read-only root (open question 9). T-8. |
| R-8 | Boot inrush through the eFuse browns out mid-boot. | Slew control, 8 A limit, 30 s boot blanking. T-5. |
| R-9 | A USB 3 injector degrades the OAK link to USB 2 speed. | T-10. Fallback: a Luxonis Y-cable from port P1. |
| R-10 | Charger lifecycle (BQ24610 RGET listed obsolete by one distributor). | VERIFY at TI; BQ24650 fallback. |
| R-11 | Top-up leaves the pack at 4.2 V per cell if the drain mode fails. | Drain mode is MCU-driven; a failed MCU floats SHDN high, the charger CE stays on at 7.60 V, and the pack self-discharges slowly. Log and alarm. T-13. |
| R-12 | Cell holder contact resistance adds to cell resistance. | T-14 measures the whole string. |
| R-13 | J2 pulse width wrong: ignored, or a forced power-off. | T-9. |
| R-14 | Condensation on 54 V tracks. | Conformal coat except thermal pads and connectors. Creepage 2 mm minimum on the input side. |
| R-15 | EMI from a 150 W converter next to the OAK and the GPS. | Brick is filtered (-F variant is an option). T-15 sniff. |

### 9.2 Bench test plan, before the board goes on the robot

| # | Test | Pass |
|---|---|---|
| T-1 | Hot-plug the module into the real pack through the real XT60 harness, 10 times. Scope at the input. | Peak under the clamp set point plus margin; no part above 80 percent of its rating. |
| T-2 | Robot on stands, both VESCs, full-speed brake 10 times. Scope at the input. | Rail stays up; no stopper open; log shows no `INPUT_LOST`. |
| T-3 | 150 W resistive load on the 14 V rail in a closed box at 45 C for 2 h. | Case under 95 C; 5 V rails in tolerance; derate logic trips as designed. |
| T-4 | 14 V rail: load step 0 to 8 A, short on OUT1 with OUT2 loaded. | Pi rail unaffected; OUT1 limits and retries; OUT2 stays on. |
| T-5 | Pi 5 + NVMe boot through the eFuse, 50 cycles, with the OAK on P1. | 50 boots; no `throttled` flag; rail dip under 250 mV. |
| T-6 | Pull the input 50 times at random points. | 50 clean halts (journal shows `Journal stopped`); 50 cold boots; collapse under 1 V each time. |
| T-7 | Input pulse train: 1 s, 3 s, 10 s, 60 s, 150 s outages, 5 each. | 1 s ignored; others one halt and one boot; no boot loop; backoff timing as in 5.3. |
| T-8 | Simulate a stuck camera driver (a process in state D, or `wall-e` with `TimeoutStopSec=300`). | Pi still halts under 60 s via systemd; hard cut at 120 s if forced; filesystem check passes on the next boot. |
| T-9 | Halt with input present. Pulse J2 at 100, 200, 500 ms. | Find the width that wakes without a force-off. |
| T-10 | OAK on P1 through the injector; `getUsbSpeed()` and a 1 h depth stream. | SUPER speed; zero USB resets in `dmesg`. |
| T-11 | Hub on P2; pull the hub's power; probe the Pi-side VBUS pin of the injector. | 0 V on the Pi side; no over-current message in `dmesg`. |
| T-12 | MCU held in reset for 10 min with input present; then during a shutdown. | 7.60 V on the charger; Pi stays powered in the first case; cut holds 300 s then applies in the second. |
| T-13 | Force a top-up: CV reaches 8.40 V, taper, drain mode, return to 7.60 V. Pull the input mid-top-up. | Balancer LEDs light above 4.20 V; top-up ends on the loss; no cell above 4.25 V. |
| T-14 | Load the pack with 2.6 A for 10 s with the input off. | Sag under 0.25 V with the specified cells; the module reports the resistance within 20 percent of a 4-wire meter. |
| T-15 | Near-field probe over the board, OAK and GPS running. RTK fix quality before and after. | No new spurs at the GPS L1/L5 bands; fix quality unchanged. |
| T-16 | Idle current from the pack after a cut, input absent, 24 h. | Under 0.3 mA; pack voltage change under 5 mV. |

## 10. Open questions for Kevin

1. 12 V or 14 V. A strict 12 V load (some lidars) can be out of
   specification at 14.4 V. If 13.2 V is acceptable, a 48 V to 12 V
   quarter brick (commodity, 92 percent, 25 A, trim +10 percent) is
   cheaper and cooler than the TEP 150. If 14.0 V is required, the
   TEP 150-4815WI trimmed down, pending the trim check.
2. Is the 10 A tap simultaneous with the Pi and the charger? If yes,
   the rail is 200 W, the brick changes (TEP 200 class or spin 2 at
   15 A), and the price rises about 40 USD.
3. Which loads go on OUT1 and OUT2 in the next year? Lidar (6 to
   13 W), lights, a heater, a horn? Their inrush sets the current
   limit.
4. Pack connector on the robot harness: XT60 or XT90? Does the module
   hang behind the Daly discharge FET (so a BMS cut also cuts the
   module) or straight on the cells?
5. Mechanical relay with isolation, or the smart FET recommended in
   3.3?
6. Cells: buy Samsung 35E or Molicel P28A now, and scrap the 1 ohm
   cells?
7. RTC: fit a rechargeable cell on J5 with `rtc_bbat_vchg` and
   inhibit charging below 0 C, or accept a clock jump until GPS or
   NTP sets it? Journald, TLS and log timestamps break on a backward
   jump.
8. After a ch5 e-stop the Pi halts with the input present. Button-only
   wake (recommended) or an auto-wake after N minutes?
9. Read-only root with a separate data partition, so that a hard cut
   cannot leave the robot in an `fsck` prompt?
10. USB ports: default off with a dead MCU (safe, no camera) or
    default on (camera runs even with a dead MCU, but the module can
    no longer shed USB before a cut)?
11. Keep the external self-powered hub behind the injector, or put a
    USB5744 hub on the module in spin 2?
12. The I2C bus guard uses the UPS status file as its witness. Move
    the witness to the GPS reader, or let the new daemon write a
    heartbeat file?
13. Spin 1 with the brick (about 325 USD, lower risk) or straight to
    spin 2 with the discrete buck (about 190 USD, more design work)?
14. Where does the module mount, and is there a metal path from that
    spot to the enclosure skin for the 22 W case?

## 11. Appendix: Grok second opinions

Three `grok_second_opinion` calls, Grok 4.7, 2026-11-04, single shot,
no repository access. Sessions `01a1a72a`, `01a1a730`, `01a1a737`.
Cost 0.51 USD total.

### 11.1 The 14 V converter

Proposed: LM5146-Q1 discrete buck with 100 V FETs first, brick second.

| Objection | Response |
|---|---|
| The 100 V rating stops at the LM5069 and the pass FET; it buys no margin on its own. | Accepted. The front end is now a surge stopper with a 150 V pass FET (3.1). |
| An SMCJ58A cannot both stay off at 60 V and clamp under 100 V parts. | Accepted. SMCJ70A behind a 150 V FET. |
| An OVLO that opens during regen drops the Pi while the VESCs still run. | Accepted. The stopper clamps and stays up; it does not open on overvoltage. |
| A shorted high-side FET puts 54 V on the 14 V rail; the brick's isolation is failure containment. | Accepted. Brick for spin 1; an OV disconnect for spin 2. |
| 93 to 95 percent is a best case; a first board is 90 to 93 percent; the controller's bias from 54 V burns 2 W. | Accepted. Numbers in section 4 use the brick's 87 percent; spin 2 rules in 3.2. |
| The 10 A tap plus the Pi plus the charger exceeds 10 A; freeze a load table. | Accepted. Section 4.1 and open question 2. |
| The Mean Well SD-150C does not belong (36 V minimum, 12 V, 77 percent). | Accepted. Listed as rejected. |
| Measure the hot-plug ring and a hard brake before freezing the clamp. | Accepted. T-1 and T-2. |
| Use the TRACO for the boards that make the robot move; discrete later. | Accepted as the spin plan. Rejected in part: Kevin wants a product, so spin 2 stays in the document with the review's rules. |

### 11.2 The charger and BMS

Proposed: BQ25713 NVDC charger plus BQ76905 with host balancing at
3.80 V and a coulomb counter.

| Objection | Response |
|---|---|
| A register charger's dead-MCU state is its power-on default, which is not 7.60 V; a watchdog expiry can restore an 8.4 V class float. | Accepted. The charger is now a stand-alone BQ24610 with the float in resistors and an MCU-held bump to 8.40 V (3.6). |
| Dropping the CV register does not bring the cells down after a top-up; a bleed or a drain is needed. | Accepted. Drain mode through the LTC4359 SHDN pin (3.5). |
| Host balancing at 3.80 V is unmeasurable (flat OCV, millivolt errors, sag on a 1 ohm cell). Passive bleeders near 4.2 V during the rare top-up are the right tool. | Accepted. HY2213-class balancers at 4.20 V plus the scheduled top-up. |
| A coulomb counter is useless for a 65 mAh mission; offset drift is the whole buffer per month. | Accepted. INA228 stays as a monitor; SOC from resting voltage; the daily sag test reports cell resistance instead. |
| NVDC seamlessness is hidden behind the 5 V bucks; an ideal-diode OR with holdup does the same. | Accepted. LTC4359 + LM74700 OR (3.5). |
| Low-side protector FETs split the grounds; two I2C devices on one bus across that split are invalid when the FETs open. | Accepted. The I2C protector is dropped for spin 1; a fixed S-8252 with no bus. INA228 sits on the system side. |
| Boot race: enables must default off in hardware; a floating MCU pin is not off. | Accepted. Section 5.4 table. |
| Idle current is the charger's number; BQ25713 battery-only Iq blows a 1 mA budget. | Accepted. No BQ25713. Idle estimate 120 uA (4.3). |
| 1 ohm cells are the HAT problem; neither IC repairs it. | Accepted. Cell specification 35 mOhm; T-14. |
| LiFePO4 at 3.80 V is over full. | Accepted. NMC/NCA only, stated in 3.6. |
| Skip the fuel gauge until the mission is longer than a minute. | Accepted in part. The INA228 stays for telemetry and the shunt-calibration lesson; it is not called a gauge. |
| BQ76905 if programmable limits are wanted, with an MCU referenced to cell-negative. | Kept as the product-spin alternative. |

### 11.3 Cut-and-wake

Proposed: cut 5 V after the halt, re-apply on input return; J2 pulse
only for the halt-with-input-present case. Grok agreed with the policy
and listed the holes.

| Objection | Response |
|---|---|
| A short outage can re-close the eFuse before the Pi rail collapsed; the PMIC standby latch then holds and the Pi never boots. | Accepted. Discharge FET, collapse check under 1 V, apply check, retries (3.8). |
| gpio-shutdown is level-sensitive; a line still asserted at boot loops the Pi. Check `HandlePowerKey`. | Accepted. Deassert before apply; assert only after the heartbeat; logind check in 6.3. |
| Do not use gpio-poweroff as a witness; it interferes with the standby path. | Accepted. Removed; 3V3 sense only. The overlay README says the same. |
| Sense 3V3 with a high-impedance divider; source nothing into the pin. | Accepted. |
| The 120 s cut is an NVMe corruption path; set systemd stop timeouts; consider a read-only root. | Accepted. 6.3 and open question 9. |
| Size the eFuse for boot surge with slew control; blank the loss detector through the boot window. | Accepted. 8 A limit, slew, 30 s boot window with a 10 s loss rule. |
| No auto-wake after an e-stop; J2 pulse only with input present and 3V3 down; confirm the width. | Accepted. HALT_NOW path, autowake default 0, T-9. |
| RTC: a primary coin cell with charging enabled is a fire; a rechargeable cell must not charge below 0 C; GPS or NTP is the real clock. | Accepted. Open question 7. |
| Hardware default: input present and not in an active shutdown forces 5 V on even with a dead MCU. | Accepted. EN_PI5V hardware OR with the HOLD latch and timeout (3.8, 5.4). |
| The module should go fully dark after the cut; the 60 V comparator wakes it. | Accepted. 3.7 and 4.3. |
| Treat the UART daemon as a head start, not the thing the cut depends on. | Accepted. 6.1. |

## Appendix: Technical Names

Raspberry Pi 5, Pi 5, Raspberry Pi, bookworm, NVMe, PCIe, GPIO, PMIC,
STANDBY, EEPROM, `POWER_OFF_ON_HALT`, `WAKE_ON_GPIO`, `PSU_MAX_CURRENT`,
`usb_max_current_enable`, `rpi-eeprom-config`, `rtc_bbat_vchg`, J2, J5,
PWR_BUT, RTC, UART, UART0, `/dev/ttyAMA0`, `dtoverlay`, `uart0-pi5`,
`gpio-shutdown`, `gpio-poweroff`, `config.txt`, systemd, logind,
`HandlePowerKey`, `TimeoutStopSec`, `DefaultTimeoutStopSec`, `fsck`,
`dmesg`, `journalctl`, SIGKILL, `poweroff`, `sync`, `wall-e.service`,
`walle-ups.service`, `walle-ups-wakecheck.service`, `walle-power.service`,
`bin/walle_ups.py`, `bin/walle_power.py`, `i2c-bus-guard.service`,
`/tmp/ups_status.json`, `/debug`, `controller.py`, `config.py`, ch5,
e-stop, VESC, CAN, MCP2515, SPI0, Daly, BMS, 13S, 2S, 18650, Li-ion,
NMC, NCA, LiFePO4, OCV, SOC, CC/CV, CV, CC, NTC, OV, UV, OCD, XT60,
XT90, Molex Micro-Fit 3.0, JST-PH, Keystone 1043, D2PAK, SuperSO8,
SOIC, MSOP, HTSSOP, VQFN-HR, SOT-23-6, LQFP, TVS, SMCJ58A, SMCJ70A,
SMBJ18A, LT4363, LM5069, LTC4359, LM74700-Q1, IPB072N15N3,
BSC160N15NS5, BSC098N10NS5, BSC160N10NS3, BSC0902NS, LM5146-Q1, LM5145,
TEP 150-4815WI, TEP 150WI, TRACO, Mean Well SD-150C-12, TPS1HB08-Q1,
BTS7008-2EPA, PROFET, TE T9AS1D12-12, Omron G8P, LM61495, LM61495-Q1,
LMQ61460, TPS25982, TPS25221, TPS2561, USB5744, BQ24610, BQ24650,
BQ25713, BQ25792, BQ76905, S-8252, ABLIC, HY2120, HY2213, HYCON,
TL431, INA228, INA219, TLV7031, STM32G071, STM32G071CBT6, RP2040,
RP2350, TPS7B81-Q1, TPS7B69, 74LVC1G74, 2N7002, Samsung INR18650-35E,
Molicel P28A, Littelfuse, LP-JCASE, NANO2, OAK-D Lite, Luxonis,
`getUsbSpeed`, USB, USB 3, USB 2, USB-C, USB-A, VBUS, SuperSpeed, SSTX,
SSRX, D+, D-, Arduino, RC, RTK, GPS, L1, L5, NTP, TLS, journald, Grok,
Grok 4.7, `grok_second_opinion`, Kevin, WALL-E, Texas, Witty Pi 5,
UUGear, HAT+, StromPi 3, Joy-IT, Geekworm X1202, Geekworm UPS_Scap,
DFRobot FIT0992, Waveshare UPS HAT (B), Waveshare UPS HAT (E),
SunFounder PiPower 5, Sequent Microsystems, Super Watchdog, PiJuice,
Sfera Labs, Strato Pi UPS, mjbots, VBCores, ROS, AGV, Mean Well,
quarter brick, eFuse, ideal diode, surge stopper, hot-swap, NVDC,
OR-ing, SYS bus, holdup, brown-out, flapping, debounce, backoff,
heartbeat, watchdog, CRC-8, firmware, spin, BOM, PCB, four-layer,
2 oz, thermal via, conformal coat, creepage, SOA, RDS(on), EMI, PD,
Power Delivery, Stop mode, EXTI, ADC, I2C, SHDN, CE, FB, ISET, TS,
STAT, FAULT, SNS, EN, HOLD, VIN_OK, PI_HALTED, PI_5V_SENSE,
SHUTDOWN_REQ, EN_PI5V, OUT1, OUT2, P1, P2, P3, P4, HALT_NOW,
SHUTDOWN_PENDING, ARMED, DISARMED, TOPUP, HB, `walle-power/1`.
