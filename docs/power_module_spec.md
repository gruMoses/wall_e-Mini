# Pi 5 power module: hardware design specification

Status: research draft, 2026-11-04. Revised the same day for Kevin's
answers: a 13.2 V rail, peak ratings, smart FET outputs. Revised again
on 2026-11-04 for Kevin's choice of an off-the-shelf converter that sits
off the board. The rail is now a 12 V rail, and the board carries no
pack power. Nothing is ordered. No part is sampled. Each part number
below was checked on the manufacturer's page or a distributor page on
2026-11-04 unless the text says VERIFY. A VERIFY mark means the number
or the status comes from memory or from a third-party listing, and Kevin
must check it in the datasheet before he buys the part.

This document uses Simplified Technical English as far as is reasonable.
The appendix lists the Technical Names.

## 0. Summary

The module takes 12 V from an off-the-shelf converter that sits off the
board. The converter runs from the robot's 13S traction pack (37.7 V to
54.6 V). The module also senses the pack voltage on a thin lead. It
makes four things:

1. A 12 V rail, 150 W continuous, with two switched outputs that the Pi
   controls. The rail comes from an off-the-shelf potted golf-cart
   converter that is bolted to the chassis and rated for 72 V carts. The
   board takes the 12 V in and switches the two outputs. It carries no
   pack power.
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
what changed because of them, plus three more on the converter (11.4 to
11.6). The Murata quarter brick and the discrete 100 V buck stay in
section 3.2 as documented alternatives. The discrete buck is also the
product spin.

## 1. Requirements

Assumptions are marked (A). Kevin's words are quoted.

| # | Requirement | Source |
|---|---|---|
| R1 | Input: 13S Li-ion pack, 37.7 V (BMS cut) to 54.6 V (full). Design working input 60 V. | Kevin: "up to 60 V straight from the battery". `config.py` lines 225-242. |
| R2 | The board carries no pack power. The converter's own rating and a fuse at the pack cover the bus. The board's pack-sense lead survives 100 V. | Kevin, 2026-11-04: "we're going to use something off the shelf." Bus overvoltage: section 3.2a. |
| R3 | Fixed 12 V rail (12.0 V to 13.8 V, depending on the unit) from an off-board converter of 20 A class, 150 W continuous. Two switched outputs, each 10 A peak. 8 A combined continuous while the Pi runs. | Kevin: "a tap for 10 A at 14 V", "a relay to control the 12V output, or better yet, two". Kevin chose "peaks are fine" on 2026-11-04 (open question 2). He then chose the converter (open question 13): "we're going to use something off the shelf." |
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

Not requirements: galvanic isolation of the 12 V rail (the converter is
non-isolated, and its output negative is pack negative, section 3.2),
ride-through of the robot's loads, USB Power Delivery to the Pi.

## 2. Block diagram

```
 13S pack     +----------+   +-----------------------------------+
 37.7-54.6 V->| F1 fuse  |-->| 12 V converter, OFF THE BOARD     |
 (robot)      | at pack  |   | bolted to the chassis             |
     |        +----------+   | 72 V-class golf-cart unit, potted |
     |                       | 20 A class, 12.0 to 13.8 V        |
     |                       +-----------------+-----------------+
     | pack-sense lead                         |
     | (0.5 A fuse)                            | 12 V
- - -|- - - - - - - - - - - - - - - - - - - - -|- - - -  board edge
  +--v------------------+    +-----------------v-----------------+
  | VIN_OK divider      |    | 12 V IN (on the board)            |
  | 998k + 43k, clamp,  |    | F2 fuse 20 A, TVS SMBJ15A,        |
  | 100 nF, comparator  |    | LTC4368 and 2 x 100 V FETs,       |
  +---------------------+    | OV trip 15.0 V, UV trip 10.5 V    |
                             +-----------------+-----------------+
                                               | 12 V rail
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

### 3.1 Pack sense and 12 V input

The board carries no pack power. Two leads enter the board. One is a
thin pack-sense lead. The other is the 12 V lead from the off-board
converter (3.2).

Pack sense. The lead takes the pack voltage to the VIN_OK divider and
comparator (3.7).

| Item | Part | Key ratings |
|---|---|---|
| Sense-lead fuse | 0.5 A fuse at the pack end of the lead | DC rating 80 V or more (VERIFY the part). |
| Divider | 2 x 499 kOhm 1206 in series, then 43 kOhm to ground | The two resistors share the voltage. 80 V reads 3.3 V. |
| Clamp | Clamp diode from the divider node to 3.3 V | At 100 V the clamp carries less than 20 uA. The lead survives 100 V (R2). |
| Filter | 100 nF at the ADC pin | Time constant about 4 ms. |

12 V IN. The converter's 12 V lead enters the board here. It passes, in
this order, through the fuse F2, the TVS and the disconnect.

| Item | Part | Key ratings |
|---|---|---|
| Fuse F2 | Littelfuse 0895020.Z LP-JCASE 20 A, 58 V DC | At the board end of the 12 V lead. 58 V is above the 54.6 V pack maximum, so F2 also covers a converter that shorts its input to its output. VERIFY: for a 14S pack or for margin, use a fuse with a DC rating of 80 V or more. https://www.littelfuse.com/products/fuses/automotive-passenger-car/cartridge-fuses/895.aspx |
| TVS | Littelfuse SMBJ15A | 15 V standoff, clamp about 24.4 V (VERIFY). The clamp is below the 33 V absolute maximum of the BQ24610. |
| Disconnect controller | ADI LTC4368 | 100 V controller for back-to-back N-FETs, with reverse protection. Overvoltage trip 15.0 V. Undervoltage trip 10.5 V. VERIFY the variant and its set points. https://www.analog.com/en/products/ltc4368.html |
| Disconnect FETs | 2 x N-FET, 100 V, 5 mOhm or less, back to back | VERIFY the part, the gate drive and the heat at 10 A (about 1 W for the pair). |

Why the fuse and the TVS. A converter can fail with its input shorted
to its output. The pack voltage then reaches the 12 V line, and the
BQ24610 fails in tens of microseconds. A series FET alone can lose that
race. The TVS clamps the line first, and F2 opens. The disconnect FETs
are rated 100 V so that they survive pack voltage on the line while F2
opens (Grok, 11.5 and 11.6). VERIFY that the TVS survives until F2
opens (risk R-20, test T-18).

The disconnect also opens at 15.0 V and at 10.5 V. The 15.0 V
comparator on the rail is a second layer (3.2).

Product spin. The surge stopper of the first draft moves to the product
spin, where the converter is on the board. Its parts: ADI LT4363-2,
Infineon IPB072N15N3 G as the 150 V pass FET (alternative
BSC160N15NS5), Littelfuse SMCJ70A as the TVS, an 8 mOhm sense resistor,
and Littelfuse 0895020.Z or NANO2 0454010.MR as the input fuse. TI
LM5069-2 was the alternative stopper. It opens on overvoltage and does
not clamp. The first draft set an output clamp of 68 V. Kevin doubts
that the bus reaches 68 V (3.2a), so set the product-spin clamp from the
T-1 and T-2 measurements. The full first-draft table is in the git
history of this file.

### 3.2 12 V converter (off the board)

Kevin chose this on 2026-11-04: "we're going to use something off the
shelf." The 12 V comes from a potted golf-cart DC-DC converter that is
bolted to the chassis. The converter is not on the board. The board
takes its 12 V through the 12 V IN block (3.1). The Murata brick and the
discrete buck stay below as documented alternatives.

Why a bought unit (Grok, 11.1 and 11.6): the 100 V margin of a discrete
buck does not help when the front end is the real limit; a shorted
high-side FET puts the pack on the 12 V rail; a first hand-built buck
runs 90 to 93 percent, not 95; layout is the product. A bought unit is
potted and rated. It sits on the chassis, so its heat stays off the
board. A shorted high-side FET still puts the pack on the 12 V lead, so
the 12 V IN block (3.1) has a fuse, a TVS and a 100 V disconnect.

| Item | Part | Key ratings |
|---|---|---|
| Default | Off-the-shelf potted golf-cart converter, rated for 72 V carts | Example (VERIFY every number; seller data only): Vatrer 48V/72V to 12V golf cart converter, 25 A, 335 W. The product images say 40 V to 90 V input and 13.5 V output. IP55. About 30 USD (VERIFY). https://www.vatrerpower.com/products/48v-72v-to-12v-golf-cart-power-converter-25a-335w-dc-to-dc-voltage-reducer |
| Cheaper option | dkplnt "48V 36V to 12V 20A 240W" golf-cart converter (Amazon B08KDFQ4JT) and its clones | Seller data only: 30 V to 60 V input, fixed 12.0 V, 20 A, potted aluminium case, fuse, "up to 95 to 97 percent". About 25 USD (VERIFY). No datasheet, non-isolated buck, unit-to-unit variation unknown. Use it only if the bus measurements (T-1, T-2) show that the bus stays under about 58 V, and bench test T-17 passes. Its 60 V maximum is the weak point (3.2a). |
| Alternative brick, isolated | Murata HPQ-12/25-D48NB-C | Quarter brick, 36 V to 75 V input, 12 V 25 A, 300 W, isolated 2.25 kV, 94.5 percent, -40 C to +85 C. Active, in stock at DigiKey on 2026-11-04 (76.59 GBP at quantity 1). It could sit off the board on a bracket. Trim +10 percent gives 13.2 V; 25 A at 13.2 V is 330 W, so the brick is power-limited near 22 A, which is far above this design. Input undervoltage lockout, output OVP, hiccup overcurrent, over-temperature (series data). The 75 V input maximum gives less margin than a 72 V-class golf-cart unit (3.2a). VERIFY in the current datasheet: undervoltage shutdown (falling) and start-up threshold (a sibling HPQ model lists 32 V and 33 V), the trim range across 36 V to 75 V and temperature, the OVP threshold and whether it latches or hiccups, the input surge rating, the short-circuit mode, minimum load, baseplate temperature limit and whether the baseplate is isolated, and the external input and output capacitors. |
| Cheaper brick | Flex PKB4313DPI | Eighth brick, 36 V to 75 V, 12 V 25 A, 300 W, 96.2 percent, active. 56.78 USD, but none in stock and a 39-week lead time on 2026-11-04. VERIFY trim range. |
| Wide-input brick | Cincon CHB150W-48S12 | Half brick, 18 V to 75 V, 12 V 12.5 A, 150 W, 91 percent, trim about ±10 percent. 180 to 260 USD. Use it only if a converter's undervoltage floor is a problem (risk R-16). |
| Rejected | TRACO TEP 150-4815WI | 18 V to 75 V, 15 V 10 A, 150 W, 87 percent. The first draft priced it at 160 USD from memory. DigiKey lists it at about 268 USD with a 19-week lead time. It also makes 22 W of heat at full load, and a 15 V part may not trim down to 13.2 V. |
| Rejected | Murata MPQ1500-12V125-L48 | Checked 2026-11-04 from the datasheet (rev A01.D18). Quarter brick, 1500 W, 125 A, 97.2 percent peak, PMBus. Semi-regulated: below about 51 V input the output follows the input (9.6 V at 40 V, no load; 9.10 V to 12.10 V over 40 V to 52 V). The 13S pack is below 51 V for most of its discharge, so the rail would run about 9.5 V to 12 V. MFR_VOUT_MAX is 13.00 V, so 13.2 V is not possible. Operating input 40 V to 60 V, absolute maximum 65 V, input OVP 64 V: the pack's 37.7 V to 40 V tail is outside it, and the bus overvoltage would have to stay below 60 V. Current limit 137.5 to 162.5 percent of 125 A, that is 172 A to 203 A, on a 175 W design. Needs 270 uF of input capacitance. |
| Rejected | Mean Well SD-150C-12 | 36 V to 72 V input, 12 V 12.5 A, 77 percent. Rejected: 36 V minimum sits at the 37.7 V BMS cut, a fixed 12 V output, 77 percent and 35 W of heat. https://www.meanwell.com/productDetail.aspx?i=361 |
| Product spin, controller | TI LM5146-Q1 controller | 5.5 V to 100 V input, 0.8 V to 60 V output, synchronous buck. https://www.ti.com/product/LM5146-Q1 |
| Product spin, FETs | Infineon BSC098N10NS5 (low side), BSC160N10NS3 or similar (high side) | 100 V. Low side 9.8 mOhm. High side smaller: duty is 0.26, so switching loss dominates there (Grok). VERIFY high-side part. |
| Alternative controller | TI LM5145 | 6 V to 75 V. Less margin. |

Selection rules for any converter. VERIFY each rule against the seller
data, and then against bench test T-17:

- An input operating range that covers the pack under sag.
- An input maximum of 80 V or more (72 V class).
- A fixed output of 12 V to 13.8 V.
- An output of 20 A or more.
- A potted case.
- Hiccup short-circuit protection.
- A fuse at the pack end of its input lead (F1, below).
- A case that bolts to bare chassis metal.

WARNING: A converter with a 40 V input floor drops its rail when the
pack is low, because 40 V is above the 37.7 V BMS cut. The Vatrer
example has that floor. Do not depend on the 12 V rail when the pack is
below 40 V. The rail-lost rule in 5.3 keeps the Pi safe.

Rules for the converter, from the 2026-11-04 reviews (11.4 and 11.6):

- Fit a fuse at the pack end of the converter's input lead (F1). Use a
  DC rating of 80 V or more. Size it above the converter's full-load
  input current at the lowest pack voltage (about 9 A for 335 W at 40 V
  and 90 percent, VERIFY).
- The converter is non-isolated. Its output negative is pack negative.
  Do not add a second ground bond. Do not let USB or signal cables
  become a second bond.
- A short in the 12 V lead is upstream of F2. Size the lead for the
  converter's current limit, not for 10 A (VERIFY the limit). Rely on
  the converter's hiccup mode (risk R-18).
- Bolt the case to bare chassis metal. The screws carry the converter,
  not its leads.
- Route the leads away from the GPS and the OAK (risk R-15).

12 V rail protection. The 12 V IN block opens at 15.0 V (3.1). The
board adds an SMBJ18A TVS on the rail, and a 15.0 V comparator that
opens the LTC4359 (3.5) and OUT1/OUT2. VERIFY the converter's own OVP,
if it has one, and VERIFY that its output stays under 15.0 V in normal
use (13.8 V maximum).

Rules for the product spin, from the review: sense-resistor current
limit, not RDS(on); bias (VCC) back-fed from the output through a small
LDO so the controller does not burn 2 W from 54 V; copy the evaluation
module layout; an output overvoltage disconnect (a second LM5069 with
OVLO at 15.0 V) between the buck and the rail; a fixed output voltage,
not a range.

Thermal case, section 4.

### 3.2a Bus overvoltage is a robot-level problem

Kevin doubted the 68 V clamp of the first draft (2026-11-04): "I'm not
so sure that it's actually going to see 68 V. Wouldn't that be really
bad for the batteries?" He is right that the pack cannot reach 68 V.
Thirteen cells at 5.2 V would be a fire. The Daly BMS stops charge near
4.25 V per cell, about 55 V in all.

The bus goes above about 56 V only when the cells do not absorb the
energy. There are two cases:

1. A hot-plug ring. A hot-plug into ceramic input capacitors rings for
   microseconds, up to about twice the pack voltage.
2. Regen with no place to go. The VESCs brake and push current into the
   bus while the BMS charge FET is open. Or a faulted VESC coasts, and
   its MOSFET body diodes act as a rectifier. The second case needs a
   motor back-EMF above the bus voltage.

This affects the VESCs and every converter on the bus. It is not a
problem of this board alone. A surge stopper on this board would
protect only the converter. The actions:

1. Record the VESC hardware model and firmware version. The repository
   does not have them today.
2. VESC Tool 6.05 and later have "Battery Voltage Regen Cutoff Start/End" (off by default at
   1000/1100 V; research 2026-11-04). VERIFY the robot's firmware version, then set it near 55.5/57 V.
3. WARNING: "Battery Voltage Cutoff Start/End" in VESC Tool is the
   LOW-voltage cutoff. Do not set it near 56 V. The robot then stops
   driving.
4. Consider a chassis-mounted bus dump clamp: a FET, a power resistor
   and a trigger near 58 V. It sinks braking energy and protects the
   VESCs and the converter together. This is open question 16.
5. Buy converter voltage margin: a 72 V-class unit (3.2).

Tests T-1 and T-2 measure the bus (9.2).

### 3.3 Switched 12 V outputs (OUT1, OUT2)

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
| Relay, if isolation is required | TE T9AS1D12-12 or Omron G8P-1A4P 12 V | 30 A contacts. Coil at 12 V to 13.8 V is inside the 130 percent limit (VERIFY coil rating). |

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

The SYS bus feeds both 5 V bucks. It is the OR of the 12 V rail and
the buffer pack.

| Item | Part | Key ratings |
|---|---|---|
| 12 V to SYS ideal diode | ADI LTC4359 | 4 V to 80 V, 9 uA shutdown, SHDN pin with internal pull-up (floats on). https://www.analog.com/en/products/ltc4359.html |
| Pack to SYS ideal diode | TI LM74700-Q1 | 3.2 V to 65 V, 20 mV forward drop, 80 uA. https://www.ti.com/product/LM74700-Q1 |
| Both FETs | Infineon BSC0902NS or similar 30 V, 2 mOhm | VERIFY. |
| SYS holdup | 2 x 220 uF 25 V polymer | Covers the 12 V to 7.6 V hand-over. |

Behaviour. Input present: SYS = the 12 V rail, the pack diode is reverse
biased, the pack only sees the charger. Input lost: the 12 V rail
falls, the pack diode conducts at about 7.5 V, SYS follows. The bucks
accept 3 V to 36 V, so the 5 V rails do not see the step. There is no
NVDC charger and no gap.

Drain mode. The MCU pulls LTC4359 SHDN low. SYS falls to the pack.
The Pi runs from the pack with the charger disabled. The module uses
this to bring the pack from 8.40 V back down to 7.60 V after a top-up
(3.6), without a bleed resistor. If the MCU dies, SHDN floats high and
the 12 V path returns.

### 3.6 Charger, protector, balancing, monitor (the "custom BMS")

Lessons from the HAT (B) and how each is closed:

| Lesson | Fix |
|---|---|
| No charger IC; the supply set point was the charge limit. | A real CC/CV charger. Its float is set by resistors, 7.60 V. |
| Mains presence came from battery current. | Input presence comes from the pack-sense lead (3.7). |
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

The pack-sense divider (3.1; 80 V reads 3.3 V) feeds the MCU ADC and a
comparator with hysteresis (TI TLV7031, 335 nA). Thresholds: present at
38.0 V rising, lost at 34.0 V falling. Present sits just above the
37.7 V BMS cut. The divider draws from the sense lead only. With no
input it draws nothing from the buffer pack. The comparator edge wakes
the MCU from Stop mode.

The converter's own undervoltage floor may sit above the "lost" line
(40 V on the Vatrer example, VERIFY). The 12 V rail then drops while
VIN still reads present. The rail-lost rule (5.3) covers this case.

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
| 12 V IN | 2-pin screw terminal, 7.62 mm pitch | 20 A or more (VERIFY the part). Not an XT connector, so that the pack harness cannot plug into it. |
| PACK SENSE | 2-pin pluggable terminal | 0.5 A, 100 V or more (VERIFY). Carries the pack voltage to the divider (3.1). |
| OUT1, OUT2 | 5.08 mm screw terminal, 2 pin | 15 A. |
| Pi power | 6-wire harness to header pins 2, 4 (5 V) and 6, 9, 14, 20 (GND) | 3 A per pin, 6 A on 2 pins. Alternative: pogo pins to the Pi 5 bottom pads as the HAT (B) does. |
| Pi signals | 6-wire harness: 3V3 sense (pin 1), GPIO 14, 15, 27, 22, GND | Under 150 mm. |
| J2 | 2-wire to the Pi's J2 pads | Solder or 2-pin 1.0 mm JST. |
| USB injectors | 2 x USB 3 Type-A receptacle + 2 x USB 3 Type-A plug cable, 150 mm | 90 ohm pairs. |
| USB power outputs | 2 x USB Type-A receptacle, power only | D+ D- open. |
| Cells | 2 x 18650 holder, Keystone 1043 or spring holder | VERIFY contact resistance under 10 mOhm. Alternative: solder tabs on protected cells. |
| Fan (optional) | 2 pin JST-PH, 5 V | For the enclosure. |

Fusing: F1 at the pack end of the converter's input lead (off the
board, 3.2). A 0.5 A fuse in the sense lead, and F2 on the 12 V IN
(3.1). The 12 V outputs are protected by the high-side switches. The 5 V
rails are protected by the eFuse and the port switches. The pack path
is protected by the protector FETs and a 5 A polyfuse in the cell lead
as a second layer.

## 4. Power budget, efficiency, thermal

### 4.1 Loads

| Load | Typical | Peak |
|---|---|---|
| Pi 5 + NVMe, control stack | 6 W | 12 W (boot, 2.5 A) |
| OAK-D Lite | 4 W | 6.5 W |
| Hub, RC receiver, Arduino | 2 W | 3 W |
| Spare USB port | 0 | 7.5 W |
| 5 V side total | 12 W | 29 W |
| At 12 V after 92 percent bucks | 13 W | 32 W |
| Charger at 1 A into 8.4 V, 90 percent | 0 (float) | 9.5 W |
| OUT1 + OUT2 (future lidar, lights) | 0 today | 72 W (6 A) |
| Module housekeeping | 1 W | 1 W |
| 12 V rail total | 14 W (1.2 A) | 115 W (9.6 A) |
| Converter input at 90 percent, 37.7 V | 16 W (0.41 A) | 128 W (3.4 A) |

The 10 A tap peak (120 W on the outputs alone) plus the Pi and the
charger is about 165 W. The 20 A class converter carries it. The limit
is heat, not the converter rating. Kevin chose "peaks are fine"
(2026-11-04), so the outputs may draw 10 A combined for 60 s and 8 A
continuous, and the charger pauses when the rail current passes 9 A.

### 4.2 Heat

The converter is off the board. Its heat goes into the chassis through
its case. The board has its own, smaller heat.

Board:

| Source | Typical | Worst |
|---|---|---|
| Two 5 V bucks at 92 percent | 1 W | 2.5 W |
| Charger | 0 | 1 W |
| OUT1 + OUT2, 8 mOhm each | 0 | 0.4 W |
| 12 V IN FETs and fuse, about 14 mOhm | 0.02 W | 1.4 W |
| Total, board | 1 W | 5 W |

The 5 V bucks, the charger and the output switches give about 3 to 4 W
worst case. The 12 V IN path adds about 1.4 W at the full 10 A rail
current. Typical WALL-E operation is near the typical column. The worst
column is the future-loads case.

Converter, on the chassis. Plan on 90 percent efficiency.

| Source | Typical (14 W out) | At 150 W out |
|---|---|---|
| Converter at 90 percent | 2 W | 17 W |

If an isolated brick sits in its place, Grok puts the Murata brick at 9
to 13 W at 150 W, and near 20 W at 300 W (11.4).

Thermal path. Bolt the converter to bare aluminium that is part of the
robot chassis, with its case flat on the metal. Anodize or powder coat
under the case is an insulator. A 3 mm aluminium plate of 150 x 150 mm
in still air is roughly 3 K/W (VERIFY on the bench, T-3). At 17 W that
is a 51 K rise, which is too much at 45 C ambient. Use a larger chassis
area, and measure the case temperature in T-3 against the unit's limit
(VERIFY). A chassis in the Texas sun runs hotter than 45 C. The
converter has its own thermal protection (VERIFY). The firmware reads
the board thermistor and trims the output limits when the board is hot.
The 5 V bucks dissipate 1 to 2.5 W on the PCB copper; a 4-layer board
with 2 oz outer copper and thermal vias is enough.

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
| RUNNING | 12 V rail under 10.5 V for 10 s, VIN present | SHUTDOWN_REQ |
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
| any | 12 V rail over 15.0 V | FAULT (OV): open LTC4359, OUT1/2 |

### 5.3 Flapping input, brown-out, a Pi that never halts

Flapping. A loss shorter than 2 s is ignored. Once SHUTDOWN_REQ is
asserted, the shutdown completes; an input return does not cancel it.
The return debounce grows: 5 s normally, 60 s after the third loss
within 10 minutes, 300 s after the sixth. The counter clears after 1 h
of stable input. The boot window (first 30 s after APPLYING) requires
10 s of loss instead of 2 s, so that boot inrush sag cannot start a
shutdown.

Brown-out. Between 34 V and 38 V the module logs `degraded` and does
nothing else. The converter runs to its own undervoltage floor (VERIFY;
40 V on the Vatrer example). A pack at 40 V is nearly empty, about 3.1 V
per cell. The VESC pack-low logic in `config.py` (39 V for 30 s) cuts
the motors near that voltage. A converter floor above 39 V can drop the
12 V rail first. The rail-lost rule below covers that case.

Rail lost with the input present. Motor sag on a low pack, a failed
converter, a blown F2 or an open LTC4368 can stop the 12 V rail while
VIN still reads present. The Pi does not notice: SYS falls to the buffer
pack. If the rail stays below 10.5 V for 10 s with VIN present, the
module treats it as an input loss and runs the shutdown. A shorter
dropout only switches OUT1 and OUT2 off; the Pi turns them on again.

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
| LTC4359 SHDN | floats high | 12 V path on |
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
quotes. The Murata and TRACO prices in 3.2 were checked on DigiKey on
2026-11-04. The first draft carried the TRACO TEP 150 at 160 USD from
memory; its real price is about 268 USD. The price of the off-the-shelf
converter (about 30 USD) is seller data (VERIFY).

| Block | Parts | Estimate USD |
|---|---|---|
| Pack sense lead, divider, clamp | | 2 |
| 12 V converter, off the board, 72 V-class golf-cart unit | | 30 (VERIFY) |
| 12 V IN: fuse, SMBJ15A, LTC4368, 2 FETs | | 10 |
| OUT1/2: 2 x TPS1HB08-Q1, terminals | | 10 |
| 5 V: 2 x LM61495, inductors, caps, TPS25982 | | 16 |
| USB: 4 x TPS25221, 2 USB 3 receptacles, 2 cables, 2 USB-A | | 22 |
| Power path: LTC4359, LM74700, FETs, polymer caps | | 10 |
| Charger: BQ24610, FETs, inductor, sense | | 10 |
| Protector, balancers, FETs, polyfuse | | 5 |
| INA228, NTC, shunt | | 6 |
| MCU, LDO, comparator, LED, button, buzzer, latch | | 8 |
| Connectors: terminals, Micro-Fit, harnesses, JST | | 15 |
| Passives, TVS on rails | | 12 |
| 4-layer PCB, 5 pieces, 100 x 120 mm | per board | 15 |
| Cells, 2 x INR18650-35E, holders | | 14 |
| Total, first board | | about 185 |
| Product spin: discrete buck and its input protection back on the board | | about 200 |

A product at 100 pieces lands near 120 to 140 USD in parts with the
discrete buck. The first board costs about 80 USD less than the first
draft (about 265 USD), because the converter is a bought unit off the
board and the board carries no pack power. A product needs the
converter inside its own enclosure, so the product spin puts the
discrete buck and its input protection back on the board.

## 8. Build or buy

Products checked on 2026-11-04 (vendor pages or retailer listings):

| Product | Input | Battery | Pi 5 | Gap |
|---|---|---|---|---|
| UUGear Witty Pi 5 HAT+ | 6 to 30 V DC or 5 V USB-C, 5 A out | none | yes | No battery, 30 V maximum, no 12 V-class tap. Has voltage-triggered safe power-off and RP2350. Closest "wide input + safe shutdown" board. |
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
daemon would sell in the hundreds, not the thousands. The 12 V-class tap and
the switched outputs are what the competitors lack most.

## 9. Risks and bench test plan

### 9.1 Risks

| # | Risk | Mitigation |
|---|---|---|
| R-1 | Hot-plug ring: microseconds above the pack voltage, up to about twice it with ceramic input capacitors. It can damage the converter and the VESCs. | Robot-level problem (3.2a). Converter voltage margin (72 V class). T-1 measures the ring at the converter input. |
| R-2 | Regen overvoltage while the BMS charge FET is open, or from a faulted VESC that coasts and rectifies. The cells do not absorb it. | Robot-level problem (3.2a). VERIFY the VESC high-side limit. Consider a bus dump clamp. T-2, started at low speed. |
| R-3 | Brick option only. +10 percent trim is the brick's limit, not a set point. Tolerance and temperature put the rail near 13.2 V, not on it, and the brick's OVP may sit within a volt. | VERIFY the trim range, the OVP threshold and its mode. Sense at the brick pins. Fallback: trim to +8 percent (13.0 V). |
| R-4 | The converter dissipates about 17 W at 150 W out, on the chassis, in the sun. | T-3 thermal soak of the converter on the chassis. VERIFY the unit's temperature limit and its derating. |
| R-5 | The Pi rail does not collapse (sneak feed) and a re-apply does not boot the Pi. | Discharge FET, collapse check, apply check, 5 retries. T-6. |
| R-6 | SHUTDOWN_REQ asserted during boot loops the Pi. | Deassert before apply; assert only after the heartbeat. T-7. |
| R-7 | Hard cut corrupts the NVMe root. | systemd timeouts; consider a read-only root (open question 9). T-8. |
| R-8 | Boot inrush through the eFuse browns out mid-boot. | Slew control, 8 A limit, 30 s boot blanking. T-5. |
| R-9 | A USB 3 injector degrades the OAK link to USB 2 speed. | T-10. Fallback: a Luxonis Y-cable from port P1. |
| R-10 | Charger lifecycle (BQ24610 RGET listed obsolete by one distributor). | VERIFY at TI; BQ24650 fallback. |
| R-11 | Top-up leaves the pack at 4.2 V per cell if the drain mode fails. | Drain mode is MCU-driven; a failed MCU floats SHDN high, the charger CE stays on at 7.60 V, and the pack self-discharges slowly. Log and alarm. T-13. |
| R-12 | Cell holder contact resistance adds to cell resistance. | T-14 measures the whole string. |
| R-13 | J2 pulse width wrong: ignored, or a forced power-off. | T-9. |
| R-14 | Condensation on the 54 V pack-sense tracks. | Conformal coat except thermal pads and connectors. Creepage 2 mm minimum around the sense divider. |
| R-15 | EMI from the converter and its leads next to the OAK and the GPS. | Mount the converter away from the GPS. Twist the 12 V leads. T-15 sniff with the converter running. T-17 repeats it for the chosen unit. |
| R-16 | The converter's input floor sits above the BMS cut (40 V on the Vatrer example, 36 V on the HPQ). Motor sag on a low pack can take its input below the floor, and the 12 V rail drops out mid-throttle. | The Pi rides on the buffer (rail-lost rule, 5.3). A pack at 40 V is nearly empty. The VESCs stop at 39 V. If the floor is a problem, choose a unit with a lower floor (Cincon CHB150W-48S12 class, 18 V to 75 V) before changing the topology. |
| R-17 | Not applicable to the off-board design: no surge stopper is on the board. In the product spin, a shorted stopper FET passes the TVS clamp (about 113 V) to the converter. | Product spin only: a fuse or a second clamp on the converter side. |
| R-18 | The converter's current limit into a partial short in its 12 V lead cooks the lead or a joint, if the limit is constant-current. | Select a unit with hiccup mode (3.2). Size the lead for the converter's current limit. T-17. |
| R-19 | No load on the converter (both outputs off) makes some units unstable or trip OVP. | VERIFY "no minimum load" at its output voltage (T-17, no-load output). The charger and the SYS bus are a small load. |
| R-20 | A converter fails with its input shorted to its output. The pack voltage reaches the 12 V line, and the BQ24610 (33 V maximum) fails in tens of microseconds. | F2 and the SMBJ15A TVS act first (clamp about 24.4 V). The LTC4368 and its 100 V FETs open at 15.0 V. VERIFY that the TVS survives until F2 opens. T-18. |

### 9.2 Bench test plan, before the board goes on the robot

| # | Test | Pass |
|---|---|---|
| T-1 | Hot-plug the real pack into the converter's input through the real harness, 10 times. Scope at the converter input. | Record the peak. No part on the bus is above 80 percent of its rating. Fail: damp the converter input (a bulk capacitor with ESR, or a pre-charge). |
| T-2 | WARNING: Do not brake hard on a full pack with the BMS charge path open until a bus clamp is fitted or the VESC limit is confirmed. Robot on stands, both VESCs. Start with brake captures at low speed with the pack at mid charge, then step up the speed. Scope at the converter input. | Record the bus peak at each step. The peak stays under about 58 V. The converter stays up. The log shows no `INPUT_LOST`. |
| T-3 | 150 W resistive load on the 12 V rail for 2 h, with the converter bolted to its chassis spot and the board in its enclosure, at 45 C ambient. | Converter case under its limit (VERIFY); board under 85 C; 5 V rails in tolerance. |
| T-4 | 12 V rail: load step 0 to 8 A, short on OUT1 with OUT2 loaded. | Pi rail unaffected; OUT1 limits and retries; OUT2 stays on. |
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
| T-15 | Near-field probe over the board and the converter leads, OAK, GPS and the converter running. RTK fix quality before and after. | No new spurs at the GPS L1/L5 bands; fix quality unchanged. |
| T-16 | Idle current from the pack after a cut, input absent, 24 h. | Under 0.3 mA; pack voltage change under 5 mV. |
| T-17 | Chosen converter, before it goes on the robot: no-load and 10 A output voltage; real undervoltage shutdown and overvoltage limit; input at 60 V, 65 V and 68 V for 1 s (a 30 V to 60 V unit) or at 60 V, 70 V and 80 V for 1 s (a 72 V-class unit), from a current-limited supply; output short (which way it fails, what the output does); 10 A for 2 h with the case at 50 C; the RTK fix quality and a near-field probe with it running next to the GPS. | Output 11.6 V to 13.8 V; survives 68 V for 1 s (30 V to 60 V unit) or 80 V for 1 s (72 V-class unit); hiccup on a short; case under its rating; no change in the RTK fix. Any fail: choose another unit, or an isolated brick off the board. |
| T-18 | 12 V IN block on a bench fixture, without the rest of the board: apply 30 V, 54 V and 60 V to the 12 V input from a supply that gives 30 A, into a dummy load on the rail side. | F2 opens. The rail side stays under 33 V for the whole event. A TVS that fails short is acceptable. Nothing burns. |

## 10. Open questions for Kevin

1. Answered 2026-11-04: 13.2 V. Changed the same day: the off-the-shelf
   converter sets the rail at 12.0 V to 13.8 V. Check each future load's
   maximum: a 12 V part rated ±10 percent is at its limit at 13.2 V, and
   a 13.5 V rail (the Vatrer example) is above it.
2. Answered 2026-11-04: peaks are fine. The 20 A class converter has
   room for more if that changes.
3. Which loads go on OUT1 and OUT2 in the next year? Lidar (6 to
   13 W), lights, a heater, a horn? Their inrush sets the current
   limit.
4. Pack connector on the robot harness for the converter's fused input
   lead and the sense lead: XT60 or XT90? Do they hang behind the Daly
   discharge FET (so a BMS cut also cuts the module) or straight on the
   cells?
5. Answered 2026-11-04: smart FET (TPS1HB08-Q1).
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
    USB5744 hub on the module in the product spin?
12. The I2C bus guard uses the UPS status file as its witness. Move
    the witness to the GPS reader, or let the new daemon write a
    heartbeat file?
13. Answered 2026-11-04: off the shelf, the converter sits off the
    board.
14. Where does the converter mount on the chassis, and is there bare
    metal under it with a path to the chassis skin? Where does the
    module mount, and how long is the 12 V lead?
15. Which VESC hardware model and firmware version are on the robot?
    Does that firmware have a high-side regen voltage limit (3.2a)?
16. Fit a bus dump clamp on the robot (3.2a)?
17. Which converter: a 72 V-class unit (recommended), or the dkplnt
    after T-17 passes?

## 11. Appendix: Grok second opinions

Six `grok_second_opinion` calls, Grok 4.7, 2026-11-04, single shot, no
repository access. Sessions `01a1a72a`, `01a1a730`, `01a1a737` (11.1 to
11.3, 0.51 USD in all) and the three below (11.4 to 11.6, 0.37 USD).

### 11.1 The 14 V converter

History: this review was for the 14 V rail of the first draft. The
13.2 V brick review is 11.4.

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

### 11.4 The 13.2 V brick (2026-11-04)

One `grok_second_opinion` call, Grok 4.7, medium effort, session
`01a1a752`, 0.14 USD. Proposed: the Murata HPQ-12/25-D48 trimmed to
13.2 V. Against: the TRACO TEP 150, the discrete LM5146, and a cheap
potted 30 V to 60 V module. Grok agreed with the Murata and listed the
holes.

| Objection | Response |
|---|---|
| The 36 V input floor clips the rail under motor sag on a low pack; the TRACO's 18 V floor was the feature given up. | Accepted as risk R-16. Thresholds moved to 38 V present and 34 V lost, a rail-lost rule added (5.3), the Cincon 18 V to 75 V brick kept as the fallback. |
| +10 percent trim is the stop, not a set point; the brick's OVP may be referenced to 12 V and sit within a volt. | Accepted. R-3, the 15.0 V comparator note and the sense rule in 3.2. |
| A 12 V load rated ±10 percent is at its ceiling at 13.2 V. | Accepted. Open question 1 note. |
| 8 W is an average; 10 A on both outputs reaches 300 W and about 20 W; baseplate, not ambient, is the limit. | Accepted. Section 4.2, R-4, T-3. |
| The TVS clamp can exceed the brick's surge rating if the stopper FET fails short. | Accepted. R-17 and the 3.2 rule. |
| A surge stopper in front of an undamped brick input can oscillate. | Accepted. The 3.2 rule: use the datasheet capacitors and damper. |
| A 25 A brick on a board sized for 10 A; constant-current into a partial short cooks it. | Accepted. R-18 and the 3.2 rule. |
| The isolated output needs one deliberate ground bond. | Accepted. The 3.2 rule. |
| Some bricks are unstable with no load. | Accepted. R-19. |
| Do not build the LM5146 first; the TRACO is a weak fallback. | Accepted. The LM5146 stays spin 2; the TRACO is rejected. |

### 11.5 The cheap off-board converter (2026-11-04)

One `grok_second_opinion` call, Grok 4.7, medium effort, session
`01a1a764`, 0.12 USD. Proposed: the dkplnt 20 A golf-cart converter
off the board, the surge clamp lowered to 57-58 V to protect its 60 V
input, and a 15 V overvoltage disconnect after it. Grok rejected it
for spin 1.

| Objection | Response |
|---|---|
| A 57-58 V clamp sits about 3 V above a full pack; the stopper FET then absorbs ordinary regen braking on a full pack until its timer opens, which is how these FETs fail short. | Accepted. The clamp stays at 68 V. The converter must survive 68 V on the bench (T-17). Open: the VESCs' own bus limit is not recorded in the repository; if they fault near 57 V, regen cannot hold the bus above the clamp. |
| A series disconnect can lose the race to a shorted high-side FET; the BQ24610 (33 V) dies in tens of microseconds. | Accepted. Fuse and a TVS under 28 V before the disconnect. |
| Low-side or double FET short, overshoot and hiccup are not covered; no datasheet means no spec and no test limit. | Accepted. T-17 measures them; the Murata stays the default. |
| A non-isolated buck ties logic ground to the motor return; the RTK and the OAK sit on it. | Rejected in part. The Pi is already tied to pack negative through the CAN link to the VESCs, and section 3.2 bonds the brick output to pack negative anyway. The switching noise and the 20 A lead loop stay a real risk; T-17 checks the RTK fix. |
| Plan on 90 percent, not 95 to 97; 20 A is often a peak rating. | Accepted. T-17 runs 10 A for 2 h at a 50 C case. |
| 65 USD is less than one respin; spin 1 should be the boring, documented supply. | Accepted. The Murata stays on spin 1; the 12 V IN connector lets the cheap converter be tried without a respin. |

### 11.6 Off the shelf, no surge stopper (2026-11-04)

One `grok_second_opinion` call, Grok 4.7, medium effort, session
`01a1a779`, 0.11 USD. Proposed: the converter on the pack with a fuse
and a TVS, the stopper dropped, and the VESC regen cutoff set near 56 V
and 57.5 V. Grok agreed with dropping the stopper and rejected the rest.

| Objection | Response |
|---|---|
| The VESC "Battery Voltage Cutoff Start/End" is the low-voltage cutoff, not a regen limit. | Accepted. WARNING added in 3.2a. VERIFY whether the firmware has a high-side limit. |
| A faulted, coasting VESC rectifies through its body diodes, and no setting stops that. | Accepted as a robot-level risk (3.2a). It needs a motor back-EMF above the bus voltage, which the robot's `max_erpm` 20000 cap may prevent. VERIFY against the motor KV. |
| Golf-cart converters assume a pack that stays connected. A steady bench test says nothing about a 2x ring. | Accepted. A 72 V-class unit is the default (3.2). |
| A TVS covers the hot-plug ring, not braking energy. | Accepted (3.2a). |
| Keeping the stopper protects only the converter, never the VESCs. | Accepted. The stopper is dropped from the board (3.1). |
| The halt must also start on loss of the 12 V rail. | Already in the design: the rail-lost rule (5.3). |
| The overvoltage disconnect must withstand pack voltage if the converter shorts its input to its output. | Accepted. 100 V-rated parts, and F2 coordinated with the TVS (3.1, R-20, T-18). |
| Buy voltage margin or add a dump clamp. | Accepted. A 72 V-class unit is the default. The clamp is open question 16. |
| Do not run T-2 on a full pack to discover this. | Accepted. T-2 is rewritten with a WARNING and a low-speed start. |

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
TEP 150-4815WI, TEP 150WI, TRACO, Murata, HPQ-12/25-D48NB-C, MPQ1500-12V125-L48,
PMBus, MFR_VOUT_MAX, dkplnt, LTC4368, 12 V IN, Vatrer, SMBJ15A, 1206,
IP55, golf-cart converter, potted, hiccup, regen, back-EMF, body diode,
MOSFET, bus dump clamp, VESC Tool, `max_erpm`, KV, pack sense, F1, F2,
Flex,
PKB4313DPI, Cincon, CHB150W-48S12, eighth brick, half brick, baseplate,
OVP, UVLO, Mean Well SD-150C-12, TPS1HB08-Q1,
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
