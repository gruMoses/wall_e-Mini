# Pi 5 power module: hardware design specification

Revision note, 2026-11-04 (option A). This revision follows option A of
`docs/pi_power_architecture.md`. That document is the decision document.
If the two documents disagree, it wins. The Pi now gets 5.1 V from an
isolated 75 V / 100 V-surge brick on the battery side of the main switch,
behind a fuse and a high-side switch. An opto ignition sense on the
switched side, a hardware grace timer and a 5 V bleed control it. The 2S
18650 buffer, its charger, protector and balancers, the INA228, the SYS
bus, the pack-sense lead and the RTC and J2 wake paths are removed. Power
returns only when the ignition comes on again. The 12 V converter and
outputs, the USB rail, the MCU, the Pi interface and the bus-overvoltage
material (3.2a) stay. New fact: both drive VESCs read back as hardware
`75_100_V2`, firmware 6.05. The buffer design is in the git history of
this file (commit 037cc80).

Review note, 2026-11-05 (hardware review of option A). This review
changed these items, most serious first:

1. Bus overvoltage (3.2a, R1, R-28). The VESCs fault only at 90 V and
   their regen cutoff is off. A full-pack stop with the BMS charge FET
   open can then hold the bus above the SMBJ58A breakdown (64.4 V
   minimum). The TVS then tries to clamp the whole bus, burns, and
   blows F3: a hard cut of the Pi while the robot moves. The brick also
   sees more than 75 V for more than 100 ms. Section 3.2a now lists the
   conditions that must be true before the brick goes on the bus.
2. Heat defeats the halt and the backstop together (3.6, 3.7, R-29).
   The TLP293 dark current is 50 uA maximum at 85 C. Through the Pi's
   weak pull-up, IGN_OFF then reads "ignition on". Through the 47 kOhm
   pull-down, the counter stays in reset. The Pi never halts and the
   timer never expires. Both resistors are now 10 kOhm.
3. Hot-plug turn-on of the HSS (3.5, R-27). The gate-to-drain slew
   capacitor and the 12 V Zener turn the P-FET on when the pack is
   connected with the ignition off. A gate-to-source capacitor now
   holds it off.
4. Brick facts from the TRACO datasheet (3.4, 4.1, 4.2). The no-load
   input current is 65 mA at 48 V, about 3.1 W. The brick runs near
   68 percent at the Pi's typical load, not 88 percent. The board heat
   is about 4 W typical, not 1.8 W.
5. The halt budget (4.4, 6.1). `wall-e.service` sets its own
   `TimeoutStopSec=90`, so the 30 s system cap does not bound it. The
   daemon now forces the poweroff at 35 s.
6. Smaller items: a leakage bleed across the brick input (R-31), gate
   pull-downs on Q1 and Q2, the timer capacitor fixed at 39 nF with the
   vendor constants (3.7), an MCU check of the timer clock and a stuck-on
   detector (R-26, R-30), the storage arithmetic (4.3: 3 mA off current
   on the 7 Ah pack gives about 6 weeks), the STM32G071CBT6 package
   (48 pins, not 64), wire gauges (3.12), new risks R-26 to R-31, new tests T-21
   and T-22, open questions 21 to 25, and a Grok review (11.8).

Update, 2026-11-05 (after `docs/battery_settings.md`, commit 9c5daff).
The pack is 13S NMC, 7 Ah. The BMS (JHB-R24TH-V1.3) trips its
overvoltage protection at the end of every full charge, and its charge
low-temperature protection (1 C) and its 40 A charge over-current
protection also open the charge FET. Those are the conditions where
regen has no path. Section 3.2a now refers to the VESC and BMS targets
of that document in place of the 55.5/57 V regen-cutoff numbers, and
records the right VESC's 0.9 V reading offset. The `l_max_vin`
proposal stays an open question with its brake-loss trade-off (open
question 22). The storage table (4.3), the grace-window energy (4.4)
and open questions 15, 20 and 24 now use these facts.

Status: research draft. Nothing is ordered. No part is sampled. Each
part number below was checked on the manufacturer's page or a
distributor page on 2026-11-04 unless the text says VERIFY. A VERIFY
mark means the number or the status comes from memory or from a
third-party listing, and Kevin must check it in the datasheet before he
buys the part. The brick data in 3.4 was checked on 2026-11-05 in the
TRACO TEN 30WIN and TEN 20WIN datasheets (revision 8 October 2026). No
part is checked on the bench. On 2026-11-05 these datasheets were also
read: Vishay IRF9640 and VOM1271, TI CD74HC4060, Toshiba TLP293, and
distributor data for the Littelfuse SMBJ58A. Each number from them
carries its URL. The other parts that option A adds come from memory,
and each carries a VERIFY mark.

History: the first draft (2026-11-04) had a 14 V rail, a converter on
the board and a 2S buffer. The same day it changed to a 13.2 V rail,
peak ratings and smart FET outputs, then to an off-the-shelf 12 V
converter off the board, then to option A (this revision).

This document uses Simplified Technical English as far as is reasonable.
The appendix lists the Technical Names.

## 0. Summary

The module powers the Raspberry Pi 5, the USB devices and two switched
12 V outputs on WALL-E. The robot's 13S traction pack (37.7 V to
54.6 V) is the only energy source. There is no buffer battery.

Three supply leads come to the module:

- The battery tap. It comes from the BMS output on the battery side of
  the main switch, through fuse F3. It feeds only the Pi brick.
- The ignition lead. It comes from the switched side of the main
  switch, through fuse F4. It carries only an opto LED current (5 mA to
  8 mA).
- The 12 V lead. It comes from an off-the-shelf 12 V converter that is
  bolted to the chassis and runs from the switched side through F1.

The module makes four things:

1. A 5.1 V rail for the Pi 5 through the GPIO header, 6 A. An isolated
   telecom brick (TRACO TEN 30-4811WIN class) makes it from the battery
   tap. A high-side switch (HSS) in front of the brick is on while the
   ignition is on, and for a grace window after the ignition goes off.
2. An ignition signal to the Pi on GPIO 27. The Pi debounces it and
   halts itself. A hardware grace timer forces the HSS off 60 s to 90 s
   after the ignition goes off, even if the Pi never halts. A bleed then
   takes the 5.1 V rail to 0 V, so the next ignition is a real power-up
   and the Pi boots.
3. A 12 V rail, 150 W continuous, with two switched outputs that the Pi
   controls. The rail is off whenever the ignition is off.
4. A 5.0 V rail for the USB devices, from the 12 V rail, with four
   current-limited ports. The Pi's own USB ports do not carry the device
   power.

With the main switch off, the module takes only leakage from the pack.
The expected value is under 10 uA at 25 C. The datasheet maximums allow
more, so test T-16 measures it (4.3). The BMS itself takes about 0.5 mA
asleep (a Daly figure; VERIFY on the JHB-R24TH, 4.3).

WARNING: Do not connect the brick to the battery tap until the bus
conditions in 3.2a are true. With the VESC settings found on 2026-11-04
(fault at 90 V, regen cutoff off), a hard stop on a full pack can burn
the tap TVS and cut the Pi while the robot moves.

A small microcontroller (STM32G071) switches the outputs and the USB
ports, and reports the rails to the Pi over UART0 (GPIO 14 and 15). It
is not in the Pi power path. A dead MCU does not change when the Pi gets
power. It is not on the Pi's I2C bus 1.

Section 11 (the Grok appendix) records the second opinions. 11.1 and
11.4 to 11.6 still cover the 12 V converter and the bus. 11.7 records
the option A review. The Murata quarter brick and the discrete 100 V
buck stay in section 3.2 as documented alternatives for the 12 V rail.
The discrete buck is also the product spin.

## 1. Requirements

Assumptions are marked (A). Kevin's words are quoted.

| # | Requirement | Source |
|---|---|---|
| R1 | Input: 13S Li-ion NMC pack, 7 Ah, 37.7 V (design minimum; the BMS pack undervoltage protection is 40.0 V, `docs/battery_settings.md`) to 54.6 V (full). Design working input 60 V. Each part on the battery tap and the ignition lead survives 100 V for 100 ms. The robot holds the bus at or below about 61 V for any event longer than 100 ms (3.2a); until it does, the brick does not go on the bus. The 12 V converter follows 3.2. | Kevin: "up to 60 V straight from the battery". `config.py` lines 225-242. 100 V surge: `docs/pi_power_architecture.md` section 4. Bus limit: this review, 3.2a. |
| R2 | The board carries pack power only in the Pi power block: one battery tap of 2 A or less, fused at the tap, behind an isolation barrier (optos and the isolated brick). The 12 V converter stays off the board. | Option A. Kevin, 2026-11-04: "we're going to use something off the shelf." Bus overvoltage: section 3.2a. |
| R3 | Fixed 12 V rail (12.0 V to 13.8 V, depending on the unit) from an off-board converter of 20 A class, 150 W continuous, fed from the switched side. Two switched outputs, each 10 A peak. 8 A combined continuous. | Kevin: "a tap for 10 A at 14 V", "a relay to control the 12V output, or better yet, two". Kevin chose "peaks are fine" on 2026-11-04 (open question 2). He then chose the converter (open question 13): "we're going to use something off the shelf." |
| R4 | 5.1 V rail for the Pi 5, 6 A continuous, through the header. It comes from a brick rated 75 V continuous and 100 V for 100 ms, on the battery side of the main switch. | (A) Pi 5 needs 5 V 5 A to lift its USB limit. Brick class: option A, section 4. |
| R5 | 5.0 V rail for USB devices, 4 ports, each current-limited. No feed from the Pi's ports and no feed into them. | Kevin: "power everything we need to power over USB". `docs/NEXT_SESSION.md`: 167 USB over-current trips from hub backfeed. `docs/ups_hat_b.md`: the hub back-powered from the Pi's ports on the 2026-11-04 main-switch pull. |
| R6 | No buffer battery. The main pack carries the Pi through the halt. | Option A. The 2S buffer drains in storage (`docs/pi_power_architecture.md` section 1). |
| R7 | The ignition is sensed from the switched side only, through an opto whose LED has its own fuse. No part may be able to bridge the main switch. | Option A, section 4. The main switch must still make the drivetrain dead. |
| R8 | Ignition off: the Pi halts itself inside the grace window, then the 5.1 V rail goes off. Ignition on: 5.1 V is applied and the Pi boots. Power returns only with the ignition. | Kevin: "safely shut down the Pi", "the Pi should boot automatically". Option A. |
| R9 | A hardware grace timer forces the Pi rail off 60 s to 90 s after the ignition goes off, even while the Pi's 3V3 is up. No firmware is in this path. | Grok, option A review (11.7): an `ignition OR 3V3` enable lets a hung Pi drain the pack in one to two weeks. |
| R10 | The module interface cannot hang the Pi's I2C bus 1. | `docs/i2c_bus_guard.md`. |
| R11 | With the main switch off, this design takes under 50 uA from the pack (target 10 uA). The brick is disconnected, not only switched off. | A brick's 3 mA off current is about 2 Ah a month (option A, section 4). |
| R12 | The module tells the Pi its state, voltages and currents, and accepts output commands. | (A) replaces `bin/walle_ups.py`. |
| R13 | A dead microcontroller does not change the Pi power path. Its safe state: switched outputs off; USB ports as open question 10 decides. | Grok objection, section 11. Option A puts the timer in hardware. |
| R14 | Outdoor enclosure, ambient -10 C to +45 C, no forced air. | (A) WALL-E lives outside in Texas. |
| R15 | One person hand-assembles the first boards. Four-layer PCB, stencil, hot plate or oven. | Kevin builds it himself. |
| R16 | When the HSS opens, the 5.1 V rail falls under 0.5 V within 2 s. When it closes, the rail rises in tens of milliseconds. | A slow 5 V ramp can leave the Pi 5 in standby (`docs/pi_power_architecture.md` section 2). |

Not requirements: galvanic isolation of the 12 V rail (the converter is
non-isolated, and its output negative is pack negative, section 3.2);
ride-through of the robot's loads; USB Power Delivery to the Pi; a
scheduled wake in storage (open question 8); a clean halt on a BMS cut,
a blown fuse, a converter fault or a loose connector. Those stay hard
cuts (option A, section 4; risk R-7).

## 2. Block diagram

Robot level:

```
            battery side            main switch           switched side
 13S pack -- BMS output --+-------------o/ o-------------+-- VESCs, drivetrain
                          |                              |
                       F3 2 A                            +-- F1 --+
                          |                              |        |
                          |                              |  +-----v--------------+
                          |                              |  | 12 V converter     |
                          |                              |  | OFF THE BOARD      |
                          |                              |  | 72 V class, potted |
                          |                              |  | 20 A, 12.0-13.8 V  |
                          |                              |  +-----+--------------+
                          |                        F4 0.25 A      |
                          | battery tap    ignition lead |        | 12 V lead
 - - - - - - - - - - - - -|- - - - - - - - - - - - - - - |- - - - | - - - - board edge
                 +--------v---------+     +--------------v--+  +--v----------------+
                 | Pi power block   |<-U1-| ignition sense  |  | 12 V IN: F2 20 A, |
                 | 3.4 to 3.8:      |<-U2-| 3.6: LED string |  | SMBJ15A, LTC4368, |
                 | HSS, brick, hold |     | U1, U2A, U2B    |  | 2 x 100 V FET     |
                 | path, grace      |     +--------+--------+  +--+----------------+
                 | timer, bleed     |              |              | 12 V rail
                 +--------+---------+              | IGN_OFF      +------+------+------+
                          | 5.1 V                  |              |      |      |      |
                          v pins 2, 4              v GPIO 27      v      v      v      v
                        Pi 5 <---------------------+           OUT1/2   USB    MCU   15 V
                          ^                                   TPS1HB08 buck    LDO   comp.
                          |                                      x 2   5.0 V (also on
                          |   UART0, PI_3V3                              |   5.1 V)
                          +--------------------> MCU                     |
                                                        TPS25221 x 4: OAK, hub, Arduino,
                                                        spare (USB 3 injectors, Pi-side
                                                        VBUS open)
```

Pi power block (3.4 to 3.8):

```
 IGNITION LEAD (switched side), 5 mA to 8 mA
   IGN+ -- F4 -- diode -- 2 x 3.3 kOhm -- U1 LED -- U2A LED -- U2B LED -- IGN-

 BATTERY SIDE (pack voltage)               ||  LOGIC SIDE (system ground)
                                           ||
 battery tap -- F3 --+-- SMBJ58A           ||
                     |                     ||
               +-----v------+              ||
               | HSS P-FET  |<-- Q1 <-- U1 ||
               | -150 V     |<-- Q2 <-- U3 <==== U3 LED <-- Q5 <-- PI_3V3, pin 1
               +-----+------+              ||                ^
                     |                     ||                Q6 <-- GRACE_EXPIRED
               +-----v------+              ||                             ^
               | Pi brick,  |              ||              74HC4060 timer, reset by U2B
               | isolated   |=====================> 5.1 V rail
               +------------+              ||       +--> output fuse, crowbar, bleed
                                           ||       +--> header pins 2, 4 --> Pi 5
                                           ||
                                           ||  U2A --> IGN_OFF --> GPIO 27

 U1 and U3 are PV optos: their outputs are on the battery side. U2A and
 U2B are phototransistor optos: their outputs are on the logic side.

 ENABLE (HSS on)  = IGNITION  OR  HOLD
 HOLD             = PI_3V3    AND NOT GRACE_EXPIRED
 GRACE_EXPIRED    = ignition off for T (75 s nominal, 60 s to 90 s)
 bleed on         = 5.1 V rail below about 4.6 V
```

## 3. Blocks and parts

Each block gives a recommended part and one alternative. Prices are
single-unit distributor estimates (section 7).

### 3.1 Leads into the board, and the 12 V input

Three supply leads enter the board.

| Lead | From | Fuse | Carries |
|---|---|---|---|
| Battery tap | BMS output, battery side of the main switch | F3: 2 A time-delay, DC rating 100 V or more, at the tap end (VERIFY the part and its DC breaking capacity against the BMS short-circuit current). TRACO recommends 3.15 A slow-blow for the TEN 30 48 V models and 2 A for the TEN 20, sized for full load at their 18 V floor. At 37.7 V the full-load input is under 1 A, so 2 A covers it. | The brick's input current: 0.26 A typical, 0.43 A peak at 37.7 V (4.1). The HSS turn-on inrush, about 1.5 A for 2 ms (3.5). Nothing in storage: the HSS is open. |
| Ignition lead | Across the switched side of the main switch | F4: 0.25 A, DC rating 100 V or more, at the switch end (VERIFY the part) | The opto LED current, 4.8 mA to 8.4 mA over 37.7 V to 60 V (3.6). |
| 12 V lead | The off-board 12 V converter (3.2) | F2 on the board (below) | Up to 10 A peak. |

WARNING: Key the battery-tap connector and the ignition connector
differently. If they are swapped, the ignition string runs from the
battery side and is always lit. The Pi then never halts, and it drains
the pack in storage. The brick then runs from the switched side, and
each switch-off is a hard cut.

Where to tap. Tap the BMS pack output (after its discharge FET), not the
raw cells, so that the BMS deep-discharge cut still protects the pack.
The BMS is a JHB-R24TH-V1.3 (Daly-app compatible,
`docs/battery_settings.md`). A BMS of this type usually switches the
negative lead. Then the battery tap is
pack positive before the main switch, and its return is the BMS output
negative (P-), not the cell negative (B-). VERIFY on WALL-E's BMS and
harness (open question 4, risk R-21).

The ignition lead is a pair taken across the switched side, after the
main switch. Its LED string is then lit only while the switch is closed,
whichever lead the switch interrupts.

The isolation barrier. Do not tie the battery-side negative to system
ground on the board. The only links across the barrier are the brick and
the optos (3.4 to 3.7). The design then works whether the main switch is
in the positive or the negative lead, and no part on the board can feed
the switched side (R7, test T-14). System ground is the negative of the
switched side: the 12 V converter's output negative, the Pi ground and
the CAN ground.

12 V IN. The converter's 12 V lead enters the board here. It passes, in
this order, through the fuse F2, the TVS and the disconnect.

| Item | Part | Key ratings |
|---|---|---|
| Fuse F2 | Littelfuse 0895020.Z LP-JCASE 20 A, 58 V DC | At the board end of the 12 V lead. 58 V is above the 54.6 V pack maximum, so F2 also covers a converter that shorts its input to its output. VERIFY: for a 14S pack or for margin, use a fuse with a DC rating of 80 V or more. https://www.littelfuse.com/products/fuses/automotive-passenger-car/cartridge-fuses/895.aspx |
| TVS | Littelfuse SMBJ15A | 15 V standoff, clamp about 24.4 V (VERIFY). The clamp is below the 36 V input maximum of the LM61495 USB buck, the weakest part on the rail (VERIFY its absolute maximum). |
| Disconnect controller | ADI LTC4368 | 100 V controller for back-to-back N-FETs, with reverse protection. Overvoltage trip 15.0 V. Undervoltage trip 10.5 V. VERIFY the variant and its set points. https://www.analog.com/en/products/ltc4368.html |
| Disconnect FETs | 2 x N-FET, 100 V, 5 mOhm or less, back to back | VERIFY the part, the gate drive and the heat at 10 A (about 1 W for the pair). |

Why the fuse and the TVS. A converter can fail with its input shorted
to its output. The pack voltage then reaches the 12 V line. The USB buck
(36 V) and the output switches (40 V) then fail in tens of microseconds.
A series FET alone can lose that race. The TVS clamps the line first,
and F2 opens. The disconnect FETs are rated 100 V so that they survive
pack voltage on the line while F2 opens (Grok, 11.5 and 11.6). VERIFY
that the TVS survives until F2 opens (risk R-20, test T-18).

The disconnect also opens at 15.0 V and at 10.5 V. The 15.0 V
comparator on the rail is a second layer (3.2).

Product spin. The surge stopper of the first draft moves to the product
spin, where the 12 V converter is on the board. Its parts: ADI LT4363-2,
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

The converter takes its input from the switched side of the main switch,
through F1. It has no remote-off pin, and it needs none: the main switch
turns it off, so it takes nothing from the pack in storage. The 12 V
rail, the USB rail and the outputs are therefore off whenever the
ignition is off. Nothing that must outlive the ignition hangs on them.
The Pi runs from the battery tap (3.4).

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
- A fuse at the switched-side end of its input lead (F1, below).
- A case that bolts to bare chassis metal.

WARNING: A converter with a 40 V input floor drops its rail when the
pack is low, because 40 V is at the 40.0 V BMS cut and above the
37.7 V design minimum. The Vatrer example has that floor. Do not depend on the 12 V rail when the pack is
below 40 V. The Pi does not depend on it (3.4). A rail drop takes the
USB devices and the outputs off. Section 5.3 says what follows.

Rules for the converter, from the 2026-11-04 reviews (11.4 and 11.6):

- Fit F1 at the switched-side end of the converter's input lead, near
  the main switch. Use a DC rating of 80 V or more. Size it above the
  converter's full-load input current at the lowest pack voltage (about
  9 A for 335 W at 40 V and 90 percent, VERIFY).
- The converter is non-isolated. Its output negative is its input
  negative: the switched-side negative, which is system ground (3.1).
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
switches OUT1, OUT2 and the USB buck off. VERIFY the converter's own
OVP, if it has one, and VERIFY that its output stays under 15.0 V in
normal use (13.8 V maximum).

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
Thirteen cells at 5.2 V would be a fire. The BMS stops charge at its
overvoltage protection. Today that is 4200 mV for a cell and 54.6 V for
the pack (`docs/battery_settings.md`, section 3).

The bus goes above about 56 V only when the cells do not absorb the
energy. There are two cases:

1. A hot-plug ring. A hot-plug into ceramic input capacitors rings for
   microseconds, up to about twice the pack voltage.
2. Regen with no place to go. The VESCs brake and push current into the
   bus while the BMS charge FET is open. Or a faulted VESC coasts, and
   its MOSFET body diodes act as a rectifier. The second case needs a
   motor back-EMF above the bus voltage. `config.py` records about
   22,000 eRPM per unit of duty (2026-09-19 run), so the back-EMF
   reaches the bus near 22,000 eRPM. That is only about 10 percent above
   `max_erpm` 20000 (VERIFY with the motor KV). A robot that rolls
   downhill, or that a person pushes, can pass that speed.

The charge FET is open more often than the first drafts assumed
(`docs/battery_settings.md`, findings 1, 3 and 9). The BMS is a
JHB-R24TH-V1.3 (Daly-app compatible, 24S 40 A hardware) on a 13S NMC
pack of 7 Ah. The charger is an XM2703, 54.6 V 4 A. These conditions
open the charge FET:

- The end of every full charge. Cell overvoltage protection (4200 mV)
  and pack overvoltage protection (54.6 V) are equal to the charger
  target, so the BMS trips at the end of each full charge. On
  2026-11-03 it reported `Cell OVP Trip` for 45 minutes. The first drive
  after a full charge therefore starts with the charge FET open.
- A hard stop. Each VESC allows -40 A of regen today (-80 A total), and
  the BMS charge over-current protection is 40 A.
- Cold. The BMS charge low-temperature protection is 1 C. Below it the
  charge FET opens, and driving still works.

The same document records that each VESC also allows +40 A of drive
(80 A total) against a 40 A discharge over-current protection. That trip
opens the discharge FET, which is a hard cut for the Pi (5.3).

This affects the VESCs and every converter on the bus. It is not a
problem of this board alone. A surge stopper on this board would
protect only the converter. The Pi brick sees the same bus: the main
switch is closed while the robot drives, so the battery tap and the
switched side are one node.

Measured on WALL-E, 2026-11-04 and 2026-11-05
(`docs/pi_power_architecture.md` section 4 and
`docs/battery_settings.md`; read-only CAN query with
`tools/vesc_info.py`, and a stand test with `candump`):

| Item | Left (id 2) | Right (id 1) |
|---|---|---|
| Hardware, firmware | `75_100_V2`, 6.05 | `75_100_V2`, 6.05 |
| `l_max_vin` (over-voltage fault) | 90 V | 90 V |
| `l_battery_regen_cut_start` / `_end` | 1000 / 1100 V (off) | 1000 / 1100 V (off) |
| `l_in_current_min` (regen into the pack) | -40 A | -40 A |
| `l_in_current_max` (drive from the pack) | 40 A | 40 A |
| `l_battery_cut_start` / `_end` (low-voltage cutoff) | 40.8 / 36 V | 40.8 / 36 V |
| `si_battery_cells` | 13 | 12 |
| Bus reading against the BMS cell sum | within 0.1 V | about 0.9 V high |

The right VESC reads the bus about 0.9 V high in every sample from
50.5 V to 54.5 V. The left VESC reads within 0.1 V of the BMS. Each
voltage threshold on the right VESC therefore gets +0.9 V
(`docs/battery_settings.md`, finding 7). The stand test (wheels free,
pack at 50.7 V, 48 Hz sampling) moved the bus at most 0.5 V above rest,
with regen up to -12.1 A: a pack resistance of about 42 mOhm. CAUTION:
that is the mild case. The wheels were free, the pack was not full, and
a 48 Hz sample cannot see a spike shorter than about 20 ms. Do not use
these numbers to size a part.

Note on the low-voltage cutoff. The VESC cutoff (40.8 V start, 36 V end)
ends below the BMS pack undervoltage protection (40.0 V), so the BMS
cuts the power first. `docs/battery_settings.md` section 4 gives the
targets for 13S. The BMS cut is a hard cut for the Pi (5.3). This spec
keeps 37.7 V as the design minimum of the input: it is lower than the
40.0 V BMS cut, so each current and drive number at 37.7 V is
conservative.

Why a stop with the charge FET open reaches the fault. The robot's
kinetic energy at 1.6 m/s is about 90 J at 70 kg (the mass that
`docs/battery_settings.md` uses; VERIFY). The VESC bus capacitors take
about 0.8 J from 55 V to 62 V if they total 2 mF (VERIFY the
capacitance). With the charge FET open, nothing else takes the energy.
A hard stop therefore drives the bus to the VESC over-voltage fault.
With `l_max_vin` at 90 V, that is up to 90 V for the length of the stop
(0.5 s to 1.6 s on the stand). The regen cutoff is a filtered software
current limit. With today's -80 A of regen into a few millifarads, the
bus moves by tens of volts per millisecond, faster than the limit can
act (Grok, 11.8). At the target of -5 A for each VESC
(`docs/battery_settings.md`, section 4), the rise is about 5 V/ms, and
the regen cutoff has a better chance to act (VERIFY on a scope in T-2).

What 90 V does to this design:

- The SMBJ58A at the battery tap breaks down at 64.4 V minimum at 1 mA
  (https://www.digikey.com/en/products/detail/littelfuse-inc/SMBJ58A/688286).
  Its breakdown falls when it is cold, to about 62 V at -10 C (about
  0.1 percent per kelvin; VERIFY). Above that it tries to clamp the
  whole bus against two VESCs at up to 40 A each. It is a 600 W,
  10/1000 us part. It overheats and fails short, and F3 opens. The Pi
  then has a hard cut while the robot moves.
- The brick is rated 75 V continuous and 100 V for 100 ms. A pumped bus
  for a whole stop is outside that rating.
- The LTC4368 alternative for the HSS (60 V operating maximum) and the
  dkplnt 12 V option (60 V maximum) do not fit either.

The tap TVS is a clamp for hot-plug rings of microseconds. It is not a
bus clamp, and the design must keep the bus below its breakdown.

What must be true before the brick goes on the bus. Kevin makes the
changes in the phone apps:

1. Apply the VESC targets of `docs/battery_settings.md` section 4 on
   both VESCs: the regen current, the drive current, the low-voltage
   cutoff, the regen cutoff ramp (with +0.9 V on the right VESC) and 13
   cells on the right VESC. This spec does not repeat the numbers. That
   document is the source.
2. Apply the BMS targets of `docs/battery_settings.md` section 3 in the
   order that it gives: the battery type, the charge over-current
   protection, and the overvoltage protection only after the charger is
   measured. The regen ramp ends below the BMS pack overvoltage trip, so
   normal braking still goes into the cells.
3. Decide `l_max_vin` (open question 22). `docs/battery_settings.md`
   keeps 90 V. This review proposes about 61 V on the left VESC and
   +0.9 V on the right VESC (the reading offset). The value must be
   above the highest regen-cutoff end, below the cold breakdown of the
   tap TVS (about 62 V), and below the rating of the VESC bus capacitors
   (VERIFY on the board, open question 25). The trade-off is below.
4. Read the settings back on both VESCs with `tools/vesc_info.py`.
   Record them in `docs/battery_settings.md`.
5. Run T-2 (9.2) before the brick is fitted, with the HAT (B) still on
   the Pi. Pass: the bus stays under 60 V in each shot.
6. If T-2 fails, fit the bus dump clamp (below) before the brick.
7. Do step 4 again after each VESC Tool session and each VESC firmware
   update. Do not run the motor setup wizard: it overwrites the cutoffs
   (`docs/battery_settings.md`, section 4). A restore of the defaults
   sets the regen cutoff back to 1000/1100 V. The daemon checks the
   settings at each boot (6.1).

The `l_max_vin` trade-off. The fault is a "stop and coast" backstop, not
a bus regulator. It holds the bus near its value only while the motor
back-EMF is below the bus (case 2 above). After an over-voltage fault
the VESC releases the motor for its fault stop time, about 500 ms
(`m_fault_stop_time_ms`; VERIFY the value on WALL-E), so each fault is a
brake-loss event. A lower fault protects the tap TVS and the brick, at
the cost of such an event at a lower bus voltage. At 90 V the brake
loss also happens, and the TVS burns first. A lower fault does not
restore braking that the open charge FET removes
(`docs/battery_settings.md`, section 4). What keeps the bus from the
fault is the regen ramp at -5 A and the BMS overvoltage fix. CAUTION
(drivetrain, not this board): the right VESC reads 0.9 V high, so
without the offset it faults first. That side coasts while the other
still brakes, and the robot yaws. This matters most in FOLLOW_ME and
WAYPOINT_NAV, which keep every protection (`CLAUDE.md`, safety
principle). `docs/battery_settings.md` lists a software guard as open:
refuse to arm, or limit the speed and the deceleration, while the BMS
reports the charge FET open.

Bus dump clamp (open question 16). A FET, a power resistor and a
comparator on the chassis, across the bus. These are the rules:

- Set the trigger above the highest regen-cutoff end
  (`docs/battery_settings.md`, section 4) and below `l_max_vin`
  (VERIFY against the values that Kevin sets).
- Give it its own fuse, so that a shorted clamp FET cannot open F3.
- Do not use a crowbar (an SCR) on the traction bus. A short brakes the
  motors hard through their body diodes and leaves the robot dead.
- Size it from the T-2 traces. The energy for one stop is at most the
  kinetic energy (about 90 J at 70 kg and 1.6 m/s) plus the motor rotors.
  The peak power is the full regen current at the trigger voltage: at
  the -5 A target, 2 x 5 A at about 58 V is about 0.6 kW; at today's
  -40 A it is about 4.6 kW, for milliseconds (VERIFY).

Alternative: tap the brick on the cell side of the BMS (B+ to B-), open
question 23. A BMS of this type usually switches the negative lead
(VERIFY on the JHB-R24TH). With the
charge FET open, the bus rises, and the cells do not. A brick across
the cells then never sees regen overvoltage, and neither does the tap
TVS. A BMS low-voltage cut also kills the ignition (it is on the BMS
output side), so the Pi gets a clean halt in place of a hard cut. The
cost: the BMS cut no longer disconnects the brick. A stuck-on HSS
(risk R-26) then drains the cells below the BMS cut, and that can ruin
them. This review does not recommend the cell-side tap without a second
series switch.

Not a fix: a series diode in the brick feed. A diode blocks reverse
current only, so a bus overvoltage passes through it (11.8).

Other rules:

- WARNING: "Battery Voltage Cutoff Start/End" in VESC Tool is the
  LOW-voltage cutoff. Do not set it near 56 V. The robot then stops
  driving.
- Buy converter voltage margin: a 72 V-class 12 V unit (3.2) and a Pi
  brick rated 100 V for 100 ms (3.4).
- If T-2 shows that `l_max_vin` needs more room, the SMBJ60A (66.7 V
  minimum breakdown, 96.8 V clamp; VERIFY) adds about 2 V. Its clamp is
  still under the brick's 100 V surge rating.

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
off. The Pi turns an output on over UART. Both outputs go off with the
12 V rail (ignition off), on an overvoltage on the rail, when the Pi
halts, and on an MCU reset (5.5).

### 3.4 Pi 5.1 V rail: the brick

The Pi rail comes from an isolated telecom brick on the battery tap,
behind the HSS (3.5). Its isolation is not the main point. Its input
rating is. A 60 V module (Pololu D42V55F5) cannot be made safe on 13S
with a TVS: a TVS that stays off at 54.6 V clamps near 90 V (Vishay:
SMBJ58A clamps at 93.6 V). The brick must take 75 V continuous and
100 V for 100 ms (option A, section 4). The TI LM5013 (100 V) is 3.5 A,
non-synchronous, and too small.

Candidates found on 2026-11-04 (`docs/pi_power_architecture.md`). The
TRACO rows were checked on 2026-11-05 in the current datasheets
(revision 8 October 2026):
TEN 30WIN https://www.tracopower.com/sites/default/files/products/datasheets/ten30win_datasheet.pdf ,
TEN 20WIN https://www.tracopower.com/sites/default/files/products/datasheets/ten20win_datasheet.pdf .
No part is checked on the bench.

| Item | Part | Rating | Off current | Price, stock |
|---|---|---|---|---|
| Default | TRACO TEN 30-4811WIN | 18 V to 75 V input, 100 V for 100 ms maximum; 5.1 V 6 A, 88 percent at full load; no-load input 65 mA typical at 48 V (about 3.1 W); undervoltage lockout 15 V minimum, 17.5 V maximum; output OVP 6.2 V typical; current limit 150 percent typical; capacitive load 14,400 uF maximum; no minimum load; trim plus or minus 10 percent; flyback; -40 C to +75 C, case 105 C maximum | 3 mA typical | 66.80 USD; none at TME |
| Alternative | RECOM RP30-4805SFW/N | 18 V to 75 V (100 V for 100 ms), 5 V 6 A, trims to 5.1 V. The /N suffix is negative logic: on with the remote pin shorted to -Vin, off with it open (series data, https://datasheet.eeworld.com.cn/view/9307659.html ; VERIFY in the RECOM datasheet) | 3 mA maximum | 91 EUR; 11-week lead |
| Smaller, in stock | TRACO TEN 20-4811WIN | 18 V to 75 V, 100 V for 100 ms maximum; 5.0 V (not 5.1 V) 4 A, 88 percent; no-load input 35 mA typical at 48 V (about 1.7 W); undervoltage lockout 14.5 V minimum; output OVP 6.2 V typical; -40 C to +85 C | 2.5 mA typical | 49.05 USD; 25 at TME. Trim it up to 5.1 V (VERIFY the trim resistor). With 4 A, do not set `PSU_MAX_CURRENT=5000` (6.3). |
| Not chosen | Mean Well RSD-60L-5 | 18 V to 72 V, 5 V 12 A, 89 percent | no remote pin | 44.18 USD. The HSS makes a remote pin unnecessary, but the 72 V maximum has no 100 V surge rating listed (VERIFY), and 12 A is far more than needed. |

The 18 V input floor is far below the 37.7 V BMS cut. Motor sag on a low
pack therefore cannot drop the Pi rail. The 12 V converter's floor
(40 V on the Vatrer example) can drop the 12 V rail, but not the Pi.

The no-load loss sets the brick's efficiency at the Pi's load. The
TEN 30 takes about 3.1 W with no load and about 4.2 W of loss at full
load. At the Pi's typical 1.3 A it runs near 68 percent, not 88 percent
(4.1). The TEN 20 takes about 1.7 W with no load. At 1.3 A it runs near
78 percent and makes about 1.4 W less heat. The 4 A limit is its cost
(open question 6).

Rules for the brick:

- The HSS (3.5) is the only ENABLE. Tie the brick's remote pin to its ON
  level. On the TEN parts, leave the pin open ("passive = on" in the
  datasheet). On the RECOM /N part, short it to -Vin. Reason: the
  remote pin is on the input side and refers to -Vin, so a logic-side
  drive needs another opto. A drive that fails as the rail decays could
  let the brick restart from its input capacitors and pulse the Pi rail
  (risk R-12).
- Fit a 2.2 uF, 100 V ceramic (the TRACO EMC part for the 48 V models)
  and a 47 uF, 100 V electrolytic with an ESR of 0.3 to 1 Ohm at the
  brick input. The electrolytic damps the ring of the lead inductance
  against the ceramic. The TRACO surge-test part (KY 220 uF, 100 V) is
  for EN 61000-4-5. If it is used, the HSS slew must change (3.5).
- Fit a 22 kOhm leakage bleed across the brick input, on the brick side
  of the HSS (3.5, risk R-31).
- Trim to 5.10 V at the header under 2 A. Keep the brick output at or
  below 5.3 V, so the crowbar (5.9 V) and the brick OVP (6.2 V) keep
  their margin.
- Bond its output negative to system ground once, at the board. Keep
  its input negative on the battery side (3.1).
- Minimum load: none (datasheet). The fixed 1 kOhm bleed (3.8) is a
  5 mA load in any case.
- Soft start: the output must rise from 10 percent to 90 percent in
  under 50 ms with the Pi connected. The datasheet gives a start-up
  time of 50 ms maximum, not a rise time (VERIFY in T-5; R16).
- Output overvoltage protection: 6.2 V typical. VERIFY its mode (latch
  or hiccup). The crowbar (3.8) is a second layer, set below it.
- Capacitive load: 14,400 uF maximum (TEN 30). The Pi and the board
  have about 1000 uF (VERIFY, risk R-8).
- Heat: about 3.2 W at the typical load (4.2). The datasheet gives
  12 K/W case to ambient in natural convection, so the case runs about
  40 K above the air around it. In a closed enclosure at 45 C that is
  near the 105 C case limit. The TRACO clip-on heat sink (TEN-HS1,
  10 K/W in the 2009 datasheet) helps little. Put a thermal pad between
  the case and a metal enclosure lid (VERIFY in T-3). The TEN 20 makes
  less heat at this load (above).

The Pi rail also feeds the RTK GPS (I2C 0x20) through the header 5 V
pin (`docs/ups_hat_b.md`). It now goes off with the Pi rail, so it no
longer runs while the robot is stored. Measure its draw (VERIFY, 4.1).

Pi firmware settings that this rail enables (section 6.3): with the 6 A
brick, `PSU_MAX_CURRENT=5000` in the EEPROM is correct. The repository
rule "do NOT set `usb_max_current_enable=1`" exists only because the
HAT (B) could not supply 5 A. That rule ends when this module is
installed with a 6 A brick. With the 4 A TEN 20, keep the rule. The Pi's
USB ports carry no device power on this module (3.10), so the 4 A brick
loses nothing that the robot uses.

### 3.5 Battery-side switch (HSS) and ENABLE

ENABLE is the state of the HSS. ENABLE = IGNITION OR HOLD (section 2).
Two N-FETs in parallel pull the P-FET gate on: Q1 for the ignition, Q2
for the hold. Each N-FET has its own photovoltaic opto (PV opto). A PV
opto makes its gate voltage from the light of its own LED. It needs no
supply on the battery side. With both LEDs dark, nothing on the battery
side takes current except leakage (4.3).

| Item | Part | Key ratings |
|---|---|---|
| HSS | P-FET, -150 V or more, 0.5 ohm or less. Example: Vishay IRF9640: -200 V, 0.50 ohm at Vgs -10 V, Vgs plus or minus 20 V, Vgs(th) -2 V to -4 V, IDSS 100 uA maximum at -200 V and 25 C, TO-220AB (https://www.vishay.com/docs/91086/irf9640.pdf ; VERIFY its current availability) | A -100 V part leaves only 6 percent over the SMBJ58A clamp (93.6 V). At 0.43 A the loss is under 0.1 W. The turn-on pulse (about 1.5 A for 2 ms) is inside its safe operating area (VERIFY on the SOA curve). |
| Gate network | 100 kOhm and a 12 V Zener from gate to source; 100 kOhm from the gate to the drains of Q1 and Q2 | Vgs is about -12 V whenever Q1 or Q2 conducts, at any pack voltage. 0.43 mA at 54.6 V while on. No current while off. The Zener current is only 0.14 mA at 37.7 V, so it sits below its nominal voltage; -11 V still turns the IRF9640 fully on. |
| Turn-on slew | Gate-to-drain capacitor Cgd, 15 nF, 250 V X7R or C0G | The gate current at the Miller plateau is about 0.45 mA at 54.6 V. The drain then rises at about 30 V/ms, so the turn-on takes about 2 ms. With the 47 uF input electrolytic (3.4) the inrush is about 1.4 A. With the 220 uF TRACO surge part it would be about 6.6 A: then raise Cgd and Cgs together. The brick's own soft start sets the 5 V ramp, not the HSS. The turn-off takes the same plateau through the 100 kOhm gate resistor: about 16 ms. |
| Hot-plug hold-off | Gate-to-source capacitor Cgs, 1 uF effective at 12 V (for example 2.2 uF, 25 V X7R) | A fast step on the source (the pack or F3 connected) couples through Cgd into the gate. Without Cgs, Vgs follows the step until the Zener clamps it at -12 V, and the HSS turns on with the ignition off (risk R-27). With Cgs at 66 times Cgd or more, a 100 V step gives under 1.5 V of Vgs, below the 2 V minimum threshold. Cost: about 10 ms more delay at turn-on and about 90 ms at turn-off. |
| Q1, Q2 | Small N-FET, 150 V or more, Vgs(th) under 2.5 V, SOT-23 (VERIFY the part) | They see the bus voltage while off, up to the 93.6 V TVS clamp. Each gate has its own 10 MOhm to source as well as the opto's turn-off circuit (Grok, 11.8). Their off leakage flows through the 100 kOhm gate resistor of the HSS. Keep the IDSS of Q1 plus Q2 under 10 uA at +85 C (VERIFY), so that Vgs stays above -1 V. |
| U1, U3 | PV MOSFET driver opto: Vishay VOM1271. Open-circuit voltage 8.1 V typical and short-circuit current 7 uA typical at an LED current of 5 mA; minimums are given only at 10 mA (7.8 V, 6 uA). LED 50 mA maximum, 5 V reverse maximum. Built-in turn-off circuit (https://www.vishay.com/docs/83469/vom1271.pdf) | U1: LED in the ignition string (3.6). U3: LED driven from the logic side by the hold path (3.7). One opto per FET. Do not connect two PV outputs to one gate: the turn-off circuit of the dark one shunts the lit one. At 7 uA a SOT-23 gate charges in well under 1 ms. VERIFY the drive at 5 mA at -10 C and +85 C (T-9). |
| TVS | Littelfuse SMBJ58A, between F3 and the HSS | 58 V standoff, breakdown 64.4 V to 71.2 V at 1 mA, clamp 93.6 V at 6.4 A, 600 W (10/1000 us), leakage 1 uA maximum at 58 V and 25 C (https://www.digikey.com/en/products/detail/littelfuse-inc/SMBJ58A/688286). A clamp for hot-plug rings only. It must never see a sustained bus above about 62 V (3.2a, risk R-28). |
| Input leakage bleed | 22 kOhm, two 1206 parts in series (200 V each), across the brick input on the drain side of the HSS | With the HSS off, its leakage would charge the brick input. Without a bleed it could pass the 15 V undervoltage lockout, start the brick and pulse the Pi rail in storage (risk R-31). The IRF9640 maximum is 500 uA at 125 C; 500 uA in 22 kOhm is 11 V, under the lockout. It takes 2.5 mA (0.14 W) while the HSS is on. It also drains the input capacitors after a cut (1 s time constant). |
| Alternative HSS | ADI LTC4368-class controller with back-to-back N-FETs | Named in option A. Its operating maximum is 60 V, below the bus as found (90 V, 3.2a). Do not use it unless the bus stays under 60 V. VERIFY its shutdown current against R11. |

Resistors on the battery side. Use 1206 or larger parts, rated 200 V or
more, for each resistor that can see the bus. A 0603 part is often rated
only 50 V to 75 V.

No bridge across the main switch. The battery side has no galvanic link
to the ignition lead (U1 isolates it) or to the logic side (U3 and the
brick isolate it). No part of it can feed the switched side, so the
main switch still makes the drivetrain dead (R7). Test T-14 checks this.

Single points of failure. A shorted HSS, Q1 or Q2, or a shorted U1 or U3
output, holds the brick on for ever. The grace timer cannot open it. The
MCU detects it and reports it (3.11, risk R-26), but it cannot open the
path. Open question 21 asks whether to accept this.

### 3.6 Ignition sense

The LED string is U1 (PV), U2A and U2B (phototransistor) in series. It
runs from the ignition lead through F4, a series diode and a resistor
chain.

| Item | Part | Key ratings |
|---|---|---|
| Resistor chain | 2 x 3.3 kOhm, 2512, in series | The string drops about 4.3 V to 5.8 V (three LEDs at 1.2 V to 1.6 V each, and the diode). The current is then 4.8 mA to 5.1 mA at 37.7 V, 7.4 mA to 7.6 mA at 54.6 V (0.38 W for the pair) and 8.2 mA to 8.4 mA at 60 V. At the 90 V bus of 3.2a it is 13 mA and 1.1 W for the pair, which two 1 W parts carry for seconds. Two parts share the voltage. |
| Series diode | 200 V or more (VERIFY the part) | Reverse polarity on the ignition lead. The LEDs take only 5 V reverse (VOM1271 datasheet). |
| U2A, U2B | Phototransistor opto, 2 pieces or one dual. Example: Toshiba TLP293. CTR 50 percent minimum (unranked) at an LED current of 5 mA. Dark current 0.08 uA maximum at 25 C, 2 uA typical and 50 uA maximum at 85 C, both at 48 V (TLP293 family datasheet, copy at https://static.chipdip.ru/lib/877/DOC011877980.pdf ; VERIFY in the current Toshiba datasheet) | U2A: IGN_OFF to the Pi. U2B: grace timer reset (3.7). |

U1's drive at 5 mA. The VOM1271 gives 8.1 V typical at 5 mA, which is
far above the 2.5 V threshold of Q1. Its minimums are given only at
10 mA. T-9 checks the drive at 5 mA, cold and hot. If it fails, use a
100 V-class constant-current source of about 10 mA in place of the
resistor chain.

IGN_OFF. U2A's collector goes to GPIO 27 through 1 kOhm. Its emitter
goes to system ground. A 10 kOhm pull-up on the module goes from
GPIO 27 to PI_3V3 (header pin 1). It takes current only from the Pi's
own 3V3, so the module sources nothing into the Pi: with the Pi off,
the pull-up is at 0 V too. The Pi's libgpiod bias pull-up stays on as a
second layer. With the ignition on, U2A conducts and GPIO 27 reads low.
With the ignition off, GPIO 27 reads high. An open path (F4 blown, the
lead broken, an opto open) also reads high, so every open fault reads
as "ignition off", and the Pi halts (risk R-6).

Why 10 kOhm and not the Pi's pull-up alone. The Pi's internal pull-up is
weak (tens of kOhm, VERIFY on the RP1), and it is off until the pin is
claimed. At 85 C the TLP293 dark current can reach 50 uA. Through a
pull-up of about 50 kOhm that is a drop of about 2.5 V: GPIO 27 then
reads low, "ignition on", with the ignition off. The Pi never halts. The
same heat holds the grace timer in reset (3.7), so the backstop fails
too (risk R-29). Through 10 kOhm, 50 uA is 0.5 V. U2A then sinks
0.33 mA when lit, which the 50 percent CTR at 5 mA covers many times.

GPIO 27 is unclaimed on the robot (checked 2026-11-04, 6.3).

### 3.7 Hold path and grace timer

Hold path. Q5 (small N-FET, Vgs(th) under 1.5 V, BSS138 class, VERIFY)
switches U3's LED from the 5.1 V rail, about 8 mA through 470 ohm
(VERIFY against U3). Q5's gate takes the Pi's 3V3 (header pin 1)
through 100 kOhm, with 1 MOhm to ground. The module sources nothing
into the pin. Q6 (2N7002 class) pulls Q5's gate to ground when
GRACE_EXPIRED is high. It then sinks 33 uA from the Pi's 3V3. That is
the transistor that "pulls ENABLE off even if the Pi's 3V3 is still up"
(option A, section 4).

With `POWER_OFF_ON_HALT=1`, a halt puts the PMIC in STANDBY, and "all
outputs are switched off" (Raspberry Pi documentation, bootloader
properties). 3V3 falls to 0 V. That is the halt witness. The hold path
then opens at once, without the timer. VERIFY on the robot that pin 1
reads under 0.3 V in standby with the RTK GPS connected: its I2C
pull-ups must not lift the Pi's dead 3V3 rail (T-6).

If the EEPROM loses `POWER_OFF_ON_HALT=1` (an EEPROM recovery restores
the defaults), 3V3 stays up after a halt. Each ignition-off then ends in
a grace cut. The cut lands on a Pi that has already halted and synced,
so it does no harm, but the brick runs for the full grace time. The
daemon checks the setting at each boot (6.1).

Grace timer. A counter timer, not a large RC: a phototransistor's dark
current grows with heat, and a megohm RC timer at +70 C can then never
expire (risk R-11).

Datasheet facts (TI CD74HC4060,
https://www.ti.com/lit/ds/symlink/cd74hc4060.pdf): supply 2 V to 6 V;
pin 3 is the last stage (TI calls it Q14; Nexperia calls the same pin
Q13); pin 7 is Q4; MR (pin 12) is active high, and a high level resets
the counter and disables the oscillator; the counter advances on the
falling edge of the clock input (pin 11), which has a Schmitt trigger;
the RC frequency is about 1/(2.2 Rt Ct). Nexperia gives about
1/(2.5 Rt Ct) with Rs about 2 Rt (from a search summary of the
Nexperia 74HC4060 datasheet; its page refused the download, VERIFY).
The vendors differ by about 13 percent. Quiescent current: 8 uA maximum at 25 C, 80 uA maximum at
85 C.

| Item | Part | Key ratings |
|---|---|---|
| Counter | 74HC4060 (CD4060B is the alternative) | 14-stage counter with an RC oscillator. Rt 100 kOhm 1 percent (trimmed), Ct 39 nF C0G 5 percent, Rs 220 kOhm. Before the trim: 117 Hz and 70 s (TI constant) or 103 Hz and 80 s (Nexperia constant). Pin 3 goes high after 8192 periods. Trim Rt on each board to 75 s (T-9). Do not use 47 nF: with the Nexperia constant it gives about 96 s, above the 90 s maximum. |
| Latch | Small signal diode (1N4148 class), anode on pin 3, cathode on pin 11 | When pin 3 goes high, it holds the clock input high, so the oscillator stops. GRACE_EXPIRED then stays high until the reset or a loss of supply. If the latch diode is open, pin 3 stays high for another 8192 periods. The rail is already off by then, so the Pi's 3V3 cannot close the hold path again. |
| Reset | U2B from the timer supply into MR (pin 12), 10 kOhm to ground | With the ignition on, U2B holds the counter at reset. With the ignition off, the pull-down sets reset low. The TLP293 dark current can reach 50 uA at 85 C (3.6). Through 10 kOhm that is 0.5 V, a logic low (VIL is 30 percent of VCC, about 1.4 V). Through the 47 kOhm of the earlier draft it was 2.35 V: the counter could stay in reset, and the timer could never expire (risk R-29). U2B must source 0.34 mA to make a high, which the 50 percent CTR at 5 mA covers. |
| Timer supply | From the 5.1 V rail through a Schottky diode into 220 uF | About 4.8 V while the rail is up. After the rail goes, it holds the stopped counter and Q6 above 2 V for at least 8 s at the 85 C current maximum, and much longer at 25 C. Q6 then stays on until long after the Pi's 3V3 has gone (risk R-12). |
| MCU taps | Pin 3 (GRACE_EXPIRED) and pin 7 (Q4, about 7 Hz while counting) to two 5 V-tolerant MCU pins, each through 100 kOhm | The MCU checks that the clock runs during GRACE (risk R-30, 3.11). The resistors limit the current into a dead MCU while the timer supply still holds. |
| Alternative timer | TI TPL5110 or TPL5111 nano-power timer (VERIFY that its one-shot mode fits the reset-by-ignition rule) | Resistor-set interval. |

Power-up state. The counter powers up with a random count. With the
ignition on, MR is high from the start and clears it. With the ignition
off (a pack connected with the HSS forced on, which Cgs now prevents),
the random count can only end early: the latch stops the count at the
first high on pin 3, so the time is never longer than one full count.

Grace time. 75 s nominal after the trim, 60 s minimum, 90 s maximum over
-10 C to +45 C (VERIFY in T-9). The RC oscillator moves with the supply
and the temperature. The fast end matters most, because a fast timer
cuts a normal halt. The halt is bounded at about 43 s (4.4). The Pi
cannot change the time: it is set by parts, and no register or command
reaches it.

### 3.8 Bleed, output fuse and crowbar

These parts sit on the 5.1 V rail, on the logic side.

| Item | Part | Key ratings |
|---|---|---|
| Output fuse | 7 A, 32 V DC or more, between the brick and the header (VERIFY the part) | Above the brick's 6 A rating. The brick limits at about 150 percent (9 A) and recovers by itself from a short (datasheet; the mode is not stated). It may therefore not open a 7 A fuse. Then the crowbar still holds the rail down. |
| Crowbar | TL431 set near 5.9 V triggers an SCR rated 10 A or more across the rail after the fuse (VERIFY the parts, and 5.9 V against the Pi 5 input rating) | A second layer behind the brick's own output OVP (6.2 V typical, 3.4). 5.9 V sits above the trimmed output (5.3 V maximum) and below the brick OVP. |
| Switched bleed | Logic-level P-FET and 47 ohm, 1 W across the rail. Gate from a nano-power supervisor, for example TI TPS3839 or TPS3840 (VERIFY the threshold variant): on below about 4.6 V, off above about 4.8 V | When the brick stops, the rail sags, the supervisor trips, and 47 ohm takes the rail to about 1 V within 0.2 s with about 1000 uF on the rail (VERIFY the real capacitance). |
| Fixed bleed | 1 kOhm, 0.125 W | 5 mA, 26 mW while running. It takes the last volt down within a few seconds. |

The bleed does not use ENABLE. It acts on every loss of the rail: an
ignition-off halt, a grace cut, a BMS cut, a blown F3, a brick fault.
Each time the Pi sees a clean drop, and the next 5 V application is a
real one (R16).

Sneak feeds. Nothing may hold the rail up from outside. The USB
injectors leave the Pi-side VBUS open (3.10). The MCU sets its UART TX
to high impedance while the Pi's 3V3 is down (3.11). The RTK GPS on the
header 5 V pin is a load, not a source. Its I2C pull-ups must not lift
the Pi's 3V3 (3.7). The IGN_OFF pull-up goes to the Pi's own 3V3, so it
is not a source either (3.6). No other lead goes to the Pi.

### 3.9 Pi interface signals

| Signal | Direction | How |
|---|---|---|
| 5.1 V | Module to Pi, header pins 2 and 4 | From the brick through the output fuse. Ground on pins 6, 9, 14, 20. |
| IGN_OFF | Module to Pi, GPIO 27 | U2A open collector to ground, through 1 kOhm. 10 kOhm pull-up to PI_3V3 on the module, plus the Pi's bias pull-up. High means the ignition is off, or the path is open (3.6). |
| PI_3V3 | Pi to module, header pin 1 | Into Q5's gate through 100 kOhm (3.7), to a 5 V-tolerant MCU pin with no clamp to its supply, and to the 10 kOhm IGN_OFF pull-up. The module sources nothing into the pin. |
| UART0 | GPIO 14 (TX), GPIO 15 (RX) | 3.3 V, 115200 8N1, 1 kOhm series resistor each way. The MCU's TX is high impedance while PI_3V3 is low. |

Removed with the buffer design: SHUTDOWN_REQ, the PWR_BUT pulse on J2,
PI_5V_SENSE as a control signal (the MCU still reads the 5.1 V rail for
telemetry) and the HOLD latch with its 300 s timeout.

### 3.10 5.0 V USB rail

One buck from the 12 V rail makes the USB rail. A USB fault cannot
brown out the Pi: the Pi has its own brick on another lead.

| Item | Part | Key ratings |
|---|---|---|
| USB buck | TI LM61495 (LM61495-Q1) | 3 V to 36 V input, 10 A, synchronous, 400 kHz, VQFN-HR 4.5 x 3.5 mm. https://www.ti.com/product/LM61495-Q1 . Set 5.00 V. 5 A. |
| Alternative | TI LMQ61460 | 36 V, 6 A. |
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
The device cannot backfeed the Pi. The Pi's port sees no load. This
also ends the hub back-power of the 2026-11-04 main-switch pull: with
its own supply off, the hub back-powered from the Pi's ports, and the
HAT carried 1.6 A at 6.34 V (`docs/ups_hat_b.md`). The SuperSpeed
pairs need 90 ohm differential routing, under 50 mm, on the four-layer
board. VERIFY: enumerate the OAK at 5 Gbps through the injector (bench
test T-10).

The USB rail is off whenever the 12 V rail is off: with the ignition
off, and on a 12 V rail loss (5.3).

Alternative for the product spin: a Microchip USB5744 four-port USB 3
hub on the module, which removes the external hub.

### 3.11 Microcontroller

| Item | Part | Why |
|---|---|---|
| MCU | ST STM32G071CBT6 | 48-pin LQFP, 7 x 7 mm, 0.5 mm pitch (the earlier draft said 64 pins; the 64-pin part is the STM32G071RBT6). 128 kB flash, 4 USARTs, 12-bit ADC, Stop mode in the microamp range, independent watchdog, 1.7 V to 3.6 V. Hand-solderable. https://www.digikey.com/en/products/detail/stmicroelectronics/STM32G071CBT6/9749147 |
| Alternative | Raspberry Pi RP2040 | Familiar to Pi users. Needs external flash and has higher sleep current. Firmware in MicroPython or C. |
| 3.3 V housekeeping LDO | TI TPS7B81-Q1, from a diode OR of the 12 V rail and the 5.1 V Pi rail | 40 V input, 150 mA, 2.7 uA quiescent. VERIFY part number. Alternative: TI TPS7B69. The 5.1 V feed keeps the MCU up when the 12 V rail is lost, so it can report that. The diodes stop any feed back into either rail. |
| Status | 1 RGB LED, 1 buzzer | LED patterns in 5.5. The module has no button: there is no wake path to start. |

Inputs: the 12 V, 5.1 V and 5.0 V rails (dividers into the ADC), the
OUT1 and OUT2 current-sense pins, the four port FAULT pins, a board NTC,
PI_3V3 (3.9), GRACE_EXPIRED and the timer clock Q4 (3.7). Use
5 V-tolerant pins for the two timer lines. Outputs: OUT1 and OUT2
enables, the four port enables, the LED and the buzzer.

Two checks of the Pi power path. The MCU observes them. It does not act
on the path:

- TIMER_FAULT. During GRACE (12 V rail down, 5.1 V rail up, pin 3 low),
  Q4 must change state at least every 0.5 s. If it does not for 1 s,
  the oscillator is dead, and a hung Pi would never be cut (risk R-30).
  The MCU stores the fault in flash and sends `E,TIMER_FAULT` at the
  next boot.
- STUCK_ON. The 5.1 V rail is up, the 12 V rail is down and
  GRACE_EXPIRED is high, for 10 s. The timer has done its work, so the
  HSS should be open and the rail should be gone within about 1 s.
  Something in the switch is shorted (risk R-26). The MCU shows fast red on the LED,
  stores the fault and sends `E,STUCK_ON` at the next boot.

The MCU has no output into the Pi power path. It cannot hold the Pi on,
and it cannot cut it. Its I2C bus carries nothing in spin 1, and it
never touches the Pi. The UART to the Pi is UART0 on GPIO 14 and 15
(3.9).

Firmware watchdog: the independent watchdog resets the MCU after 2 s
without a kick. All pin defaults after a reset are the safe states in
section 5.4.

### 3.12 Connectors and fusing

| Connector | Type | Rating |
|---|---|---|
| BATT TAP | 2-pin pluggable terminal, 5.08 mm | 2 A, 100 V or more (VERIFY the part). Keyed differently from IGNITION (3.1). |
| IGNITION | 2-pin pluggable terminal, other key or pitch | 0.25 A, 100 V or more (VERIFY the part). |
| 12 V IN | 2-pin screw terminal, 7.62 mm pitch | 20 A or more (VERIFY the part). Not an XT connector, so that the pack harness cannot plug into it. |
| OUT1, OUT2 | 5.08 mm screw terminal, 2 pin | 15 A. |
| Pi power | 6-wire harness to header pins 2, 4 (5 V) and 6, 9, 14, 20 (GND) | 3 A per pin, 6 A on 2 pins. Alternative: pogo pins to the Pi 5 bottom pads as the HAT (B) does. |
| Pi signals | 5-wire harness: 3V3 sense (pin 1), GPIO 14, 15, 27, GND | Under 150 mm. |
| USB injectors | 2 x USB 3 Type-A receptacle + 2 x USB 3 Type-A plug cable, 150 mm | 90 ohm pairs. |
| USB power outputs | 2 x USB Type-A receptacle, power only | D+ D- open. |
| Fan (optional) | 2 pin JST-PH, 5 V | For the enclosure. |

Fusing: F1 at the switched-side end of the 12 V converter's input lead
(off the board, 3.2). F3 at the battery tap and F4 on the ignition
lead, each at its source end (off the board, 3.1). F2 on the 12 V IN
(3.1). The output fuse on the Pi rail (3.8). The 12 V outputs are
protected by the high-side switches. The USB rail is protected by the
port switches.

Wire gauges. These are starting numbers for copper wire with 105 C
insulation rated 300 V, in short runs (under 1 m) on the chassis. Each
wire must carry its fuse rating, or the current limit of its source
where no fuse is upstream. VERIFY each against the chosen fuse and the
real length.

| Lead | Protected by | Starting gauge |
|---|---|---|
| Battery tap | F3, 2 A | 22 AWG |
| Ignition lead | F4, 0.25 A | 24 AWG |
| 12 V converter input | F1, about 15 A (3.2) | 14 AWG |
| 12 V lead, converter to 12 V IN | The converter's current limit (no fuse upstream, 3.2) | 12 AWG for a 25 A unit; VERIFY the limit |
| OUT1, OUT2 | Switch limit, 12 A | 16 AWG |
| Pi power harness, each wire | Output fuse, 7 A, on two 5 V wires | 22 AWG, under 150 mm. At 2.5 A the drop is about 0.1 V to the header; the trim covers it (3.4). |

## 4. Power budget, efficiency, thermal

### 4.1 Loads

Pi rail, from the brick:

| Load | Typical | Peak |
|---|---|---|
| Pi 5 + NVMe, control stack | 6 W | 12 W (boot, 2.5 A) |
| RTK GPS on the header 5 V pin | 0.5 W (VERIFY: measure) | 0.5 W |
| Hold LED, timer, supervisor, fixed bleed, IGN_OFF pull-up | 0.1 W | 0.1 W |
| MCU, when the 12 V rail is down | 0 | 0.2 W |
| Pi rail total | 6.6 W (1.3 A) | 12.8 W (2.5 A) |
| TEN 30 brick input (3.1 W no-load loss, plus about 1 W more at full load, rising with the square of the current) | 9.8 W: 0.26 A at 37.7 V, 0.18 A at 54.6 V | 16.1 W: 0.43 A at 37.7 V |
| TEN 20 brick input (1.7 W no-load loss, plus about 1 W more at full load) | 8.4 W: 0.22 A at 37.7 V | 14.9 W: 0.40 A at 37.7 V |
| Battery side: gate network and input bleed (3.5) | 0.16 W | 0.19 W (60 V) |

The brick input rows are estimates from the datasheet no-load current
(65 mA and 35 mA typical at 48 V) and the full-load efficiency (88
percent). The earlier draft used 88 percent at every load, which gave
7.5 W. VERIFY at 0 A, 1.3 A and 2.5 A in T-19.

The 6 A brick runs at 42 percent of its current rating at peak. The 4 A
TEN 20 would run at 63 percent.

With the Pi halted and the ignition on (HALTED_IGN_ON, 5.1), the tap
still feeds the brick's no-load loss (3.1 W), the RTK GPS (0.5 W) and
the housekeeping: about 3.8 W from the pack. The VESCs and the 12 V
side add to that. The 7 Ah pack holds about 340 Wh (13 cells at 3.7 V
nominal). From half charge, a robot left in that state runs the pack
down in about two days at most (risk R-24).

12 V rail, from the off-board converter:

| Load | Typical | Peak |
|---|---|---|
| OAK-D Lite | 4 W | 6.5 W |
| Hub, RC receiver, Arduino | 2 W | 3 W |
| Spare USB port | 0 | 7.5 W |
| USB side total | 6 W | 17 W |
| At 12 V after the 92 percent USB buck | 6.5 W | 18.5 W |
| OUT1 + OUT2 (future lidar, lights) | 0 today | 72 W (6 A) |
| Module housekeeping | 0.5 W | 0.5 W |
| 12 V rail total | 7 W (0.6 A) | 91 W (7.6 A) |
| Converter input at 90 percent, 37.7 V | 8 W (0.21 A) | 101 W (2.7 A) |

The ignition string takes 0.2 W to 0.42 W from the switched side.

The 10 A tap peak (120 W on the outputs alone) plus the USB side is
about 140 W. The 20 A class converter carries it. The limit is heat,
not the converter rating. Kevin chose "peaks are fine" (2026-11-04), so
the outputs may draw 10 A combined for 60 s and 8 A continuous.

### 4.2 Heat

The 12 V converter is off the board. Its heat goes into the chassis
through its case. The Pi brick is on the board and is the largest heat
source there.

Board:

| Source | Typical | Worst |
|---|---|---|
| Pi brick (TEN 30, 4.1) | 3.2 W | 3.3 W (12.8 W out); 4.2 W at its 30 W rating |
| USB buck at 92 percent | 0.5 W | 1.5 W |
| OUT1 + OUT2, 8 mOhm each | 0 | 0.4 W |
| 12 V IN FETs and fuse, about 14 mOhm | 0.02 W | 1.4 W |
| Ignition resistor chain | 0.3 W | 0.5 W (60 V) |
| HSS, gate network, input bleed | 0.2 W | 0.3 W |
| Total, board | 4.2 W | 7.4 W |

Typical WALL-E operation is near the typical column. The worst column
is the future-loads case. The earlier draft gave 1.8 W and 5.7 W,
because it used 88 percent for the brick at every load. With the TEN 20
the brick line is about 1.8 W, and the board total is about 2.8 W
typical. VERIFY the brick's derating at 45 C and in the enclosure (T-3).
The TEN 30 derates by its case temperature (105 C maximum).

Converter, on the chassis. Plan on 90 percent efficiency.

| Source | Typical (7 W out) | At 150 W out |
|---|---|---|
| Converter at 90 percent | 1 W | 17 W |

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
the board thermistor and reports it. The USB buck and the brick
dissipate about 3.7 W to 4.8 W on the PCB. A 4-layer board with 2 oz
outer copper and thermal vias is enough for the USB buck. The brick's
case needs its own path to the enclosure (3.4; VERIFY in T-3).

### 4.3 Storage drain

With the main switch off:

| Path | Current |
|---|---|
| HSS P-FET leakage at 54.6 V | Expected under 1 uA at 25 C. The IRF9640 datasheet maximum is 100 uA at -200 V and 25 C, and 500 uA at -160 V and 125 C. |
| Q1 and Q2 leakage | Expected under 1 uA each at 25 C (VERIFY on the chosen part) |
| SMBJ58A at 54.6 V | Under 1 uA (datasheet: 1 uA maximum at 58 V and 25 C); more when hot |
| Input leakage bleed (22 kOhm) | Carries only the HSS leakage: nothing extra |
| Gate network, PV opto outputs | 0: the gate sits at the source |
| Brick off current (3 mA) | 0: the HSS disconnects the brick |
| Ignition string, 12 V converter, the 12 V rail and all on it | 0: switched side |
| Total from this design | Expected about 4 uA at 25 C. The datasheet maximums do not guarantee R11 (50 uA). T-16 measures it at 25 C and at 45 C. |

The BMS sets the storage time, not the module. The pack is 13S NMC,
7 Ah (`docs/battery_settings.md`, section 1). The estimate below uses
these assumptions: BMS current asleep 0.5 mA (the Daly figure of the
architecture document; VERIFY on the JHB-R24TH), about 0.36 Ah a month;
cell self-discharge about 2 percent a month, 0.14 Ah; and a start at
50 percent, 3.5 Ah. The BMS sleeps only after 3600 s idle (Daly figure,
VERIFY).

| Case | Drain a month | Time from 50 percent |
|---|---|---|
| Brick disconnected by the HSS (this design) | about 0.5 Ah | about 7 months |
| Brick switched off by its remote pin only (3 mA) | about 2.7 Ah | about 6 weeks |
| A stuck-on HSS with the Pi halted (risk R-26, about 3.7 W) | | about 2 days |
| A hung Pi with no grace timer (about 10 W from the tap) | | about 17 hours |

Note on `docs/pi_power_architecture.md`. Its storage estimate uses a
20 Ah pack, which is wrong: the pack is 7 Ah. Its "about 8 months with
the converter idling" also does not follow from its own figure of 2 Ah
a month for a brick's 3 mA. Use the table above. That document is not
changed by this review.

### 4.4 Grace window budget

| Step | Time |
|---|---|
| Daemon debounce | 3 s (settable, 5 s maximum) |
| SIGKILL to `wall-e.service` | under 1 s |
| Halt to "Journal stopped", measured with the HAT (B) watcher (`docs/ups_hat_b.md`) | 4.4 s from the loss on the 2026-11-04 unplug (the kill included); 5 s on the 2026-11-04 main-switch pull; 8 s on the bench (2026-11-03) |
| PMIC standby, 3V3 down, the HSS open | about 1 s (VERIFY in T-6), plus about 0.1 s of Cgs delay (3.5) |
| Typical total | about 10 s to 13 s |
| Bound: the daemon forces the poweroff (6.1) | `--force` at 25 s and `--force --force` at 35 s after the halt starts. The second one powers off at once after a `sync`. |
| Worst bounded total | about 43 s: 3 s + 35 s + about 5 s for the last `sync`, the PMIC and the HSS (VERIFY in T-8) |
| Grace timer | 60 s minimum, 75 s nominal, 90 s maximum |

Why the daemon must force the poweroff. `wall-e.service` sets
`TimeoutStopSec=90` itself (`bin/walle-fast-stop-system.conf` says so),
so the 30 s system cap does not bound it. A wall-e process in
uninterruptible sleep (state D, for example in a USB call to the
camera) survives SIGKILL. The stop job can then wait up to 90 s, and the
grace cut lands first. `bin/walle_ups.py` already assumes this: it arms
a 300 s alarm while wall-e still stops. The earlier "worst bounded total
of about 45 s" used the 30 s cap and was not a bound.

A process in state D can still block the unmount. The forced poweroff
then remounts the root read-only and powers off (VERIFY), and the
filesystem check runs at the next boot (risk R-7).

Energy for the grace window comes from the main pack: 90 s at 16 W is
0.4 Wh, about 0.1 percent of the 7 Ah pack (about 340 Wh). It is not a
design limit.

## 5. State machine

### 5.1 States of the Pi power path

No firmware is in the Pi power path. These are the states of the
hardware in 3.5 to 3.8.

| State | Ignition | Grace timer | Pi 3V3 | HSS and 5.1 V | 12 V rail, USB, OUT |
|---|---|---|---|---|---|
| STORED | off | unpowered | off | off, bled to 0 V | off |
| RUNNING | on | held at reset | on | on | on (OUT as the Pi sets) |
| GRACE | off | counting | on | on, through the hold path | off |
| HALTED_IGN_ON | on | held at reset | off (PMIC standby) | on; the Pi at about 3 mA, the brick at its no-load loss (about 3 W, 4.1) | on (OUT off, USB off after 10 s, 5.5) |
| GRACE_CUT | off | expired | on, then falls | forced off, then bled | off |

The table assumes that no part is shorted. A shorted switch part keeps
the rail up after the grace cut. The MCU reports that as STUCK_ON
(3.11, risk R-26).

### 5.2 Transitions

| From | Event | To |
|---|---|---|
| STORED | Ignition on | RUNNING. Q1 closes the HSS after about 10 ms (Cgs, 3.5), the brick soft-starts, the Pi boots from a cold power-on. |
| STORED | The pack is connected or F3 is fitted, ignition off | STORED. Cgs holds the HSS off against the step (3.5, T-21). |
| RUNNING | Ignition off | GRACE. Q1 opens, the hold path keeps the HSS on, the timer counts, the daemon debounces. |
| GRACE | Ignition on inside the debounce | RUNNING. The timer resets. No halt. |
| GRACE | The Pi halts, 3V3 falls, ignition still off | STORED. The hold path opens, the brick stops, the bleed takes the rail to 0 V. |
| GRACE | Ignition on after the daemon started the halt | The halt completes. 3V3 falls with the ignition on: HALTED_IGN_ON. |
| GRACE | Timer expires, 3V3 still up | GRACE_CUT. |
| GRACE_CUT | Rail below 4.6 V | STORED, with the bleed on. |
| GRACE_CUT | Ignition on before the rail reaches 0 V | RUNNING. The timer resets. The Pi rides through the dip or restarts. A hung Pi stays hung until the next ignition cycle. |
| GRACE | The Pi reboots (`reboot`, a kernel panic) | If 3V3 dips, the hold path opens: STORED. If 3V3 stays up, the timer counts on, and the daemon halts after the boot or the grace cut lands. VERIFY which on the Pi 5. |
| RUNNING | The Pi halts with the ignition on (ch5 e-stop, operator `poweroff`) | HALTED_IGN_ON. |
| HALTED_IGN_ON | Ignition off | STORED. The hold path is already open (3V3 is down), so the HSS opens at once. |
| Any powered state | BMS cut, F3 open, brick fault, loose connector | STORED by a hard cut (5.3). |

### 5.3 Flapping, boot, a Pi that never halts, rail lost, hard cuts

Flapping ignition. The daemon ignores an ignition loss shorter than its
debounce. Once it starts the halt, the halt completes. If the ignition
is back by then, the Pi stays halted with 5 V on (HALTED_IGN_ON) until
the next ignition cycle. The MCU shows this on the LED (5.5). This is
the cost of the rule that power returns only with the ignition: the
module cannot apply 5 V again by itself.

Boot window. If the ignition goes off before the Pi's 3V3 is up, the
hold path never closes, and the rail goes off at once. If it goes off
later in the boot, the daemon reads IGN_OFF high when it starts, and it
halts after the debounce. The daemon starts early (6.1). If the boot
plus the halt is longer than the grace time, the grace cut lands. This
is accepted (risk R-7).

A Pi that never halts. The grace timer forces the HSS off 60 s to 90 s
after the ignition went off. The NVMe root may be dirty (risk R-7). The
Pi cannot extend the timer.

Rail lost with the ignition on. A failed 12 V converter, a blown F2, an
open LTC4368 or motor sag below the converter's input floor (40 V on
the Vatrer example) stops the 12 V rail. The Pi keeps its power: it is
on the battery tap. The USB devices and the outputs go off. The RC
reader (the Arduino on P3) then goes silent, and the controller disarms
on the RC dropout (`cmd.is_armed` goes False on a 1 s RC dropout,
`CLAUDE.md`). The MCU runs on the Pi rail through its diode OR and
reports `RAIL_LOST`. The Pi does not halt on it.

Hard cuts. A BMS cut, a blown F3 (for example after the tap TVS fails
on a bus overvoltage, 3.2a), a brick fault or a loose connector removes
the Pi rail at once. There is no buffer to carry a halt. The
bleed still drops the rail cleanly, so the next power-up boots. Keep the
filesystem tolerant (open question 9). A robot left with the ignition on
runs the pack down to the BMS cut. `config.py` cuts the motors at 39 V
but does not halt the Pi, because it assumed a UPS. Open question 20
asks for a low-pack halt.

ch5 e-stop. `controller.py` schedules `sudo shutdown -h now` 5 s after
the e-stop. Nothing on the module changes for it. With the ignition on,
the Pi sits in PMIC standby at about 3 mA and stays down until a person
cycles the main switch. The brick's no-load loss (about 3 W) continues
until then. That keeps the rule that the e-stop leaves the
Pi down until a person acts (`CLAUDE.md`, "ch5 e-stop powers the Pi
off").

### 5.4 Safe states with a dead MCU

| Line | PCB default | Effect |
|---|---|---|
| HSS, hold path, grace timer, bleed | Hardware only | Unchanged. The Pi gets power, halts and boots as with a live MCU. |
| OUT1, OUT2 | Pull-down, off | Loads off. |
| USB port enables | Pull-down, off (open question 10) | USB off until the MCU runs. |
| UART TX | High impedance (reset state) | No feed into the Pi. |
| LED, buzzer | Off | |

The USB default "off" means that a dead MCU with a running Pi has no
camera. The Pi reports this on the dashboard (`module_alive` false in
6.2). Open question 10 asks whether USB should default on instead.

### 5.5 MCU rules

- OUT1 and OUT2 are off at reset. The Pi turns them on over UART. The
  MCU turns both off on a rail overvoltage (15.0 V; the comparator also
  does this in hardware), on `RAIL_LOST`, and when PI_3V3 falls.
- USB ports follow open question 10 at reset. The Pi can cycle a port
  (`USB,<port>,OFF` then `ON`) for the camera restart case. The MCU
  reports each FAULT pin. In PI_HALTED the MCU switches the ports off
  after 10 s: the devices have no host, and they only drain the pack.
- The MCU runs the TIMER_FAULT and STUCK_ON checks (3.11). It stores
  each fault in flash and reports it at the next boot. It never acts on
  the Pi power path.
- LED: green, RUNNING. Slow amber blink: the Pi is halted with the 12 V
  rail up (HALTED_IGN_ON); cycle the main switch. Fast amber blink: the
  12 V rail is down with PI_3V3 up (GRACE, or a rail fault). Red: a rail
  overvoltage. Fast red blink: STUCK_ON. Dark: the module has no power.
- The buzzer sounds on a rail overvoltage only.

## 6. Pi-side software interface

### 6.1 What replaces `bin/walle_ups.py`

The watcher becomes the ignition daemon (option A). A new daemon
`bin/walle_power.py` (`walle-power.service`, root) owns GPIO 27 and
`/dev/ttyAMA0`. It:

- reads IGN_OFF on GPIO 27 through libgpiod with the bias pull-up (the
  module also pulls it up to PI_3V3, 3.6), and debounces it for 3 s
  (settable, 5 s maximum);
- on a debounced ignition-off, runs the halt. This is the halt; the
  grace timer is the backstop. Each step must fit inside the 60 s
  minimum grace time (4.4):
  1. writes an `ignition_off` event line;
  2. sends SIGKILL to `wall-e.service` (`systemctl kill
     --signal=SIGKILL`);
  3. writes the runtime drop-in
     `/run/systemd/system/wall-e.service.d/50-halt.conf` with
     `TimeoutStopSec=10s`, then runs `systemctl daemon-reload`. A
     process in state D survives SIGKILL, and the unit's own 90 s stop
     timeout would then outlast the grace time. The drop-in is gone at
     the next boot, so normal restarts keep the 90 s;
  4. runs `sync`, then `systemctl poweroff`;
  5. if the Pi is still up 25 s after step 2, runs `sync` and
     `systemctl poweroff --force` (no unit stops; it still unmounts);
  6. if the Pi is still up 35 s after step 2, runs `sync` and
     `systemctl poweroff --force --force` (immediate, no unmount).
  VERIFY that the escalation survives the shutdown: run it from a unit
  with `DefaultDependencies=no`, so that the shutdown does not stop it
  first. `bin/walle_ups.py` has the same pattern, with 30 s and 60 s;
- if IGN_OFF reads high when the daemon starts, starts the same
  sequence after the debounce (the ignition went off during the boot);
- starts early in the boot, before `wall-e.service`;
- reads the module status at 2 Hz and writes `/tmp/ups_status.json`
  with the schema `walle-power/1` (fields in 6.2), so the `/debug` page
  and the dashboard keep working with small changes;
- forwards output commands from the web UI (`POST /api/power/out`);
- sends a 1 Hz heartbeat;
- logs `RAIL_LOST` from the module. It does not halt on it (5.3);
- at each boot, logs a WARNING and shows it on the dashboard when: the
  module reports a stored `TIMER_FAULT` or `STUCK_ON`;
  `rpi-eeprom-config` does not show `POWER_OFF_ON_HALT=1` (3.7); or a
  read-only CAN read of both VESCs (the method of `tools/vesc_info.py`)
  shows `l_max_vin`, the regen cutoff or the current limits different
  from the values recorded in `docs/battery_settings.md`. VERIFY that the VESC read is safe beside
  `wall-e.service`.

The UART is not in the halt path. A dead MCU or a dead UART does not
stop the halt.

Alternative: the `gpio-shutdown` overlay with a 3000 ms debounce puts no
daemon in the halt path (VERIFY that its `debounce` parameter takes
3000 ms). The daemon is preferred: it stops `wall-e` first, logs the
reason, and forces the poweroff in time. The overlay has no forced
poweroff, so a `wall-e` process in state D can outlast the grace time
(4.4). Do not use both on GPIO 27.

Deleted when the module goes in: `walle-ups.service`,
`walle-ups-wakecheck.service`, the shutdown hook
`bin/walle-ups-shutdown-alarm`, the RTC alarm and `chain.json`. There is
no RTC polling. Keep the stop-timeout drop-ins
(`bin/walle-fast-stop-system.conf`, `bin/walle-fast-stop-user.conf`).
Today `bin/install_walle_ups.sh` installs them, so the new installer
must take them over. `i2c-bus-guard.service` loses its UPS witness (the
UPS status age); open question 12.

### 6.2 Protocol

ASCII lines, 115200 8N1, `\n` terminated, CRC-8 as the last field.

Module to Pi, every 500 ms and on change:

```
S,<state>,<v12>,<v5pi>,<v5usb>,<pi3v3>,<grace_exp>,
  <t_board>,<out1>,<i_out1>,<out2>,<i_out2>,<usb_mask>,
  <usb_fault>,<flags>,<uptime>,<crc>
```

`<state>` is the MCU's view: `RUN`, `PI_HALTED` (PI_3V3 down, 12 V up),
`RAIL_LOST` (12 V down, PI_3V3 up) or `OV`.

Events: `E,RAIL_LOST`, `E,RAIL_BACK`, `E,OV,<v12>`,
`E,USB_FAULT,<port>`, `E,FAULT,<code>`, `E,TIMER_FAULT`, `E,STUCK_ON`.
The last two are stored in flash and sent again at each boot until the
Pi clears them (`CLR,faults`).

Pi to module:

| Command | Effect |
|---|---|
| `HB` | heartbeat |
| `OUT,1,ON` / `OUT,2,OFF` | switch an output |
| `USB,<port>,ON/OFF` | switch a port (for the camera restart case) |
| `GET,cfg` | dump configuration |
| `ID` | firmware version, board revision |
| `CLR,faults` | clear the stored TIMER_FAULT and STUCK_ON flags |

Removed with the buffer design: `ARMED`, `DISARMED`, `HALT_NOW`,
`SET,timeout`, `SET,autowake`, `SET,topup_weeks`, `TOPUP`, and the
events `SHUTDOWN_PENDING`, `INPUT_LOST`, `INPUT_BACK`, `TOPUP_START`,
`TOPUP_END`.

The module ignores a command with a bad CRC. It never trusts the Pi
for a safety decision: the Pi cannot extend the grace timer (it is
hardware), cannot keep an output on through an overvoltage, and has no
command that reaches the Pi power path.

### 6.3 Pi settings that change

EEPROM (`rpi-eeprom-config -e`):

```
POWER_OFF_ON_HALT=1     # already set 2026-11-03; load-bearing (below)
WAKE_ON_GPIO=0          # already set 2026-11-03
PSU_MAX_CURRENT=5000    # new: only with a 6 A brick (3.4)
```

`POWER_OFF_ON_HALT=1` is load-bearing. It drops 3V3 at the halt, so the
hold path opens at once (3.7). Without it, a halt draws 1 to 1.4 W, 3V3
stays up, and each ignition-off ends in a grace cut. That cut lands on a
Pi that has already halted, so it is harmless, but the daemon warns
about it (6.1).

`PSU_MAX_CURRENT` "instructs the firmware to skip USB power-delivery
negotiation and assume that it is connected to a power supply with the
given current rating" (Raspberry Pi documentation). With 5000 the
firmware sets the USB limit to 1.6 A on its own, so
`usb_max_current_enable` is not needed. Condition: the Pi rail must be
rated 5 A continuous at the header, which the 6 A brick gives (3.4).

`/boot/firmware/config.txt`:

```
dtoverlay=uart0-pi5                 # /dev/ttyAMA0 on GPIO 14, 15
# GPIO 27 is IGN_OFF, read by walle-power.service (no overlay)
# do NOT add gpio-poweroff: it interferes with the PMIC standby path
```

Checked on the Pi on 2026-11-04: the CAN board is an MCP2515 on SPI0
(`dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25`), so SPI0
and GPIO 25 are in use. GPIO 14, 15, 22 and 27 are unclaimed. UART0 is
free: `serial0` points to the debug UART `ttyAMA10`.

systemd: `wall-e.service` keeps `TimeoutStopSec=90` for normal stops
(the camera teardown on an auto-deploy restart). The daemon shortens it
at the halt only, with a runtime drop-in, and then forces the poweroff
(6.1). `DefaultTimeoutStopSec=30s` (system) and 10 s (user) are set
today (`bin/walle-fast-stop-system.conf`,
`bin/walle-fast-stop-user.conf`). Keep them. They do not bound a unit
that sets its own `TimeoutStopSec`, so the grace budget does not rely on
them (4.4).

The ch5 e-stop path in `controller.py` does not change. Recovery after
an e-stop is a main-switch cycle (5.3).

## 7. Rough bill of materials (research only)

Single-unit prices, estimated from 2026 distributor listings. Not
quotes. The brick prices come from `docs/pi_power_architecture.md`
(2026-11-04). The Murata and TRACO TEP prices in 3.2 were checked on
DigiKey on 2026-11-04. The price of the off-the-shelf 12 V converter
(about 30 USD) is seller data (VERIFY). The other lines are estimates.

| Block | Parts | Estimate USD |
|---|---|---|
| Pi brick | TRACO TEN 30-4811WIN | 67 (66.80; none at TME on 2026-11-04) |
| Battery side | F3 and holder, SMBJ58A, HSS P-FET, gate parts (Zener, Cgd, Cgs, resistors), Q1, Q2, brick input capacitors and bleed | 6 |
| Ignition sense | F4 and holder, resistor chain, series diode, U1, U2 | 6 |
| Hold path and grace timer | U3, Q5, Q6, 74HC4060, timer parts | 5 |
| Pi rail | Output fuse, crowbar, bleed, supervisor | 4 |
| 12 V converter, off the board | 72 V-class golf-cart unit | 30 (VERIFY) |
| 12 V IN | F2, SMBJ15A, LTC4368, 2 FETs | 10 |
| OUT1/2 | 2 x TPS1HB08-Q1, terminals | 10 |
| USB rail | LM61495, inductor, caps | 8 |
| USB ports | 4 x TPS25221, 2 USB 3 receptacles, 2 cables, 2 USB-A | 22 |
| MCU | MCU, LDO, OR diodes, LED, buzzer | 6 |
| Connectors | Terminals, Micro-Fit, harnesses, JST | 14 |
| Passives | Passives, TVS on rails | 10 |
| PCB | 4-layer, 5 pieces, 100 x 120 mm, per board | 15 |
| Total, first board | | about 215 |
| With the RECOM RP30-4805SFW/N | 91 EUR instead of 66.80 USD (VERIFY the exchange rate) | about 40 more |
| With the TEN 20-4811WIN | 49.05 USD | about 195 |
| Product spin: the discrete 12 V buck and its input protection back on the board | | about 230 |

The 2026-11-05 review added small passives: Cgs, the brick input
electrolytic and ceramic, the 22 kOhm input bleed, two gate pull-downs
and two pull-up or pull-down resistors. They are not priced. The review
did not change any estimate (VERIFY when the parts are chosen). The
totals do not include the bus dump clamp (3.2a, open question 16) or a
heat path for the brick (3.4). Both are open.

Removed from the earlier board: the 2S cells and holders, the charger,
the protector and balancers, the INA228, the power path, the Pi buck,
the Pi eFuse and the pack-sense lead (about 55 USD). The Pi brick costs
more than all of them. The first board is therefore about 30 USD over
the earlier 185 USD. A product at 100 pieces with the discrete 12 V buck
lands near 150 to 170 USD in parts; the Pi brick is the largest single
part (VERIFY volume prices). A product spin could replace the brick with
a 100 V synchronous buck (LM5146-Q1 class) for the Pi rail (VERIFY).

## 8. Build or buy

Products checked on 2026-11-04 (vendor pages or retailer listings):

| Product | Input | Battery | Pi 5 | Gap |
|---|---|---|---|---|
| UUGear Witty Pi 5 HAT+ | 6 to 30 V DC or 5 V USB-C, 5 A out | none | yes | No battery, 30 V maximum, no 12 V-class tap. Has voltage-triggered safe power-off and RP2350. Closest "wide input + safe shutdown" board. |
| Joy-IT StromPi 3 | 6 to 61 V, 3 A out | 1000 mAh LiFePO4 option | not listed | 61 V input is right. 3 A cannot run a Pi 5. Pi 5 not in the compatibility list. |
| Geekworm X1202 / DFRobot FIT0992 | 6 to 18 V | 4 or 2 x 18650 | yes | 18 V maximum. 5 A. Cuts power after halt. No wide input. |
| Waveshare UPS HAT (E) | USB-C PD 5 to 20 V | 21700 | yes | PD input only. |
| Waveshare UPS HAT (B), current | 8.4 V jack | 2 x 18650 | yes | Cannot switch its output off or see its input; the buffer drains in storage (`docs/ups_hat_b.md`, `docs/pi_power_architecture.md` section 1). |
| SunFounder PiPower 5 | USB-C PD 5 to 15 V | internal | yes | PD only. |
| Geekworm UPS_Scap | 9 to 24 V | supercapacitor | yes | Seconds of hold-up, 24 V maximum. |
| Sequent Super Watchdog | 5 V | 1 or 2 x 18650 | partial (3 A) | 5 V input. |
| PiJuice HAT | 5 V | LiPo | unverified | 5 V input. |
| Sfera Labs Strato Pi UPS | 9 to 28 V | 12 V lead-acid | no (Pi 3, 4) | 28 V maximum, industrial case, no Pi 5. |
| mjbots, VBCores power boards | up to 44 V | none | n/a | Robot power distribution, no Pi UPS logic, no halt coordination. |

How close the best one comes. The StromPi 3 takes the pack voltage but
cannot run a Pi 5 and is not supported on it. The Witty Pi 5 has the
right safe-shutdown logic and a 5 A rail but stops at 30 V. Nothing
found takes 36 to 60 V, runs a Pi 5 at 5 A, halts it on an ignition
signal with a hardware backstop, takes nothing from the pack in
storage, powers the USB devices, and gives switched accessory rails.
The usual substitute is a Mean Well brick plus a Geekworm UPS, two
boxes with no coordination and the 18 V cap. Robot vendors (Clearpath
Jackal, Husarion ROSbot, Duckietown) have a controller ask the computer
to shut down, then cut power. None treats a hard cut as normal
(`docs/pi_power_architecture.md` section 2). This design does the same
with a hardware timer in place of a controller.

Market note. 24 V and 48 V packs are standard on ROS robots, mowers
and AGVs, and the Pi 5 is the common computer. The gap is real but
narrow: the buyers are robot builders, not makers with a bench supply.
A board at 150 to 250 USD with a documented UART protocol and a Pi
daemon would sell in the hundreds, not the thousands. The 12 V-class
tap, the switched outputs and the zero-drain ignition gating are what
the competitors lack most.

## 9. Risks and bench test plan

### 9.1 Risks

| # | Risk | Mitigation |
|---|---|---|
| R-1 | Hot-plug ring: microseconds above the pack voltage, up to about twice it with ceramic input capacitors. It can damage the converters and the VESCs. The battery tap is hot-plugged whenever the pack is connected. | Robot-level problem (3.2a). Margin: a 72 V-class 12 V converter, a 100 V-surge Pi brick, the SMBJ58A ahead of the HSS. The HSS is open at a hot-plug (Cgs holds it off, R-27), so the ring meets the TVS and the HSS, not the brick. T-1 at both inputs. |
| R-2 | Regen overvoltage while the BMS charge FET is open, or from a faulted VESC that coasts and rectifies. The cells do not absorb it. | Robot-level problem (3.2a). Both VESCs are `75_100_V2`, firmware 6.05, with the fault at 90 V and the regen cutoff off. Before the brick goes on: the VESC and BMS targets of `docs/battery_settings.md`, a decision on `l_max_vin` (open question 22), and T-2. The bus dump clamp if T-2 fails (open question 16). See also R-28. |
| R-3 | Brick trim. The Murata 12 V option: +10 percent trim is its limit, not a set point, and its OVP may sit within a volt. The RECOM and TEN 20 Pi bricks: 5.0 V trimmed to 5.1 V. | The TRACO bricks trim by plus or minus 10 percent and have OVP at 6.2 V typical (3.4). VERIFY the other trim ranges, the OVP thresholds and their modes. Sense at the brick pins. Fallback for the Murata: trim to +8 percent (13.0 V). |
| R-4 | The 12 V converter dissipates about 17 W at 150 W out, on the chassis, in the sun. The TEN 30 Pi brick dissipates about 3.2 W at all loads, near its 105 C case limit in a hot enclosure (3.4). | T-3 thermal soak of the converter on the chassis and of the brick in the enclosure. VERIFY the unit's temperature limit and its derating. The TEN 20 makes about 1.4 W less heat. |
| R-5 | The Pi rail does not fall to 0 V (a sneak feed), or it rises too slowly. The Pi 5 then stays in standby on the next ignition. | Supervisor bleed plus fixed bleed (3.8); injectors; UART tri-state; soft-start rule (3.4). T-5, T-6. |
| R-6 | IGN_OFF reads high while the ignition is on (F4 blown, the lead broken, an opto open). | Fail-safe: the Pi halts, and the rail goes off. The robot can drive, but the Pi does not run until the fault is fixed. T-20. |
| R-7 | A hard cut corrupts the NVMe root: a grace cut on a hung Pi, a BMS cut, F3, a brick fault, a loose connector. | The debounce and the daemon's forced poweroff bound a halt at about 43 s, under the 60 s minimum grace (4.4, 6.1). The 30 s system stop cap alone does not: `wall-e.service` sets 90 s. Read-only root (open question 9). T-8. |
| R-8 | The brick's start-up into the Pi's input capacitance trips its current limit, or a boot step browns out the rail. | The TEN 30 takes 14,400 uF maximum, against about 1000 uF on the rail (3.4). VERIFY the limit of the brick that is chosen. T-5. |
| R-9 | A USB 3 injector degrades the OAK link to USB 2 speed. The OAK already enumerates at USB 2 speed today, and the cable is the suspect (`docs/ups_hat_b.md`). | T-10. Fallback: a Luxonis Y-cable from port P1. |
| R-10 | Brick stock and lead time: no TEN 30 at TME, 11 weeks for the RECOM (2026-11-04). | The TEN 20-4811WIN (4 A, 25 in stock) covers the 12.8 W peak. Then keep the USB current rule (3.4). Open question 6. |
| R-11 | The grace timer runs short (a normal halt is cut) or never ends (a hung Pi holds the rail and drains the pack). | Counter timer with an RC oscillator, not a megohm RC (3.7). 10 kOhm reset pull-down against dark current (R-29). The capacitor fixed at 39 nF and Rt trimmed on each board. T-9. A U2B that fails shorted holds the reset; then only a second fault (a hung Pi) drains the pack. |
| R-12 | A forced grace cut re-latches: the logic dies while the brick still has input charge, and the HSS closes again. | Brick remote pin tied ON (3.4). The timer supply holds Q6 on for seconds after the Pi's 3V3 has gone. The hold path needs the Pi's 3V3, and that needs a boot. The 22 kOhm input bleed drains the brick input after the cut (3.5). T-13. |
| R-13 | U2A fails shorted: IGN_OFF always reads low. The daemon never halts, and each ignition-off ends in a grace cut. | T-20 at commissioning. During the grace window, the daemon logs `RAIL_LOST` while IGN_OFF reads low; that pair is the signature. |
| R-14 | Condensation on the pack-voltage tracks (battery side and ignition string). | Conformal coat except thermal pads and connectors. Creepage 2 mm minimum across the isolation barrier and around the battery side. |
| R-15 | EMI from the converters and their leads next to the OAK and the GPS. | Mount the 12 V converter away from the GPS. Twist the 12 V leads. T-15 sniff with both converters running. T-17 repeats it for the chosen 12 V unit. |
| R-16 | The 12 V converter's input floor sits above the BMS cut (40 V on the Vatrer example, 36 V on the HPQ). Motor sag on a low pack can take its input below the floor, and the 12 V rail drops out mid-throttle. | The Pi does not depend on the 12 V rail (3.4). The USB devices drop, and the controller disarms on the RC dropout (5.3). A pack at 40 V is nearly empty, and the VESCs stop at 39 V. If the floor is a problem, choose a unit with a lower floor (Cincon CHB150W-48S12 class, 18 V to 75 V). |
| R-17 | Not applicable to the off-board design: no surge stopper is on the board. In the product spin, a shorted stopper FET passes the TVS clamp (about 113 V) to the converter. | Product spin only: a fuse or a second clamp on the converter side. |
| R-18 | The 12 V converter's current limit into a partial short in its 12 V lead cooks the lead or a joint, if the limit is constant-current. | Select a unit with hiccup mode (3.2). Size the lead for the converter's current limit. T-17. |
| R-19 | No load on a converter makes some units unstable or trip OVP: the 12 V unit with both outputs off, the Pi brick with the Pi halted. | The TRACO Pi bricks need no minimum load (datasheet, 3.4). VERIFY the 12 V unit in T-17. The USB rail and the MCU load the 12 V rail; the fixed bleed loads the Pi brick. |
| R-20 | The 12 V converter fails with its input shorted to its output. The pack voltage reaches the 12 V line, and the USB buck (36 V) and the output switches (40 V) fail in tens of microseconds. | F2 and the SMBJ15A TVS act first (clamp about 24.4 V). The LTC4368 and its 100 V FETs open at 15.0 V. VERIFY that the TVS survives until F2 opens. T-18. |
| R-21 | The battery tap sits on the cell side of the BMS, or its return is B- on a BMS that switches the negative. The BMS cut then no longer protects the pack from the Pi. | Tap the BMS output (3.1). VERIFY the Daly topology and the harness (open question 4). A cell-side tap is an option against regen overvoltage, with this risk as its cost (3.2a, open question 23). |
| R-22 | The battery-tap and ignition connectors are swapped. | Key them differently (3.1, 3.12). |
| R-23 | The Pi stays halted after an e-stop or a flapping switch until a person cycles the main switch. | Designed (5.3). The MCU LED shows it (5.5). |
| R-24 | A robot left with the ignition on runs the pack down to the BMS cut, which is a hard cut for the Pi. | Open question 20: a low-pack halt from the Daly BLE data. |
| R-25 | Voltage margin on the HSS, Q1 and Q2: the SMBJ58A clamps at 93.6 V, and the bus as found can reach 90 V (3.2a). | Parts rated 150 V or more (3.5). Resistors on the battery side rated 200 V (1206 or larger). T-1 records the peak at the battery tap. |
| R-26 | A shorted HSS, Q1 or Q2, or a shorted U1 or U3 output, holds the brick on for ever. With the Pi halted, the brick's no-load loss and the RTK GPS take about 3.7 W from the pack: about 2 days from half charge to the BMS cut (4.3). | Detect only: the MCU reports STUCK_ON (3.11). T-16 and T-22 at commissioning. A second series switch is open question 21. |
| R-27 | Hot-plug turn-on of the HSS. A fast step on the source couples through Cgd, and the Zener then holds Vgs at -12 V. The brick starts with the ignition off. The Pi gets a pulse or a short boot, then a grace cut. | Cgs of 66 times Cgd or more (3.5). T-21. |
| R-28 | The tap TVS acts as a bus clamp. A regen overvoltage above its breakdown (64.4 V minimum, about 62 V cold) lasts longer than microseconds. The TVS burns, fails short and opens F3: a hard cut while the robot moves. The brick also sees more than 75 V. | The bus conditions in 3.2a before the brick goes on: the `docs/battery_settings.md` targets, a decision on `l_max_vin` (open question 22), T-2, and the dump clamp if T-2 fails. The daemon checks the VESC settings at each boot (6.1). |
| R-29 | Heat: the TLP293 dark current (50 uA maximum at 85 C) makes IGN_OFF read "ignition on" and holds the timer in reset. The Pi never halts, and the backstop never cuts. The rail stays on until the BMS cut. One cause defeats both layers. | 10 kOhm pull-up on IGN_OFF and 10 kOhm on MR (3.6, 3.7). T-9 at +85 C. |
| R-30 | A latent timer fault: a dead oscillator or an open Q6. A normal halt ends long before the timer, so normal use never tests it. A later hung Pi then holds the rail. | The MCU checks the timer clock in each GRACE (TIMER_FAULT, 3.11). T-9 at commissioning. Repeat T-8 once a year (VERIFY the interval with Kevin). An open Q6 is not detected; it needs a second fault (a hung Pi) to matter. |
| R-31 | The HSS leakage charges the brick input in storage until the brick starts. The brick then pulses the Pi rail. | 22 kOhm input bleed (3.5): the IRF9640 maximum of 500 uA at 125 C gives 11 V, under the 15 V undervoltage lockout. T-16 at 45 C. |

### 9.2 Bench test plan, before the board goes on the robot

| # | Test | Pass |
|---|---|---|
| T-1 | Hot-plug the real pack into the 12 V converter's input and into the battery tap through the real harness, 10 times each. Scope at the converter input and at the HSS. | Record the peaks. No part on the bus is above 80 percent of its rating. Fail: damp the input (a bulk capacitor with ESR, or a pre-charge). |
| T-2 | Do this before the brick is fitted, with the HAT (B) still on the Pi (3.2a step 5). First do steps 1 to 4 of 3.2a. WARNING: Do not brake hard on a full pack until those steps are done. Scope the bus at the battery tap and at the 12 V converter input at 1 MS/s or more. Log both VESC voltage readings on the same trigger. A 48 Hz CAN log cannot see a spike shorter than about 20 ms. Shots, in this order: (a) on stands, low speed, pack at mid charge; (b) on stands, step up to full speed; (c) on the ground, a full pack just off the charger, hard stops from full speed in MANUAL, a person at the RC; (d) repeat (c) right after a full charge, while the BMS still reports `Cell OVP Trip` (charge FET open; `docs/battery_settings.md`, finding 1); (e) on stands at full speed, release both VESCs (no brake) to see whether the body diodes pump the bus. | Record the peak and its length in each shot. Pass: under 60 V in each shot. Fail: fit the dump clamp (3.2a), then repeat. Both converters stay up. The Pi does not reboot. |
| T-3 | 150 W resistive load on the 12 V rail and 6 A on the Pi rail for 2 h, with the converter bolted to its chassis spot and the board in its enclosure, at 45 C ambient. | Converter case under its limit (VERIFY); the brick case under its limit (VERIFY); board under 85 C; 5 V rails in tolerance. |
| T-4 | 12 V rail: load step 0 to 8 A, short on OUT1 with OUT2 loaded. | Pi rail unaffected; OUT1 limits and retries; OUT2 stays on. |
| T-5 | Pi 5 + NVMe boot on ignition-on, 50 cycles, with the OAK on P1. Scope the 5.1 V rail at the header. | 50 boots; 10 to 90 percent rise under 50 ms; no `throttled` flag; rail dip under 250 mV. |
| T-6 | Ignition off at random points, 50 times. | 50 clean halts (journal shows `Journal stopped`); the HSS opens within 1 s of 3V3 falling; the rail is under 0.5 V within 2 s; PI_3V3 under 0.3 V in standby with the RTK GPS connected (3.7); 50 cold boots on the next ignition. |
| T-7 | Ignition pulse train: 1 s, 3 s, 10 s, 60 s, 150 s off, 5 each. | Shorter than the debounce: no halt, the timer resets. 10 s: the halt completes, and the Pi stays halted with 5 V on until the next ignition cycle (5.3). 60 s and 150 s: one halt, the rail off, one boot when the ignition returns. No boot loop. |
| T-8 | Hung Pi: mask `walle-power.service`, then switch the ignition off. Then, with the daemon on, simulate a stuck camera driver (a process in state D). | First case: the grace cut lands 60 s to 90 s after the ignition went off. Second case: the daemon's forced poweroff halts the Pi under 45 s, and the grace cut does not land. The filesystem check passes on the next boot in both cases. |
| T-9 | Grace timer alone, with a bench 3.3 V on PI_3V3: trim Rt to 75 s at +25 C, then time it at -10 C and +45 C. With the ignition off, heat U2A and U2B to +85 C (heat gun, a thermocouple on the package). Check U1's drive to Q1 at a 5 mA LED current at -10 C and +85 C. | 60 s to 90 s in every case. At +85 C: MR under 1.0 V, the timer expires, and GPIO 27 reads high. Q1's gate gets 5 V or more. |
| T-10 | OAK on P1 through the injector; `getUsbSpeed()` and a 1 h depth stream. | SUPER speed; zero USB resets in `dmesg`. |
| T-11 | Hub on P2; pull the hub's power; probe the Pi-side VBUS pin of the injector. | 0 V on the Pi side; no over-current message in `dmesg`. |
| T-12 | MCU held in reset for 10 min; cycle the ignition twice. | The Pi halts and boots as normal. Outputs off. USB as open question 10 decides. |
| T-13 | Forced grace cut with a real Pi (daemon masked), at no load and at 2 A on the rail. Scope the rail and the HSS gate for 10 s after the cut. | One clean drop. No restart pulse on the rail. The brick input falls under 15 V within 2 s (the 22 kOhm bleed). |
| T-14 | No bridge across the main switch, on the bench with the board off the robot (on the robot the harness can join the two negatives): main switch off; drive the board through each state (ignition off, grace, hold, grace cut). Measure the switched side into 10 kOhm. Measure the resistance across the barrier. | Under 0.1 V on the switched side in every state. Over 10 MOhm across the barrier. |
| T-15 | Near-field probe over the board, the brick and the converter leads, with the OAK, the GPS and both converters running. RTK fix quality before and after. | No new spurs at the GPS L1/L5 bands; fix quality unchanged. |
| T-16 | Storage drain: main switch off, 24 h, at 25 C and again with the board at 45 C. Measure the current in F3, F4 and F1. Probe the brick input and the 5.1 V rail. | Under 50 uA in all (target 10 uA). The brick input stays under 15 V, and the 5.1 V rail stays at 0 V. |
| T-17 | Chosen 12 V converter, before it goes on the robot: no-load and 10 A output voltage; real undervoltage shutdown and overvoltage limit; input at 60 V, 65 V and 68 V for 1 s (a 30 V to 60 V unit) or at 60 V, 70 V and 80 V for 1 s (a 72 V-class unit), from a current-limited supply; output short (which way it fails, what the output does); 10 A for 2 h with the case at 50 C; the RTK fix quality and a near-field probe with it running next to the GPS. | Output 11.6 V to 13.8 V; survives 68 V for 1 s (30 V to 60 V unit) or 80 V for 1 s (72 V-class unit); hiccup on a short; case under its rating; no change in the RTK fix. Any fail: choose another unit, or an isolated brick off the board. |
| T-18 | 12 V IN block on a bench fixture, without the rest of the board: apply 30 V, 54 V and 60 V to the 12 V input from a supply that gives 30 A, into a dummy load on the rail side. | F2 opens. The rail side stays under 36 V for the whole event. A TVS that fails short is acceptable. Nothing burns. |
| T-19 | Chosen Pi brick, before it goes on the board: input at 18 V, 37.7 V, 54.6 V and 75 V; output at 0 A, 1.3 A, 2.5 A and its full rating; input power at each point; output rise time; its output OVP; the crowbar trip from a bench supply on the output with the brick off; the 100 V for 100 ms surge only as its datasheet test allows (VERIFY). | 5.0 V to 5.2 V at every point after the trim; rise under 50 ms; no-load input power near the datasheet (about 3.1 W for the TEN 30, 1.7 W for the TEN 20); the crowbar trips near 5.9 V and the brick survives it. Record the efficiency for 4.1. |
| T-20 | Ignition sense faults: pull F4 with the robot on; then short U2A's output. | F4 pulled: the Pi halts and the rail goes off. U2A shorted: the daemon does not halt, the grace cut lands at 60 s to 90 s, and the daemon log shows `RAIL_LOST` with IGN_OFF low. |
| T-21 | Hot-plug with the ignition off: connect F3 (or the pack connector) 10 times with the main switch off. Scope the HSS gate-to-source voltage, the brick input and the 5.1 V rail. | Vgs stays above -1.5 V. The brick input stays under 15 V. The 5.1 V rail stays under 0.5 V. The Pi's LED stays dark. |
| T-22 | MCU checks: (a) during GRACE, short Ct so the clock stops; (b) fit a test link across the HSS drain and source, then do an ignition-off with the Pi halting normally. Remove the link after the test. | (a) `E,TIMER_FAULT` at the next boot. (b) `E,STUCK_ON` and the fast red LED within 20 s after the timer expires. |

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
4. Taps on the robot. Where is the BMS output on the battery side of
   the main switch (for F3), and where is the switched side (for F1 and
   F4)? Which lead does the main switch interrupt, and which lead does
   the BMS (JHB-R24TH) switch? XT60 or XT90 for the converter's fused lead?
5. Answered 2026-11-04: smart FET (TPS1HB08-Q1).
6. Which Pi brick: the TRACO TEN 30-4811WIN (none at TME), the RECOM
   RP30-4805SFW/N (11-week lead), or the TEN 20-4811WIN (4 A, in stock;
   then `PSU_MAX_CURRENT` stays unset)? New facts, 2026-11-05 (3.4): the
   TEN 20 is a 5.0 V part that needs a small trim, and it has the same
   100 V surge rating. At the Pi's typical load it makes about 1.4 W less
   heat than the TEN 30 (no-load loss 1.7 W against 3.1 W).
7. RTC clock (not a wake path: nothing arms an RTC alarm now). Each
   ignition-on is a cold power-on, and the RTC loses the time unless a
   cell sits on the Pi's J5 connector. Fit a rechargeable cell on J5
   with `rtc_bbat_vchg` and inhibit charging below 0 C, or accept a
   clock jump until GPS or NTP sets it? Journald, TLS and log
   timestamps break on a backward jump.
8. Should the robot ever wake on a schedule while stored? This design
   cannot: nothing has 5 V while the main switch is off.
9. Read-only root with a separate data partition, so that a hard cut
   cannot leave the robot in an `fsck` prompt? With no buffer, a BMS
   cut, a blown fuse and a grace cut are all hard cuts.
10. USB ports: default off with a dead MCU (no camera) or default on
    (camera runs even with a dead MCU)? The old reason for "off" was to
    shed USB before a cut. That reason is gone: the USB rail now falls
    with the 12 V rail. Recommendation: default on.
11. Keep the external self-powered hub behind the injector, or put a
    USB5744 hub on the module in the product spin?
12. The I2C bus guard uses the UPS status file as its witness. Move
    the witness to the GPS reader, or let the new daemon write a
    heartbeat file?
13. Answered 2026-11-04: off the shelf, the converter sits off the
    board.
14. Where does the converter mount on the chassis, and is there bare
    metal under it with a path to the chassis skin? Where does the
    module mount, and how long are the 12 V lead and the battery tap?
15. Answered 2026-11-05 by `docs/battery_settings.md` section 4: the
    regen cutoff ramp for each VESC (with +0.9 V on the right VESC), the
    regen and drive currents, and 13 cells on the right VESC. Kevin
    applies them in VESC Tool.
16. Fit a bus dump clamp on the robot (3.2a)? This review recommends it
    if T-2 shows the bus above 60 V with the settings in place, and as
    the only real fix for braking with the charge FET open. Its size
    needs the robot mass (70 kg is the figure of
    `docs/battery_settings.md`) and the VESC bus capacitance (VERIFY
    both).
17. Which 12 V converter: a 72 V-class unit (recommended), or the
    dkplnt after T-17 passes?
18. Grace time: 75 s nominal (60 s to 90 s)? Shorten it after T-8
    measures the halt on the robot? The halt is now bounded at about
    43 s by the daemon's forced poweroff (4.4). Without the forced
    poweroff, a `wall-e` process in state D can hold the shutdown for
    its 90 s stop timeout, so do not shorten the grace time below the
    bound.
19. Build order: build the Pi power block (3.4 to 3.8) first, on its own
    small board, so that the HAT (B) can retire before the rest of the
    module is built?
20. Low-pack halt: should `walle-power.service` halt the Pi when the
    Daly BLE pack voltage stays low (for example under the 39 V motor
    cutoff for 10 minutes), so that a robot left on does not end in a
    BMS hard cut? The VESC low-voltage cutoff for 13S is answered by
    `docs/battery_settings.md` section 4 (2026-11-05). The halt itself
    is still open.
21. Single points of failure in the switch (risk R-26). A shorted HSS,
    Q1, Q2 or PV opto output holds the brick on until the BMS cut. The
    MCU only reports it. Accept this for spin 1, or add a second series
    switch now?
22. `l_max_vin`: keep 90 V (`docs/battery_settings.md`) or lower it to
    about 61 V on the left VESC and about 61.9 V on the right VESC (3.2a
    step 3)? A lower fault keeps the tap TVS out of breakdown and the
    brick inside its rating. Each fault releases the motor for about
    500 ms, so it is a brake-loss event at a lower voltage. The regen
    ramp at -5 A and the BMS overvoltage fix are what keep the bus from
    reaching either value.
23. Tap the brick on the cell side of the BMS (B+ to B-) in place of the
    BMS output (3.2a)? The brick then never sees regen overvoltage, and
    a BMS low-voltage cut gives a clean halt. The cost: a stuck-on HSS
    can then drain the cells below the BMS cut.
24. Answered 2026-11-05 (Kevin, `docs/battery_settings.md`): the pack is
    13S NMC, 7 Ah. Section 4.3 uses it.
25. VESC Tool checks for Kevin, before 3.2a step 3: the voltage rating of
    the VESC bus capacitors, and the fault stop time
    (`m_fault_stop_time_ms`, about 500 ms by default) that releases the
    motor after an over-voltage fault.

## 11. Appendix: Grok second opinions

Six `grok_second_opinion` calls, Grok 4.7, 2026-11-04, single shot, no
repository access. Sessions `01a1a72a`, `01a1a730`, `01a1a737` (11.1 to
11.3, 0.51 USD in all) and the three below (11.4 to 11.6, 0.37 USD). A
seventh (session `01a1a8bd`) reviewed option A for
`docs/pi_power_architecture.md` (11.7). An eighth (session
`01a1ac7c-45bd-7c21-98d7-19cc90ed9e0d`, 2026-11-05, medium effort,
0.18 USD) attacked the findings of the hardware review (11.8). Section
references inside 11.1 to 11.6 point to the design at the time of each
review.

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

### 11.2 and 11.3 (removed design)

11.2 (the charger and BMS) and 11.3 (cut-and-wake) reviewed the 2S
buffer design. Option A removed that design on 2026-11-04. Their tables
are in the git history of this file (commit 037cc80). These points from
11.3 carry over:

- Sense 3V3 through a high impedance, and source nothing into the pin
  (3.7, 3.9).
- Do not use `gpio-poweroff`. It interferes with the PMIC standby path
  (6.3).
- Bound the systemd stop timeouts, and consider a read-only root (6.3,
  open question 9).
- No automatic wake after an e-stop (5.3).
- The UART daemon is a head start, not the thing the cut depends on.
  Option A goes further: no firmware is in the Pi power path (3.11).
- A primary coin cell with charging enabled is a fire. A rechargeable
  cell must not charge below 0 C (open question 7).

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
| A 57-58 V clamp sits about 3 V above a full pack; the stopper FET then absorbs ordinary regen braking on a full pack until its timer opens, which is how these FETs fail short. | Accepted. The clamp stays at 68 V. The converter must survive 68 V on the bench (T-17). Open: the VESCs' own bus limit is not recorded in the repository; if they fault near 57 V, regen cannot hold the bus above the clamp. Closed 2026-11-04: both drive VESCs are `75_100_V2` (75 V class), firmware 6.05, so they do not fault near 57 V (3.2a). They fault at 90 V (`l_max_vin`, read 2026-11-04), which is above the brick and the tap TVS; 3.2a now sets conditions for that. |
| A series disconnect can lose the race to a shorted high-side FET; the BQ24610 (33 V) dies in tens of microseconds. | Accepted. Fuse and a TVS under 28 V before the disconnect. (Option A removed the BQ24610; the USB buck at 36 V is now the weakest part, 3.1.) |
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
| The VESC "Battery Voltage Cutoff Start/End" is the low-voltage cutoff, not a regen limit. | Accepted. WARNING added in 3.2a. Firmware 6.05 has the separate "Battery Voltage Regen Cutoff Start/End" (3.2a). |
| A faulted, coasting VESC rectifies through its body diodes, and no setting stops that. | Accepted as a robot-level risk (3.2a). It needs a motor back-EMF above the bus voltage, which the robot's `max_erpm` 20000 cap may prevent. `config.py` puts the back-EMF at the bus near 22,000 eRPM, only about 10 percent above the cap (3.2a). VERIFY against the motor KV. |
| Golf-cart converters assume a pack that stays connected. A steady bench test says nothing about a 2x ring. | Accepted. A 72 V-class unit is the default (3.2). |
| A TVS covers the hot-plug ring, not braking energy. | Accepted (3.2a). |
| Keeping the stopper protects only the converter, never the VESCs. | Accepted. The stopper is dropped from the board (3.1). |
| The halt must also start on loss of the 12 V rail. | Then: the rail-lost rule. Option A replaces it: the Pi runs from the battery tap, so a 12 V rail loss does not threaten the Pi (5.3). |
| The overvoltage disconnect must withstand pack voltage if the converter shorts its input to its output. | Accepted. 100 V-rated parts, and F2 coordinated with the TVS (3.1, R-20, T-18). |
| Buy voltage margin or add a dump clamp. | Accepted. A 72 V-class unit is the default. The clamp is open question 16. |
| Do not run T-2 on a full pack to discover this. | Accepted. T-2 is rewritten with a WARNING and a low-speed start. |

### 11.7 Option A (2026-11-04)

Recorded in `docs/pi_power_architecture.md` section 4 (Grok, session
`01a1a8bd`). Proposed: a gated tap on the main pack, with the enable
taken from the ignition.

| Objection | Response |
|---|---|
| A 60 V module (Pololu D42V55F5) cannot be made safe on 13S with a TVS: one that stays off at 54.6 V clamps near 90 V (SMBJ58A: 93.6 V). | Accepted. A brick rated 75 V continuous and 100 V for 100 ms (3.4). |
| An `ignition OR 3V3` enable alone lets a hung Pi hold the converter on and drain the pack in one to two weeks. | Accepted. A hardware grace timer forces the HSS off 60 s to 90 s after the ignition goes off (3.7). |

### 11.8 The hardware review (2026-11-05)

One `grok_second_opinion` call, Grok 4.7, medium effort, session
`01a1ac7c-45bd-7c21-98d7-19cc90ed9e0d`, 0.18 USD. Proposed: the VESC
settings of 3.2a as preconditions (with `l_max_vin` at 61 V), and the
circuit fixes of this review. Against: `l_max_vin` left at 90 V with a
dump clamp, and the old pull resistors and timer capacitor range.

| Objection | Response |
|---|---|
| A lower `l_max_vin` is the wrong actuator. An over-voltage fault opens the bridge, which then rectifies while the line-line back-EMF is above the bus, so the fault can make a spike worse. | Accepted in part. The fault is a "stop and coast" backstop, not a bus regulator (3.2a step 3). It bounds the bus only while the back-EMF is below it. `config.py` puts that point near 22,000 eRPM, above `max_erpm` 20000, so it holds on flat ground. T-2 shot (e) checks it. The lower value stays a proposal, because at 90 V the tap TVS burns; `docs/battery_settings.md` keeps 90 V. Kevin decides (open question 22). |
| The regen cutoff starts after the Daly has opened its charge FET (about 55.25 V). On a bus with only capacitors, a filtered software limit overshoots. Put the cutoff end below the Daly open point. | Accepted. Answered by `docs/battery_settings.md` section 4 (2026-11-05): the ramp ends below the BMS pack overvoltage trip, and the regen current falls to -5 A for each VESC (3.2a). |
| One VESC can fault first (the readings are 1 V apart). That side coasts and the robot yaws. Confirm whether the fault latches. | Accepted. The right VESC reads 0.9 V high (`docs/battery_settings.md`, finding 7), so each threshold gets +0.9 V there. CAUTION in 3.2a, open question 25. |
| Set `l_max_vin` from the bus capacitor rating, above anything the dump clamp lets the bus reach. | Accepted (3.2a step 3, open question 25). |
| T-2 needs a shot with the charge FET open and a shot with both VESCs released at speed, at 1 MS/s, with both VESC readings on one trigger. | Accepted. T-2 is rewritten. |
| The dump clamp needs its own fuse. Do not crowbar the traction bus. | Accepted (3.2a). |
| A series diode in the brick feed separates the brick from the traction limits. | Rejected. A diode blocks reverse current only. A bus overvoltage passes through it forward. The cell-side tap gives what the objection wants, at a cost (3.2a, open question 23). |
| Cgs of 1 uF or more against hot-plug turn-on, because the Zener holds the FET on once it clamps. Give each small N-FET gate its own pull-down. | Accepted (3.5, R-27). |
| A 100 kOhm input bleed is too light for the hot IDSS. Use 10 to 22 kOhm. | Accepted: 22 kOhm (3.5, R-31). |
| 47 kOhm on MR and the Pi's pull-up alone fail at 85 C. Use 10 kOhm to the Pi's own 3V3, and 10 kOhm on MR. | Accepted (3.6, 3.7, R-29). This review found the same. |
| 39 nF against 47 nF is false precision. The RC oscillator moves by tens of percent. Trim each chip and measure it hot and cold. | Accepted in part. Each board is trimmed (T-9). 47 nF stays excluded: with the Nexperia constant it starts above the 90 s maximum. |
| If the latch diode fails, pin 3 wraps low and GRACE_EXPIRED clears: the rail stays on for ever. Power the 74HC4060 from the battery side, so that a brown-out cannot clear GRACE_EXPIRED. | Rejected. The first expiry already cuts the rail, and the hold path needs the Pi's 3V3, which needs the rail. The timer runs from the same rail with 220 uF of hold-up, so it cannot brown out while the rail is up. A battery-side timer needs a supply in storage and another link across the barrier. |
| `TimeoutStopSec=10` on `wall-e.service` alone does not bound the halt. Cap the default too. | Accepted in part. The daemon now forces the poweroff at 25 s and 35 s (6.1), which bounds the halt whatever the unit timeouts are. The runtime drop-in stays as the first step. |

## Appendix: Technical Names

Raspberry Pi 5, Pi 5, Raspberry Pi, NVMe, GPIO, PMIC, STANDBY, EEPROM,
`POWER_OFF_ON_HALT`, `WAKE_ON_GPIO`, `PSU_MAX_CURRENT`,
`usb_max_current_enable`, `rpi-eeprom-config`, `rtc_bbat_vchg`, J2, J5,
PWR_BUT, RTC, UART, UART0, `/dev/ttyAMA0`, `serial0`, `ttyAMA10`,
`dtoverlay`, `uart0-pi5`, `gpio-shutdown`, `gpio-poweroff`,
`config.txt`, `/boot/firmware/config.txt`, systemd, `systemctl`,
`TimeoutStopSec`, `DefaultTimeoutStopSec`, `fsck`, `dmesg`, journald,
SIGKILL, `poweroff`, `sync`, `sudo shutdown -h now`, libgpiod,
`wall-e.service`, `walle-ups.service`, `walle-ups-wakecheck.service`,
`walle-power.service`, `bin/walle_ups.py`, `bin/walle_power.py`,
`bin/walle-ups-shutdown-alarm`, `bin/walle-fast-stop-system.conf`,
`bin/walle-fast-stop-user.conf`, `bin/install_walle_ups.sh`,
`chain.json`, `i2c-bus-guard.service`, `/tmp/ups_status.json`, `/debug`,
`POST /api/power/out`, `controller.py`, `config.py`, `CLAUDE.md`,
`cmd.is_armed`, `module_alive`, `ignition_off`, `throttled`, ch5,
e-stop, VESC, VESCs, VESC 6, `75_100_V2`, firmware 6.05, VESC Tool,
Battery Voltage Regen Cutoff Start/End, Battery Voltage Cutoff
Start/End, CAN, MCP2515, SPI0, Daly, BMS, BMS output, discharge FET,
deep-discharge cut, B-, P-, BLE, 13S, 14S, 2S, 18650, Li-ion, LiFePO4,
LiPo, NTC, OV, XT60, XT90, Molex Micro-Fit 3.0, JST-PH, HTSSOP,
VQFN-HR, LQFP, SOT-23, 2512, C0G, TVS, SMCJ58A, SMCJ70A, SMBJ18A,
SMBJ15A, SMBJ58A, Vishay, LT4363, LM5069, IPB072N15N3, BSC160N15NS5,
BSC098N10NS5, BSC160N10NS3, LM5146-Q1, LM5145, LM5013, TEP 150-4815WI,
TRACO, TEN 30-4811WIN, TEN 20-4811WIN, RECOM, RP30-4805SFW/N, Mean Well
RSD-60L-5, Pololu D42V55F5, telecom brick, brick, remote pin, off
current, trim pin, soft start, TME, DigiKey, Murata, HPQ-12/25-D48NB-C,
MPQ1500-12V125-L48, PMBus, MFR_VOUT_MAX, dkplnt, LTC4368, 12 V IN,
Vatrer, IP55, golf-cart converter, DC-DC, potted, hiccup, regen,
back-EMF, body diode, MOSFET, bus dump clamp, `max_erpm`, KV, F1, F2,
F3, F4, Flex, PKB4313DPI, Cincon, CHB150W-48S12, eighth brick, half
brick, quarter brick, baseplate, OVP, OVLO, Mean Well SD-150C-12,
TPS1HB08-Q1, BTS7008-2EPA, PROFET, TE T9AS1D12-12, Omron G8P, LM61495,
LM61495-Q1, LMQ61460, TPS25221, TPS2561, USB5744, BQ24610, INA228,
TL431, SCR, crowbar, bleed, switched bleed, fixed bleed, supervisor,
TPS3839, TPS3840, high-side switch, HSS, P-FET, N-FET, IRF9640, IDSS,
Vgs, Vgs(th), Zener, Schottky, gate network, inrush, opto, PV opto,
photovoltaic opto, VOM1271, phototransistor, Toshiba TLP293, CTR, dark
current, LED string, 74HC4060, CD4060B, Q14, TPL5110, TPL5111,
STM32G071, STM32G071CBT6, RP2040, RP2350, MicroPython, TPS7B81-Q1,
TPS7B69, LDO, 2N7002, BSS138, Littelfuse, LP-JCASE, NANO2, OAK-D Lite,
OAK, Luxonis, `getUsbSpeed`, USB, USB 3, USB 2, USB-C, USB-A, Type-A,
VBUS, SuperSpeed, SSTX, SSRX, D+, D-, injector, Arduino, RC, RTK, GPS,
L1, L5, NTP, TLS, Grok, Grok 4.7, `grok_second_opinion`, Kevin,
WALL-E, Texas, Witty Pi 5, UUGear, HAT+, StromPi 3, Joy-IT, Geekworm
X1202, Geekworm UPS_Scap, DFRobot FIT0992, Waveshare UPS HAT (B),
Waveshare UPS HAT (E), UPS, SunFounder PiPower 5, Sequent, Super
Watchdog, PiJuice, Sfera Labs, Strato Pi UPS, mjbots, VBCores, ROS,
AGV, Clearpath Jackal, Husarion ROSbot, Duckietown, Mean Well, eFuse,
surge stopper, SYS bus, pack-sense lead, main switch, battery side,
switched side, battery tap, ignition, ignition lead, isolation barrier,
system ground, Pi power block, ENABLE, HOLD, hold path, grace timer,
grace window, grace cut, flapping, debounce, heartbeat, watchdog,
CRC-8, ASCII, firmware, spin, product spin, PCB, four-layer, 2 oz,
thermal via, conformal coat, creepage, RDS(on), ESR, EMI, PD, Power
Delivery, Stop mode, ADC, I2C, 3V3, GND, RGB LED, buzzer, FAULT, BATT
TAP, IGNITION, IGN_OFF, PI_3V3, GRACE_EXPIRED, PI_HALTED, PI_5V_SENSE,
SHUTDOWN_REQ, OUT1, OUT2, P1, P2, P3, P4, Q1, Q2, Q5, Q6, U1, U2A,
U2B, U3, STORED, RUNNING, GRACE, HALTED_IGN_ON, GRACE_CUT, RUN,
RAIL_LOST, RAIL_BACK, USB_FAULT, HALT_NOW, SHUTDOWN_PENDING,
INPUT_LOST, INPUT_BACK, ARMED, DISARMED, TOPUP, TOPUP_START, TOPUP_END,
HB, `walle-power/1`.

Added by the 2026-11-05 review: `l_max_vin`, `l_battery_regen_cut_start`,
`l_battery_regen_cut_end`, `l_in_current_min`, `l_battery_cut_start`,
`l_battery_cut_end`, `si_battery_cells`, `si_battery_ah`,
`tools/vesc_info.py`, `candump`, eRPM, duty, over-voltage fault, stop
and coast, one-sided braking, yaw, FOLLOW_ME, WAYPOINT_NAV, MANUAL,
Daly app, overcharge protection, charge FET, B+, P+, cell-side tap,
dump clamp, kinetic energy, bus capacitance, DC-link capacitor,
avalanche, breakdown voltage, standoff, temperature coefficient, Cgs,
Cgd, Miller plateau, safe operating area, SOA, dv/dt, hot-plug
turn-on, input leakage bleed, undervoltage lockout, UVLO, no-load loss,
no-load input current, flyback, case temperature, thermal pad,
TEN-HS1, KY, EN 61000-4-5, "passive = on", -Vin, negative logic,
positive logic, Nexperia, CD74HC4060, TI, MR, Rt, Ct, Rs, Schmitt
trigger, 1N4148, Q4, Q13, VIL, VIH, ICEO, TLP293-4, CTR rank,
open-circuit voltage, short-circuit current, 1206, X7R, 5 V-tolerant
pin, FT pin, STM32G071RBT6, LQFP48, USART, TIMER_FAULT, STUCK_ON,
`E,TIMER_FAULT`, `E,STUCK_ON`, `CLR,faults`, flash, state D,
uninterruptible sleep, runtime drop-in,
`/run/systemd/system/wall-e.service.d/50-halt.conf`,
`systemctl daemon-reload`, `systemctl kill`, `--signal=SIGKILL`,
`systemctl poweroff --force`, `DefaultDependencies=no`, remount
read-only, SMBJ60A, AWG, RP1, MS/s,
EEPROM recovery, 3.15 A slow-blow, time-delay fuse, breaking capacity.
Added by the 2026-11-05 update: `docs/battery_settings.md`, JHB,
JHB-R24TH-V1.3, Daly-app compatible, NMC, Ternary, LTO, XM2703, `Cell
OVP Trip`, cell overvoltage protection, pack overvoltage protection,
pack undervoltage protection, charge over-current protection, discharge
over-current protection, charge low-temperature protection, discharge
FET, `l_in_current_max`, `m_fault_stop_time_ms`, fault stop time,
brake-loss event, regen ramp, motor setup wizard, reading offset.
