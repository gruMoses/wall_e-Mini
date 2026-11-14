# RC receiver, Arduino and RTK GPS wiring

Status: draft, 2026-11-13. This document records the wiring as of
2026-11-13: the USB hub is removed, the Arduino is on a USB port of the Pi,
and the OAK-D Lite takes its 5 V from the UPS. It is the input for a Pi 5
shield specification (section 11). It does not design the shield.

WARNING: Few items in this document are measured. On 2026-11-13 the Arduino
was connected to the Pi for a flash read (section 10, check 2). The other
sources are Kevin's statements, the code, the logs, the HotRC DS600 manual
and the DFRobot wiki. Section 10 lists each open check.

## 1. Confidence levels

- **Confirmed:** the code reads the part, a log shows the behaviour, the
  maker's manual gives the value, or Kevin states it.
- **Inferred:** the code or the documents make the item necessary, but
  nobody saw it.
- **Unknown:** no source has the item.

## 2. Diagram

![RC receiver, Arduino and RTK GPS wiring](rc_arduino_gps_wiring.svg)

Solid lines are Confirmed. Orange dashed lines are Unknown or Inferred.

## 3. Parts

| Part | Model | Confidence and source |
|---|---|---|
| Transmitter | HotRC DS600 V3, LCD model, sold as Havcybin | Confirmed, 2026-11-04 (`docs/NEXT_SESSION.md`) |
| Receiver | HotRC F-06A (V3): 6 channels, PWM, 4 V to 9 V DC | Confirmed: Kevin read "F-06A" on the label, 2026-11-13. This agrees with the V3 transmitter. Kevin reports no power row, but the published F-06A drawing and photo show a seventh row with a power symbol (section 4). Research, 2026-11-13: that row is EXT-V, a voltage sense input, not a power row. |
| RC bridge MCU | Arduino Nano clone: ATmega328P at 16 MHz, CH340 USB serial chip, Nano old bootloader (57600 baud) | Confirmed 2026-11-13: `lsusb` shows `1a86:7523`, avrdude reads signature `0x1e950f` at 57600 baud, and the last flash used FQBN `arduino:avr:nano:cpu=atmega328old`. Nobody photographed the board. |
| Former motor driver | OSOYOO Model X H-bridge | Removed. Kevin, 2026-11-13: the Arduino used to control it. Today the Arduino only streams the RC channels to the Pi over USB. The flashed firmware still has the Model X code; the repo sketch does not since 2026-11-13 (section 5). |
| USB hub | Genesys Logic, USB IDs 05e3:0610 and 05e3:0626 | Removed (Kevin, 2026-11-13). Before that, the Arduino and the OAK-D Lite were on it, and it had its own supply from the main pack (`docs/ups_hat_b.md`). |
| Camera (context) | Luxonis OAK-D Lite | USB data to the Pi. The 5 V wire of its cord is cut, and it takes 5 V from the UPS (Kevin, 2026-11-13). Not part of the shield. |
| RTK rover | DFRobot KIT0198: Quectel LC29HDA, LoRa 915 MHz, I2C address 0x20 | Confirmed (`docs/rtk_gps_module.md`) |

DS600 manual (V3, text as outlines, read as images):
<https://www.flyingtech.co.uk/wp-content/uploads/2023/11/DS600-Manual-EN-.pdf>.
DFRobot wiki: <https://wiki.dfrobot.com/kit0198-us/>.

## 4. Receiver to Arduino

| Receiver row | Arduino pin | Use | Thresholds on the Pi |
|---|---|---|---|
| CH1 signal | D2 | Left track. The transmitter mixes the stick (LCD icon 1). | Deadband ±25 µs about 1500 µs |
| CH2 signal | D3 | Right track | The same |
| CH3 signal | D4 | Arm switch | Arm: 1800 µs or more, held for 0.2 s. Disarm: 1200 µs or less, or 0.5 s without a held-high reading. |
| CH4 signal | D5 | FOLLOW_ME | Start at 1800 µs or more while armed. Stop at 1200 µs or less. |
| CH5 signal | D6 | E-stop | 1800 µs or more: stop, latch DISARMED, power the Pi off after 5 s. |
| CH6 signal | A0 | Not used yet | Kevin, 2026-11-13: connect the lead now. The flashed firmware does not read A0. The repo sketch (2026-11-13, not flashed) sends A0 as the sixth field, and the Pi does not use it. A0 stays an input. Do not use D7 to D12 until the new sketch is flashed: the flashed firmware drives them as outputs (section 5). |
| + and − | Arduino side, on the USB 5 V | Receiver power | Section 6 (Kevin, 2026-11-13) |

Receiver header (HotRC wiring guide for the F-06A, Howes Models, and the
manual drawing):

- Male pin header, 2.54 mm (0.1 in) pitch. Each channel is one row of 3
  pins: signal, +, −. Signal is next to the label, and − is on the outer
  edge. A servo lead has the same order: orange or white is signal, red is
  +, brown or black is −.
- The F-06A has 7 rows: CH1 to CH6, and a seventh row with a power symbol
  under CH6. Kevin, 2026-11-13: the robot's receiver has no power row. On
  most RC receivers, the + pins of all rows connect together, and the −
  pins also, so power can go in on any channel row. VERIFY this on the
  robot's receiver with a meter (continuity, power off).
- Research, 2026-11-13 (F-06A manual mirror on manuals.plus, retailer listings): the
  seventh row is EXT-V, the voltage-return SENSE input (0 V to 27 V, a
  lead to the model battery for the transmitter's display). It is not a
  power row. The receiver takes its supply on any channel row.
- Photo and pin drawing:
  <https://howesmodels.co.uk/product/ds600-6-channel-2-4ghz-receiver-for-single-hand-radio-control/>.

The Arduino Nano layout, with the pin use on this robot, is in
`docs/arduino_nano_pinout.svg`.

Electrical facts:

- The F-06A has two output modes (manual page 6). Blue LED off: analogue
  servo mode, 50 Hz, 1000 µs to 2500 µs. Blue LED on: digital servo mode,
  333 Hz, 500 µs to 2500 µs. A 3 s press of BIND changes the mode. The logs
  show 992 µs to 2004 µs. The mode on the robot is Unknown.
- The sketch measures each pulse with `micros()` (4 µs steps) in a
  pin-change interrupt. The flashed firmware changes a width outside
  800 µs to 2200 µs to 1500 µs before it sends the line. The repo sketch
  (2026-11-13, not flashed) ignores a pulse outside 500 µs to 2500 µs. It
  sends the last valid width until 100 ms after that pulse ended, then `0`
  (section 7).
- D2 to D6 use `INPUT_PULLUP` (internal pull-up of 20 kOhm to 50 kOhm to
  5 V).
- The manual does not give the signal voltage. An ATmega328P at 5 V reads
  a high level at 3.0 V or more (0.6 × VCC). A 3.3 V output thus has 0.3 V
  of margin. VERIFY (section 10, check 5). Research, 2026-11-13: HotRC publishes no
  signal level. No scope capture or teardown of this receiver was found.
- Failsafe: the HotRC manuals of related receivers give a preset position
  on signal loss, set on the transmitter. No document gives the behaviour
  of the F-06A with the DS600 V3. Checks 6 and 7 stay open.

## 5. Model X code in the sketch (board removed, code removed from the repo sketch)

Kevin, 2026-11-13: the Arduino only streams the RC channels to the Pi over
USB. No device connects to its output pins. The Arduino used to control the
Model X, a hobby motor driver board from the first prototype (its code is in
the initial commit, 2025-08-15, before the VESCs). The board is removed. The
VESCs on CAN drive the robot.

The flashed firmware (`dcc550e`) still has the Model X code. At start it
sets D7 to D12 as outputs and drives them low. It drives them again when
the Pi sends `M,<left>,<right>`.

The repo sketch has no Model X code since 2026-11-13 (Kevin: "Remove it
now"). It does not touch D7 to D12, so they stay inputs without pull-up, the
reset default. It reads and discards all serial input from the Pi.

The Pi side is not changed. `ArduinoModelXDriver`
(`pi_app/hardware/arduino_modelx.py`) still sends `M,<left>,<right>` when the
Pi finds no VESC on CAN: `pi_app/app/main.py` ("VESC not detected") and four
calibration tools in `pi_app/cli/` do this. These writes are harmless: the
link has no flow control, so they do not block, and the new sketch ignores
them. They drove nothing before either, because the board is removed.
Without `can0` the robot has no drive path.

WARNING: Do not connect an input, for example CH6, to D7 to D12 while the
flashed firmware is on the Nano. That firmware drives those pins as
outputs.

Former pin map, for reference:

| Arduino pin | Model X pin | Use |
|---|---|---|
| D10 | ENB | Left PWM |
| D7 | IN3 | Left direction 1 |
| D8 | IN4 | Left direction 2 |
| D9 | ENA | Right PWM |
| D12 | IN1 | Right direction 1 |
| D11 | IN2 | Right direction 2 |

Status: removed from the repo sketch (2026-11-13). It leaves the Nano with
the next flash (section 9.1).

## 6. Power and ground

- The Arduino connects to a USB port of the Pi with a normal cable. The Pi
  supplies its 5 V through that port (Kevin, 2026-11-13). The USB hub is
  removed.
- The Pi 5 limits the current of all its USB ports together to 600 mA,
  unless its firmware knows that the supply gives 5 A. The Arduino and the
  receiver must stay below that limit. Nobody measured the receiver
  current. The F-06A listing gives 35 mA (research, 2026-11-13).
- The receiver gets its power from the same USB 5 V as the Arduino (Kevin,
  2026-11-13). The Arduino and the receiver do not connect to the 5 V of the
  Pi header. The exact pin on the Arduino side is not recorded; the 5V pin
  is the probable one. The receiver has no power row (Kevin), so the power
  goes in on the + and − pins of a channel row. Kevin, 2026-11-14: the
  receiver's + lead is on the Nano's 5V pin (photo
  `docs/data/shield_inputs_20261113/nano_receiver_power_5v_20261114.jpg`),
  thus after the Nano's USB diode, not on VIN. The F-06A takes 4 V to 9 V
  DC (manual page 5). The manual note gives 12 V as the maximum.
- The RTK rover is the only part in this document on the 5 V of the Pi
  header (Kevin, 2026-11-13).
- The receiver and the Arduino must have a common ground for the PWM
  signals. The USB ground connects the Arduino ground to the Pi ground.
- The RTK rover gets 5 V from the Pi header (section 8).
- Context: the OAK-D Lite takes its 5 V from the UPS through the cut 5 V
  wire of its cord, not from the Pi (Kevin, 2026-11-13). This is the
  "variant without the hub" in `docs/pi_power_architecture.md`.

## 7. Arduino to Pi (USB serial)

- Port: `/dev/ttyUSB0`, on a USB port of the Pi with no hub (USB path 3-2
  on 2026-11-13). The reader tries `/dev/ttyUSB*` first, then
  `/dev/ttyACM*`.
- Rate: 115200 baud. 8N1 is Inferred (the Arduino default).
- Arduino to Pi: one line each 20 ms, in µs. The rate is fixed. It does not
  follow the receiver frame rate. After a reset, the sketch waits 1 s and
  sends `RC Reader initialized`. The Pi parser drops that line, so no log
  shows it. There are two line formats:
  - **5 fields, `ch1,ch2,ch3,ch4,ch5`.** The flashed firmware (`dcc550e`,
    section 10, check 2) sends this format. Each field is 800 µs to
    2200 µs, or 1500 µs for any other width (finding 5).
  - **6 fields and a CRC, `ch1,ch2,ch3,ch4,ch5,ch6*HH`.** The repo sketch
    sends this format since 2026-11-13 (not flashed). Example:
    `1500,1500,2000,1000,1000,1500*25`. CH6 is the A0 input. Each field
    is a width from 500 µs to 2500 µs, or `0`, the "no signal" value
    (`RC_NO_SIGNAL_US`). One rule: a channel sends the width of its last
    valid pulse (500 µs to 2500 µs) until 100 ms after that pulse ended,
    then `0`. Before its first valid pulse, it sends `0`. The sketch
    ignores a pulse outside the range ("hold last good", Kevin,
    2026-11-13), so one glitch changes nothing, and a stream of bad pulses
    gives `0` 100 ms after the last valid one. `HH` is a CRC-8 (polynomial
    0x07, start value 0) of the text
    before `*`, in two upper-case hexadecimal digits. The logic is in
    `arduino_rc_reader/rc_channel.h`.
- Why a CRC (Grok review, 2026-11-13): with the range 500 µs to 2500 µs, a
  lost byte can give a valid width. For example, `1500` without its `1` is
  500 µs: a full stick on CH1 or CH2, or "switch OFF" on CH3, which clears
  the re-arm latch. The old range (800 µs to 2200 µs) caught that case. On
  a test of 91,531 single lost bytes, the CRC and the range let no wrong
  line through. Two lost bytes got through 2 times in 60,000.
- Why 500 µs to 2500 µs (requirement S1): the F-06A sends 1000 µs to
  2500 µs in its 50 Hz mode and 500 µs to 2500 µs in its 333 Hz mode
  (manual page 6). A narrower range changes a valid pulse of the 333 Hz
  mode to "no signal". A wider range lets through widths that no receiver
  mode sends.
- Why 100 ms (requirement S2): it is five 50 Hz frames. The minimum is
  60 ms (three frames). Worst case from the last valid pulse to the Pi's
  reaction is about 155 ms: the 100 ms timeout, up to 20 ms to the next
  line, about 3 ms on the serial link, and up to 33 ms to the next tick of
  the Pi's 30 Hz control loop (review, 2026-11-13). Until then the robot
  uses the last valid widths.
- Each time a program opens the port, the DTR line resets the Arduino. The
  Pi start-up opens the port two times (detection, then the reader), so the
  Arduino resets two times.
- Pi to Arduino: `M,<left>,<right>`, for the removed Model X only. The Pi
  sends it only when it finds no VESC on CAN (section 5). The bytes are 0 to
  254, neutral 126. The sketch accepts 0 to 255, but byte 255 overflows its
  PWM calculation (`abs(delta) * 255` in a 16-bit `int`). The Pi never sends
  255.
- The Pi parser (`pi_app/hardware/arduino_rc.py`, changed 2026-11-13, not
  deployed) accepts the two formats (requirement S12). Each field must be
  plain digits. In a 5-field line, each field must be 800 µs to 2200 µs,
  the only values that `dcc550e` sends. In a 6-field line, the CRC must
  agree, and each field must be 500 µs to 2500 µs or `0`. The parser drops
  the full line if one field is bad, so a bad line does not refresh the
  frame. It drops a line with 4 fields: no firmware since `5f275d0` sends
  that format, and today such a line is the end of a cut 5-field line, with
  each channel moved one place. It also drops a line that can be a part of
  a line: a read that has no line end, the line after that read, the first
  line after the port opens, and a 5-field line after a 6-field line on the
  same port. The flashed firmware sends only widths from 800 µs to 2200 µs,
  so its lines give the same values as before
  (`pi_app/tests/test_rc_no_signal_parser.py`).
- The Pi keeps `0` in the channel value, so the logs show it. All code that
  compares a channel with a threshold uses `pi_app/control/rc_signal.py`.
  The Pi reads "no signal" as follows:
  - CH3 or CH5: the robot disarms at once, without the debounce. The Pi
    sets the re-arm latch, so the arm switch must be cycled (OFF, then ON)
    after the signal comes back. This is not the e-stop: the Pi does not
    power off. The journal line is `RC CH3 NO SIGNAL: ...` or
    `RC CH5 NO SIGNAL: ...`, and the event is `RC_NO_SIGNAL_DISARM`.
  - CH5 that comes back with a measured high (1800 µs or more) is a rising
    edge, so it starts the e-stop, as at power-on (Kevin accepted this,
    2026-11-13).
  - CH1 or CH2: both tracks are neutral. With only the dead track neutral,
    a held stick on the other track turned the robot in place. When the
    signal comes back under a held stick (armed, MANUAL), the command ramps
    up from neutral at the MANUAL rate. It does not jump to the stick.
  - CH4: FOLLOW_ME stops. The robot stays armed in MANUAL (Kevin accepted
    this, 2026-11-13). FOLLOW_ME cannot start again until CH4 is held low.
    A middle-band value is not sufficient.
  - Every FOLLOW_ME exit sets the CH4 latch (Kevin, 2026-11-13: "Yes, held
    low"), not only a CH4 no signal. The exits are: CH4 low (one frame is
    enough), CH4 no signal, a disarm (CH3 low, CH3 or CH5 no signal, the
    middle-band disarm), the e-stop, an RC-stale link, a
    stale camera stream, the pack guard, the web or phone stop, the
    gesture stop, the calibration wizard, and a start that the controller
    refuses (no target, pack guard). A later start by CH4 then needs CH4
    held low for 0.2 s after the exit, then a rising edge. A low that
    started before the exit does not count. Thus one bounced low frame cannot
    stop FOLLOW_ME and start it again. A start from the web page or by a
    hand gesture is an operator act and is not blocked.
  - "Held low" (Kevin, 2026-11-13): a measured low (1200 µs or less) on
    each control tick for 0.2 s (`SafetyParams.latch_clear_low_persist_s`).
    Only a held low clears the re-arm latch (every cause: start-up, auto
    disarm, middle band, no signal) or the CH4 latch. A tick with no signal,
    or any value that is not low, starts the 0.2 s again. Thus one bounce
    of a loose lead (a low sample with the switch up) does not re-arm the
    robot or start FOLLOW_ME again. The VESC pack-low latch and the person
    stop hold also wait for the arm switch held low. The disarm on CH3 low
    still needs only one sample.
  - "No signal" is never high, never in the middle band, and never "switch
    OFF". Thus it cannot clear the re-arm latch, release the person stop
    hold, or clear the VESC pack-low latch.
  - Vision-hang restart gate (`pi_app/app/vision_hang.py`): no signal on
    CH3 blocks the restart for 30 s (`vision_hang_rc_off_s`), as a short RC
    dropout does. After 30 s of no signal without a break, it permits the
    restart ("disarmed, ch3 no signal for N s"), as a long RC silence does.
    A measured width starts the 30 s again. Without this, a parked robot
    with the transmitter off and a receiver that stops its outputs would
    block the restart for ever (the 2026-10-04 hang).
  - The journal gets one line when a channel loses its signal and one when
    the signal comes back. The JSON log gets the events `rc_no_signal` and
    `rc_signal_restored`. A channel gets at most one line in 5 s. A loose
    lead that changes faster gets one line with the number of changes.
    CH6 is not used, so it gets no journal line, only the JSON events.
- The Pi puts a monotonic time stamp on each line that it parses. An RC
  frame older than 1.0 s is stale (`RC_STALE_TIMEOUT_S` in
  `pi_app/control/controller.py`). A stale link is a gap in the serial
  lines (the USB cable, the Arduino), not a receiver "no signal". It
  disarms the robot and, since 2026-11-13, sets the re-arm latch (Kevin:
  "Latch it"). Before, the robot armed again by itself when the lines came
  back with the arm switch still up. Now it needs CH3 held low for 0.2 s,
  then held high. The phone cannot go around the latch: the controller
  sends neutral while it is disarmed, and only the RC arm switch arms it.
- Known residual (check 6): when the transmitter is switched off, the
  receiver's failsafe sends about 1000 µs on CH3 (stand test, 2026-11-11).
  To the Pi this is the same as the operator putting the arm switch OFF.
  The robot disarms, the low is held for more than 0.2 s, so the latch
  clears, and the robot arms again when the transmitter comes back with the
  arm switch up. This case is not latched.
- `python3 -m pi_app.cli.monitor_rc` shows `NOSIG` for a channel with no
  signal, and shows CH6 when the firmware sends it.

## 8. RTK rover to Pi (4-pin Gravity connector)

| Rover pin | Signal | Pi header | Confidence |
|---|---|---|---|
| + | Supply | Pin 2 or pin 4, 5 V | 5 V header supply is Confirmed (Kevin, 2026-11-13). The pin number is Unknown. |
| − | GND | A GND pin | The pin number is Unknown. |
| C | SCL | Pin 5, GPIO3, SCL1 | Confirmed: the driver uses bus 1. |
| D | SDA | Pin 3, GPIO2, SDA1 | Confirmed |

- Connector: Gravity 4-pin, PH2.0 (2.0 mm pitch), white housing with a
  latch. Pin order, Kevin, 2026-11-13: red = +, black = −, blue = SCL/RX
  (C), green = SDA/TX (D). That is `+ − C D`, the same order as the
  DFRobot Gravity standard. Photos of the cable end:
  `docs/data/shield_inputs_20261113/rover_cable_front.jpg` and
  `rover_cable_back.jpg`.
- Mode switch: I2C. Confirmed: the driver reads the rover at 0x20 on I2C.
- Supply range: 3.3 V to 5 V (DFRobot wiki). This answers the "VERIFY" in
  `docs/ups_hat_b.md`: the rover can operate from the Pi's 3.3 V pin. Nobody
  measured its current. `docs/power_module_spec.md` estimates 0.5 W.
  Research, 2026-11-13: DFRobot gives no current. The LC29H(DA) takes about 44 mA at
  3.3 V (retailer figure; Quectel's DA table was not read). The LLCC68
  LoRa radio takes about 4.2 mA in receive. The HM6245 is a linear LDO, so
  the 5 V current is about the 3.3 V current. Estimate: 60 mA to 90 mA at
  5 V, with short peaks below about 150 mA.
- Other devices on bus 1: the UPS HAT (B) INA219 at 0x42 on the robot, and
  the X1202 MAX17040 gauge at 0x36 on the bench. `i2c-bus-guard.service`
  unlocks a stuck bus (`docs/i2c_bus_guard.md`).
- The Pi 5 has fixed 1.8 kOhm pull-ups to 3.3 V on GPIO2 and GPIO3. If the
  rover board pulls SDA and SCL up to its own 5 V supply, the lines rest
  above 3.3 V. VERIFY (section 10, check 9).
- LoRa: 915 MHz. The base on the barn roof sends the corrections.

## 9. Safety findings

Status of findings 1, 5, 6 and 7 (2026-11-13): **fixed in code, not
deployed, not flashed.** The Pi change and the sketch change are on a
branch, not on `main`. Nothing in this section ran on the robot. The
deploy order is in section 9.1. Kevin's decisions of 2026-11-13 are in the
code: hold last good (100 ms), held low 0.2 s to clear a latch, CH4 loss
gives MANUAL armed, CH5 back high is the e-stop, the CRC, and the Model X
code removed. Later the same day: every FOLLOW_ME exit sets the CH4 latch,
and an RC-stale link sets the re-arm latch (section 7).

1. **A lost pulse keeps its last value.** The interrupt routines change a
   channel only on a pulse edge. Nothing times out a channel. The sketch
   still sends a line each 20 ms, and the Pi marks each line as fresh. Thus
   the Pi cannot see a channel that has no pulses. Causes: a loose or broken
   signal lead, a receiver without power, or a receiver that stops its
   outputs. Effects: a dead CH3 lead keeps the robot armed. A dead CH5 lead
   removes the e-stop, with no alarm. A dead CH1 or CH2 lead keeps that
   track at its last command, and a release of the stick does not stop it.
   Source: the code (`isrCH1` to `isrCH5` in the sketch,
   `ArduinoRCReader._read_loop`). Not tested on the robot (section 10,
   check 7). The agent review of 2026-11-13 agrees, and adds three cases. A
   loss in the middle of a pulse keeps a short width. A stray edge on a
   loose lead can give 1500 µs: CH3 then disarms after 0.5 s, but CH5
   cannot trigger the e-stop. A channel with no pulse since the last reset
   reads 1500 µs. For a frozen CH3 at 2000 µs, the only backstop is the
   600 s idle auto-disarm, and it needs neutral sticks.
   **Fix (code only, 2026-11-13):** the sketch sends `0` for a channel with
   no valid pulse for 100 ms, and for a channel with no valid pulse since
   the reset. The Pi disarms and latches on `0` on CH3 or CH5, holds both
   tracks neutral on `0` on CH1 or CH2, and stops FOLLOW_ME on CH4
   (section 7). A loss in the middle of a pulse now gives `0` after 100 ms:
   the pull-up holds the pin high, so the pulse never ends. One
   out-of-range pulse is ignored (Kevin's "hold last good"), so it does not
   disarm the robot. A stray edge can still give a wrong width inside
   500 µs to 2500 µs. A wrong low width cannot clear a latch, because the
   Pi needs a low held for 0.2 s (Kevin's "held low"). Tests:
   `tools/arduino_rc_reader_host_test/test_rc_channel.cpp`,
   `pi_app/tests/test_rc_no_signal_safety.py`. Open: bench check 7.
2. **Transmitter off is tested. A link loss with the transmitter on is not.**
   Stand test step 5, 2026-11-11 21:04: Kevin switched the transmitter off
   with the arm switch up. In the next 0.05 s, ch3 read 1004 µs and ch4 read
   1000 µs, and the robot disarmed (`logs/arm_20261111_210328.log` on the
   Pi). During the 3 s off, the journal shows ch3 at 1000 µs and ch4 at
   1004 µs. The data cannot show whether these values are a receiver
   failsafe preset or the last frame of the transmitter as it switches off.
   A link loss with the transmitter on (out of range, or a blocked antenna)
   has no test (section 10, check 6).
3. **The receiver failsafe covers one channel.** The manual (page 2) says
   that the "loss of control protection" protects "the throttle channel".
   The manual does not give the value of the other channels at a link loss.
   With the CH1/CH2 tank mix, the throttle channel is Unknown. To set the
   protection, hold OK while you switch the transmitter on.
4. **The transmitter switches off after 15 minutes without operation**
   (manual page 4). It beeps 1 minute before. By finding 2, this disarms the
   robot. Thus a FOLLOW_ME run with no stick or button input stops at
   15 minutes.
5. **An out-of-range pulse reads as a valid neutral.** The sketch changes
   a width outside 800 µs to 2200 µs to 1500 µs. The Pi accepts 1500 µs as
   valid, so its own "last valid value" rule never runs. A CH5 pulse above
   2200 µs thus hides the e-stop. A CH4 pulse below 800 µs reads 1500 µs, and
   FOLLOW_ME does not stop. The receiver can send 500 µs to 2500 µs. Today
   the logs stay in 992 µs to 2004 µs, at the default (maximum) travel of
   the transmitter. A change of the transmitter travel or of the receiver
   mode can cause this. Latent; source: agent review, sketch line 188.
   **Fix (code only, 2026-11-13):** the valid range is 500 µs to 2500 µs
   (S1, section 7). The sketch ignores a pulse outside it: never 1500 µs,
   and never a value. A stream of bad pulses gives `0` 100 ms after the
   last valid pulse. Each line has a CRC. The Pi reads `0` as "no signal",
   never as neutral, and drops a line with any other value outside the
   range or a bad CRC. A CH5 pulse from 1800 µs to 2500 µs now triggers the
   e-stop. A CH5 that sends only pulses above 2500 µs gives `0` after
   100 ms: the robot disarms and latches, without the power-off.
6. **No effect now: the Model X code has no motor-command timeout.** If the
   Pi stops sending, the sketch keeps the last Model X command on D7 to
   D12. The board is removed (section 5), so nothing moves. **Fix (code
   only, 2026-11-13):** the repo sketch has no Model X code (section 5).
7. **Low: `millis()` wraps after 49.7 days.** The CSV timing comparison
   then can stop the CSV. The Pi sees a stale stream and disarms, which is
   safe. **Fix (code only, 2026-11-13):** the repo sketch compares an
   unsigned difference, which stays correct across the wrap. The
   `micros()` wrap (71.6 minutes) is also handled: a channel that timed out
   stays at `0` until its next pulse.

### 9.1 Deploy order for the fixes of findings 1, 5 and 7

WARNING: Do the steps in this order. A 6-field line or a `0` from the new
firmware goes to a Pi that does not have the parser change. The old parser
drops each 6-field line, so the RC link goes stale after 1 s and the robot
disarms. That is safe, but the robot cannot be driven.

1. Deploy the Pi change first (requirement S12). It is backward compatible
   with the flashed firmware: the 5-field lines give the same values as
   before. A merge to `main` restarts `wall-e` on the robot within
   5 minutes, so do it with the robot disarmed and no cook on the smoker.
2. Check the Pi change with the flashed firmware: arm, drive, disarm, and
   use the e-stop switch as before. The journal must show no
   `RC CHn NO SIGNAL` line.
3. Build the sketch for the Nano with the Arduino toolchain
   (`arduino:avr:nano:cpu=atmega328old`). The Mac has no AVR toolchain;
   the Pi has one (section 10, check 2). Stop `wall-e` first, because it
   holds the Arduino port. Upload at 57600 baud. Keep the `dcc550e` image
   (section 10, check 2) as the fallback.
4. Run `python3 -m pi_app.cli.monitor_rc` with `wall-e` stopped. Expect 6
   fields, CH6 as `NOSIG` if no CH6 lead is connected, and the other
   channels as before.
5. Do bench checks 6 and 7 of section 10 on the stand, with `wall-e`
   stopped.
6. Start `wall-e`, with the robot on the stand and the wheels free. Arm,
   then disconnect the CH3 signal lead. Expect a disarm and the journal
   line `RC CH3 NO SIGNAL`. Connect the lead with the arm switch still up.
   Expect no arm until the switch goes OFF for at least 0.2 s and then ON.
   A lead that bounces while it connects gives short low samples; they do
   not clear the latch, because the Pi needs a low held for 0.2 s (Kevin,
   2026-11-13). A bounce that stays low for 0.2 s or more reads as a real
   switch-off. Do the same with the CH5 lead: expect a disarm and no
   power-off.
7. Connect and disconnect the CH4 lead in FOLLOW_ME. Expect FOLLOW_ME to
   stop, the robot to stay armed in MANUAL, and no new FOLLOW_ME start
   until the CH4 switch goes low for at least 0.2 s and then high. Do the
   same after a stop from the web page: a quick CH4 flick (less than
   0.2 s low) must not start FOLLOW_ME.
8. Arm, then pull the Arduino USB cable for 2 s with the arm switch up.
   Expect a disarm (`RC_STALE`). Connect the cable with the switch still up.
   Expect no arm until the switch goes OFF for at least 0.2 s and then ON.

## 10. Open checks

WARNING: Do checks 6 and 7 with the robot on the stand, wheels free, and
`wall-e` stopped. Then the motors cannot move.

1. **Arduino board.** Done 2026-11-13: a CH340 (`1a86:7523`) and an
   ATmega328P with the Nano old bootloader (section 3). Photographed on
   both sides (a CH340G with a 12 MHz crystal, an AMS1117 5 V regulator,
   and the "B2" diode at the USB end). Length: about 1.70 in (43.2 mm) on a
   dial caliper. The USB shell appears to touch the jaw, so this is
   probably the overall length with the USB socket. Photos in
   `docs/data/shield_inputs_20261113/`. Open: the diode-mode check of
   "B2".
2. **Firmware.** Done 2026-11-13 13:22, with `wall-e` stopped for about
   15 s (Kevin's approval). avrdude read the flash in no-write mode
   (`-n -U flash:r`) at 57600 baud. The first 4,806 bytes are the same, byte
   for byte, as a build of the repo sketch at `dcc550e` (2026-03-01) with
   the Debian toolchain on the Pi (`arduino-builder`, core 1.8.6, avr-gcc
   5.4.0, PinChangeInterrupt 1.2.9-9-g62e4ef1). The builds of `5f275d0` and
   `5aa12e3` do not match. Bytes 4,806 to 9,807 are left from an older,
   larger build of the same reader (they contain `RC Reader initialized`).
   The bootloader writes only the pages of the new program, so old pages
   stay. The program never runs them. The full image is saved as
   `docs/data/arduino_flash_20261113/flash_readback_20261113.hex` (MD5
   `56d5d39e94132cd8263f413f98879282`). To restore the program, upload the
   `dcc550e` build (the same bytes) at 57600 baud. Do not try to write the
   bootloader area of the image through the bootloader.
3. **Receiver label, rows and output mode.** Label done: "F-06A" (Kevin,
   2026-11-13), the V3 receiver. Howes Models: a V1 receiver has "DS600" on
   it, a V2 has "C-06A" (input 4 V to 12 V), and a V3 has "F-06A". V1, V2
   and V3 do not work with each other, and the transmitter is a V3. Open:
   the published F-06A has 7 header rows (6 channels and a power row), and
   Kevin reports no power row. Count the rows. Record the blue LED: off is
   the 50 Hz mode. Measured 2026-11-14 (Kevin's dial caliper; photos
   `docs/data/shield_inputs_20261113/receiver_*_dial_20261114.jpg`): the
   body is 1.30 x 0.785 x 0.40 in (33.0 x 19.9 x 10.2 mm). The antenna
   leaves one short end. The other end has a block of 18 FEMALE sockets
   (6 channels x 3) on the top face: the leads plug in from above. There is
   no power row. EXT-V is a separate 2-pin socket on the top face, next to
   the BIND button. Thus a lead to the receiver needs MALE pins at the
   receiver end. Research, 2026-11-13: the seventh row is EXT-V, a sense input
   (section 4). Listing size: 38 x 24 x 13 mm, 7.4 g. A V3 "voltage return"
   listing gives 32 x 19 x 13 mm. Both sizes fit the shield's receiver bay.
4. **Receiver power feed.** Answered by Kevin, 2026-11-13: the USB 5 V,
   on the Arduino side. Nothing connects to the header 5 V except the RTK
   rover. The pin on the Arduino side: 5V (Kevin, 2026-11-14).
5. **Signal voltage.** Measure the high level of the CH1 signal with a
   scope, transmitter on. A meter shows the average of the pulse, not its
   high level. Research found no published value (section 4).
6. **Link loss with the transmitter on.** Run
   `python3 -m pi_app.cli.monitor_rc`. Put the arm switch up. Hold CH1
   forward with a rubber band. Put the transmitter in a closed metal box
   (an unplugged microwave oven is sufficient) for 5 s. Record ch1 to ch5.
   Do it again with the stick released. With the new firmware, `NOSIG`
   means that the receiver stops its outputs at a link loss. Values mean
   that it sends failsafe positions: record them. Known residual: a
   failsafe CH3 of about 1000 µs looks to the Pi exactly like the operator
   switching off, so it clears the re-arm latch, and the robot can arm
   again when the link returns with the arm switch up (section 7).
7. **Lead loss.** Run `python3 -m pi_app.cli.monitor_rc`. Put the arm switch
   up. Disconnect the CH3 signal lead at the Arduino. Record ch3. With the
   flashed firmware (`dcc550e`), finding 1 expects ch3 to stay near
   2000 µs. With the new firmware (section 9.1), expect `NOSIG` about
   0.1 s after the disconnection.
8. **Model X.** Done. Kevin, 2026-11-13: the board is removed. The Arduino
   used to control it.
9. **Rover pin order and pull-ups.** Pin order done (Kevin, 2026-11-13):
   red +, black −, blue SCL/RX, green SDA/TX (section 8). Open: with the
   rover on 5 V and the Pi on, measure the SDA and SCL voltage at rest.
10. **Rover current.** Measure it at 5 V. The shield feeds the rover 5 V
    only (D14). The estimate from the parts is 60 mA to 90 mA (section 8),
    so the 0.3 A fuse has about 3 times the margin. A USB meter reading at
    the first power-up is sufficient.
11. **Header pins.** Record which 5 V pin and which GND pin the rover uses.
    The shield closes this check: 5 V on pins 2 and 4, and GND on pins 6, 9,
    14, 30, 34 and 39.

## 11. Inputs for the Pi 5 shield specification

Kevin, 2026-11-13: the target is a Pi 5 shield. This section gives the
requirements, the constraints and Kevin's decisions. It does not design the
shield.

Requirements:

- **S1. RC inputs.** Read 6 PWM channels. Accept 500 µs to 2500 µs (both
  F-06A modes). Accept 50 Hz and 333 Hz frames. Resolution: 4 µs or better.
  Report a pulse outside the valid range as invalid, not as 1500 µs
  (finding 5). Test at 333 Hz before you use that mode.
- **S2. No-pulse detection.** Report "no signal" for a channel that has no
  pulse for a set time (proposal: 100 ms, five 50 Hz frames; never less than
  60 ms, three 50 Hz frames). The Pi must treat "no signal" on CH3 or CH5 as
  a disarm. This closes finding 1.
- **S3. Signal levels.** The Pi 5 GPIO is 3.3 V and is not 5 V tolerant.
  The receiver signal level is Unknown. Each RC input must thus accept 3.3 V
  and 5 V signals, and each signal to the Pi must be 3.3 V. Changed by D13:
  no buffer. Each receiver signal goes through a 1 kOhm series resistor
  straight to its Nano pin, as today (no resistor today). This holds only
  while the Nano's 5 V stays on the Pi USB rail after its Schottky (about
  4.5 to 4.8 V, so VIH is about 2.8 V and a 3.3 V signal keeps about 0.5 V
  of margin); S10 already keeps the header 5 V off the Nano. Confirm with
  check 5 (scope the receiver high level). The 1 kOhm resistor does not
  decrease the margin: with the Nano's 20 kOhm to 50 kOhm pull-up, the pin
  is at most about 0.1 V above the receiver's high level.
- **S4. Receiver power.** Supply 5 V through a fuse, with a common ground.
  The receiver range is 4 V to 9 V DC, 12 V maximum. The F-06A listing
  gives 35 mA (research, 2026-11-13; not measured). The fuse is a Bourns
  MF-R030 (0.3 A hold, 0.6 A trip). An MF-R020 (0.2 A hold) is an option.
  No separate receiver power connector (Kevin, 2026-11-13): the 5 V and the
  ground reach the receiver on the + and − pins of the channel headers
  (S5), and one channel lead is enough, because the receiver connects its +
  rows together and its − rows together.
- **S5. Receiver connectors.** One 3-pin male servo header (2.54 mm pitch)
  for each of CH1 to CH6, so that standard servo leads connect. Pin order:
  signal, +, −, the same as the receiver (section 4). The + pin of each
  header is the fused receiver 5 V (S4), and the − pin is ground, as on a
  standard servo header. Thus any one lead powers the receiver. Mark the −
  pin on the silkscreen.
- **S6. RTK rover connector.** One Gravity PH2.0 4-pin socket on I2C bus 1
  (GPIO2, GPIO3). Supply: the 5 V of the Pi header, as today (Kevin,
  2026-11-13). That 5 V must go off when the Pi halts, so that the rover
  does not drain the cells in storage. The X1202 does this (0 V after a
  halt with the input present, measured 2026-11-13; the halt with the
  input off is not measured with a meter, `docs/pi_power_architecture.md`).
  Research, 2026-11-13:
  the Raspberry Pi bootloader page says that with POWER_OFF_ON_HALT=1 the
  5 V at the header stays on in a halt. Thus the supply board must cut it.
  Geekworm's X1202 manual says that the board cuts its output when the Pi
  shuts down, and that it starts again when the input comes back. That
  agrees with the 4 h soak on the cells. The fuse is a Bourns MF-R030
  (0.3 A hold, 0.6 A trip). The UPS HAT (B) does not.
  The pull-up voltage at the Pi pins must not be more
  than 3.3 V (check 9). Nothing on the shield may keep the Pi's 3.3 V rail
  up after a halt (refer to "Sneak feeds" in `docs/power_module_spec.md`).
- **S7. Pins and addresses that are in use.** Do not use GPIO 6 (X1202 power
  loss), GPIO 16 (X1202 charge control) or GPIO2 and GPIO3 for other
  functions. Do not use I2C addresses 0x20, 0x36 and 0x42 on bus 1. Do not
  use the pins of the Waveshare RS485 CAN HAT (S8): its MCP2515 is on SPI0
  (GPIO 8 CE0, GPIO 9, GPIO 10, GPIO 11) with its interrupt on GPIO 25
  (`dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25` in the Pi's
  `config.txt`, read 2026-11-13). Its RS485 side uses the UART (GPIO 14 and
  GPIO 15) and, on some versions, GPIO 4. Waveshare wiki (research,
  2026-11-13): RS485 is on GPIO 14 and GPIO 15. Its RSE direction pin is
  GPIO 4, and only a 0 Ohm resistor connects it; the factory default is
  automatic direction. The shield uses none of these pins. Superseded by
  D13: the footprints on GPIO 12, 13, 17 and 24 (S15) are removed.
- **S8. Mechanical.** Pi 5 HAT+ outline. Kevin, 2026-11-13: the shield
  mounts on top of the Waveshare RS485 CAN HAT, which is on the Pi's
  header. The CAN HAT passes the 40 pins up through its top, so the shield
  plugs straight onto them (Kevin, 2026-11-13). The X1202 is under the Pi
  on pogo pins. VERIFY the height of the CAN HAT's parts and terminal
  blocks, and where the NVMe carrier and the fan sit in the stack.
  Research, 2026-11-13: Waveshare publishes only the 65 x 30 mm outline and 3.0 mm
  holes. It does not give the part heights, the header type or a standoff
  length. Measure the HAT.
  Measured 2026-11-14 (Kevin's caliper, read on the beam scale, about
  +/- 0.5 mm; photos `docs/data/shield_inputs_20261113/*_20261114.jpg`).
  The stack, bottom to top: the X1202, the Pi 5 with its active cooler,
  the dual M.2 NVMe carrier on brass standoffs (PCIe ribbon under it, SSD
  in slot 1, slot 2 empty), then the CAN HAT on a stacking header. The
  CAN HAT's pins stand about 8 mm above its top face (a standard stacking
  header). Dial readings (the dial is 1 in per turn, 0.01 in per mark):
  the terminal block top is 0.355 in (9.0 mm) above the CAN HAT's top
  face, about 1 mm more than the pins; Pi top face to CAN HAT top face is
  1.29 in (32.8 mm). Thus the terminal block, not J1 (8.5 mm body), sets
  the gap: the shield's underside rests on the block, 9.0 mm above the
  CAN HAT, and the pins engage about 7.5 mm in J1. The shield's underside
  is about 21.5 mm above the NVMe carrier (photo estimate: carrier top
  face to CAN HAT top face about 12.5 mm) and about 41.8 mm above the Pi.
  Standoff lengths do not change the copper; confirm them before buying. The south standoffs (H3, H4) land on
  the carrier, not on the Pi. The SSD in slot 1 appears to cover one of
  the carrier's two holes on that side. Kevin, 2026-11-14: move the SSD to
  slot 2 (under the CAN HAT, about 6 mm clear), so both south carrier
  holes take H3 and H4 (an agent checks that the SSD boots from slot 2).
  If the pins bottom in J1 before the shield reaches the block, the pins
  set the gap instead (the PCB session checks J1's insertion depth).
- **S9. USB link.** The 40-pin header has no USB lines. Kevin's short cable
  has a 90° Mini-B plug: the cable leaves at a right angle to the plug.
  Kevin, 2026-11-13: mount the Nano so that its own Mini-B port sticks out
  of the side of the case, as it does today, and the existing cable plugs
  straight into the Nano. The Nano's own USB port is the only USB
  connection on the shield (D6). Kevin, 2026-11-13, later: put the Nano on
  the side of the Pi's USB ports (the short end with the USB-A ports and
  Ethernet), so the cable to a Pi USB port is short. This replaces the
  earlier south-edge (USB-C and HDMI side) position. Kevin's photo of the
  plug in the current Nano:
  `docs/data/shield_inputs_20261113/nano_mount_and_usb_plug.jpg`.
- **S10. Two 5 V rails that do not connect.** The microcontroller and the
  receiver get their power from the USB VBUS only, as the Arduino and the
  receiver do today: a USB port of the Pi supplies them (Kevin,
  2026-11-13). The RTK rover gets its power from the header 5 V only (S6).
  On the shield the two rails share ground and nothing else. Today both
  come from the Pi's 5 V rail, but the VBUS side goes through the Pi's USB
  current limit. A shield that joins them goes around that limit. If the
  cable ever goes to a powered hub again, it also feeds 5 V back into the
  Pi: on 2026-11-04 the hub fed 5 V in both directions through its cable,
  and the Pi carried the hub, the OAK and the Arduino
  (`docs/ups_hat_b.md`).
- **S11. One microcontroller: an off-the-shelf Arduino Nano in a socket.**
  Kevin, 2026-11-13: "If we're still going to have an Arduino, might as
  well just buy one off the shelf and make this board a whole lot simpler.
  Let's remove the small chip to replace the Arduino." The socket is two
  1x15 female headers, 2.54 mm pitch, 15.24 mm (0.6 in) apart
  (`docs/arduino_nano_pinout.svg`). Pin map: CH1 to CH5 on D2 to D6, CH6 on
  A0, so the existing sketch runs unchanged. Buy a Nano with straight
  headers on both rows: the robot's present Nano has one right-angle row
  and does not fit the socket as it is. The robot's Nano measures about
  43.2 mm overall (section 10, check 1). Superseded: the on-board
  ATmega328P + CH340C position of the first design (old D6).
- **S12. The Pi changes first.** The Pi parser must accept 6 fields and the
  "no signal" value before a firmware sends them (section 7). Kevin,
  2026-11-13: the code can come later, but the CH6 wire must exist now.
  Status, 2026-11-13: the Pi change, the sketch change (S1 range, S2
  timeout, CH6) and their tests are written, on a branch. Not deployed,
  not flashed. Deploy order: section 9.1.
- **S13. Receiver mount, if there is room.** Kevin, 2026-11-13: "It'd be
  nice to have the real estate to put the receiver on the board. Leave me
  some screw holes to 3D print a clip to attach it if we have the room."
  Reserve a clear area on the top of the shield for the F-06A, next to the
  servo headers J11 to J16 so the six leads stay short, with its pin
  header facing them. Size it for the larger published size, 38 x 24 x
  13 mm (another listing gives 32 x 19 x 13 mm; Kevin can measure). Put no
  parts taller than about 1 mm under it. Give 2 to 4 screw holes (M2.5 or
  M3) around it for a printed clip, with the hole positions in the README
  for the clip design. Leave a path for the receiver's antenna lead to
  reach the board edge, away from the USB and the crystal of the Nano. If
  it does not fit the HAT outline, say so and give the space that is
  free. Radio (Grok, 2026-11-13): a 2.4 GHz receiver on top of the Pi, the
  UPS and the motor controllers is a poor radio site, and RC is the safety
  link. Kevin: keep the mount, with the antenna lead routed out of the
  stack; if the range is poor, the receiver moves outside on longer leads
  (the servo headers allow it). Do an RC range test before relying on it.
  As built (board README): the bay is north-east of the servo headers, and
  the leads are about 10 mm to 30 mm long with a turn. Clip holes: 45.0 x
  19.0 mm, M2.5. Tie holes: 2.2 mm.
- **S14. Spare Nano pin header (fitted in v1).** Break out the spare Nano
  pins to a header for later inputs (for example bump switches): A1 to
  A5, D7 to D12, 5 V (USB side) and GND. Label D7 to D12 "outputs in
  today's firmware" until the Model X code is removed. Status, 2026-11-13:
  removed from the repo sketch, not flashed. The label stays correct until
  the Nano gets the new sketch.
- **S15. REMOVED from v1 (D13).** Kept for the record. Footprints only, no
  parts fitted (Kevin, 2026-11-13). Copper
  and silkscreen, so the parts and the code can come later:
  1. Charger-present input: an opto from the charge inlet to a free Pi
     GPIO (the research proposes GPIO 24), with a 2-pin pack-side
     connector. It works only if the inlet can be told apart from the pack
     bus (open fact: how the inlet is wired). This is a live input, not
     the rejected charger latch or "charger unplugged" button.
  2. Isolated wired BMS link: a 2-channel isolator between the Pi's UART4
     (GPIO 12 and 13) and the BMS UART port, plus an opto on the BMS's S1
     "activate" pin, and a 6-pin connector. The BMS UART ground is battery
     B−, not the BMS output, so the link must be isolated. Fit it only
     after a wake test shows that UART traffic or S1 wakes a sleeping BMS.
     Read-only: the same polls as today; BMS writes stay in the Daly app.
  3. Rover 5 V switch: a P-FET from a free Pi GPIO (the research proposes
     GPIO 17), on by default, so the Pi can power-cycle the RTK rover
     after an I2C bus lock-up.
- **S16. REMOVED from v1 (D13).** Kept for the record. VESC kill line,
  footprint only, marked EXPERIMENTAL. A
  Nano-driven stop input into each VESC (PPM or ADC2, VESC firmware 6.05),
  so the Nano can stop the motors without the Pi. Rules (Grok review):
  one opto per VESC, never one shared driver (a single tripped track
  pivots the robot); the Nano pin released means kill (fail-safe); a
  series resistor and a jumper in each LED path, default open; the
  phototransistor referred to that VESC's own ground. It is not counted as
  a safety function until a bench test shows that on firmware 6.05 the
  kill input wins over CAN current and duty commands. Price in before
  fitting: each Nano reset or upload releases the line and drops drive; a
  frozen Nano that holds the pin OK does not kill; a kill coasts unless a
  modest timeout brake current is set, and a long regenerative brake can
  open the BMS charge FET.
- **S17. Stay off the CAN HAT's pins (layout rule).** No copper on the
  shield connects to SPI0 (GPIO 8 to 11) or GPIO 25, and nothing on the
  shield can load them. A solder bridge or a clamped interrupt line there
  takes the only drive path down even though the shield has no CAN parts.
  The HAT's RS485 driver uses UART0 (GPIO 14 and 15), so the BMS link in
  S15 uses UART4, not UART0.

Research on extra roles (Opus agent and Grok, 2026-11-13), in short:
- CAN: keep the HAT (D10). The Nano as a USB-to-CAN bridge is rejected:
  about 360 frames per second (2 VESCs, 3 status types at about 50 Hz,
  plus 60 commands) would take about 70 percent of the 115200 baud link
  in ASCII slcan, the MCP2515 has only 2 receive buffers, CAN interrupts
  would skew the RC pulse timing, and arm, e-stop and drive would share
  one chip and one USB cable. Moving the HAT's MCP2515 and transceiver
  onto a later board is possible with the same overlay; then use a
  ready-made 3.3 V module on pads rather than a hand-laid crystal and PHY.
- Watchdog: no Nano watchdog. No recorded stall was a hung Pi; each was a
  peripheral or a thread with its own software fix. A hung Pi already
  stops the motors (VESC timeout and the RC stale timeout, about 1 s each).
  The Pi 5's own hardware watchdog (15 s) exists but is off
  (`RuntimeWatchdogSec` 0); turning it on is a separate config decision.
- BMS: the dark periods match the BMS's own sleep (default 3600 s with no
  current; it wakes on current). The BMS is a JHB-R24TH-V1.3
  (`docs/battery_settings.md`), not the SPIM08HP that `config.py` and
  CLAUDE.md name. Setting its sleep to 65535 in the Daly app would stop
  the dark periods at about 25 mA of standby drain instead of about
  0.5 mA. Kevin, 2026-11-13: not yet.
- Not taken: a status LED and buzzer (Kevin did not choose it), pack
  voltage sense and a current shunt (the VESCs and the BMS already report
  them), and the F-06A EXT-V voltage return (its input range is not
  published, and it would bring pack voltage onto the RC side).

Second opinion on the first S11, the on-board chip, now removed (Grok,
session `6f2eb58a-e096-4a22-a90d-751115f951fb`, 2026-11-13):

- It agrees with the AVR family and the existing sketch.
- It objects to a bare chip: a bare ATmega328P needs a crystal, a reset
  circuit and a bootloader load through ICSP, and "a CH340" is three
  different circuits. A first-time builder can damage the only working
  board. The Nano socket answers this objection: the known-good Nano stays
  the fallback, and its flash image is saved (section 10, check 2).
- Rejected: its claim that 115200 baud fills up at 333 Hz. The sketch sends
  at a fixed 50 Hz, not at the receiver frame rate. With 6 fields that is
  about 1.8 kB/s, about 16 percent of the link.
- It recommends an RP2040 only as a module (a Pico), and only after the
  protocol and the no-pulse rule work on the Nano.
- Its order of work, accepted: (1) measure the receiver (checks 5 and 6);
  (2) change the Pi parser (S12); (3) add the no-pulse rule and CH6 to the
  sketch on the current Nano; (4) build the shield.

Decisions (Kevin, 2026-11-13):

1. **D1. A microcontroller on the shield reads the RC channels**, as the
   Arduino does today. The Pi does not read the PWM directly.
2. **D3. The CAN interface stays separate** for now.
3. **D4. The microcontroller connects to the Pi by USB.**
4. **D2. The shield has no Model X outputs.** The Model X is a hobby motor
   driver board from the first prototype. The Arduino used to control it,
   and the board is removed (section 5). The VESCs on CAN drive the robot.
5. **D5. The shield has no HAT+ ID EEPROM.** The ID EEPROM is a small
   memory chip on GPIO 0 and GPIO 1 (header pins 27 and 28). The Pi reads
   it at boot to identify the board and to load its settings. The Raspberry
   Pi HAT+ specification requires it only for a board that uses the HAT+
   name. The robot's own code sets up each device, so the EEPROM gives
   nothing here. If the NVMe carrier has an ID EEPROM, a second one at the
   same address on the same pins would conflict. Do not use GPIO 0 and
   GPIO 1 on the shield.
6. **D6. Only a Nano socket; no on-board microcontroller** (S11). Changed
   on 2026-11-13: the first design had both an on-board ATmega328P +
   CH340C and the socket. Kevin removed the on-board chip to make the board
   simpler, and buys an off-the-shelf Nano.
7. **D7. CH6 is wired now:** to A0 on the current Arduino, and to A0 of the
   Nano socket on the shield. The code reads it later.
8. **D8. Hand-soldered build,** through-hole where a part allows it, like
   the other boards in `~/Documents/KiCad/pcb-lessons`. Each part is
   through-hole, except Q1 and Q2 (BSS138, SOT-23).
9. **D9. The shield mounts on the Waveshare RS485 CAN HAT** (S8), which
   passes the pins up. The Nano sits above the Pi's Ethernet jack, next to
   the USB-A ports,
   and its own USB port takes Kevin's 90° Mini-B cable (S9).
10. **D10. Keep the Waveshare CAN HAT for v1** (S17). Revisit on a later
    board with a ready-made MCP2515 module.
11. **D11. v1 extras:** the spare Nano pin header is fitted (S14); the
    charger-present input, the isolated BMS link, the rover 5 V switch
    (S15) and the experimental VESC kill line (S16) are footprints only.
    No status LED or buzzer.
12. **D12. The receiver mount stays,** with the antenna out of the stack;
    the fallback is the receiver outside on longer leads (S13).
13. **D13. Trimmed v1 (Kevin, 2026-11-13: "I think I got greedy").** The
    board matches today's proven wiring and the verified firmware
    (`dcc550e`: CH1 to CH5 on D2 to D6, `INPUT_PULLUP`), plus CH6 on A0.
    Removed:
    - the 74HCT541 buffer, its socket, its capacitor and the bypass links
      (Grok, session `0619778f-1c3f-4ea7-80de-831a01a7bfde`: drop it while
      the Nano stays on the Schottky rail; it ran below its own 4.5 V
      minimum and was what clashed with the CAN HAT's terminal block);
    - the 100 kOhm pull-downs (they would fight the sketch's
      `INPUT_PULLUP` once wired straight to the Nano);
    - the BSS138 I2C level shifter and its pull-up positions: the rover
      goes straight onto the Pi's I2C bus 1, as today;
    - the cut jumpers on the RC channels (the pin map is checked against
      the firmware and the Nano's silkscreen);
    - all of S15 (charger sense, wired BMS link, rover power switch) and
      S16 (VESC kill line). The wired BMS link is held for later, possibly
      over I2C.
    Kept: the Pi header socket, the Nano socket, six servo headers with a
    1 kOhm series resistor per signal, the rover connector, the two PTC
    fuses, the spare Nano pin header (S14), the receiver bay (S13), and
    the S17 layout rule (no copper on SPI0 or GPIO 25, and keep
    through-hole tails off the CAN HAT's terminal block and 4-pin header).
    Added: a 3-pad solder jumper that selects the rover's supply, 5 V by
    default or the Pi's 3.3 V, in case check 9 shows the rover pulls I2C
    up to 5 V. The rover is rated 3.3 to 5 V. (Changed by D14.)
14. **D14. Rover on 5 V with the level shifter; no JP2; no I2C cut
    jumpers (Kevin, 2026-11-13).** Facts from DFRobot's TEL0171 rover
    schematic (V1.0, 2024-05-09,
    `img.dfrobot.com.cn/wiki/5ea64bf6cf1d8c7738ad2881/87f50e19ace99aae86a93d8d35a0f130.pdf`):
    - The Gravity port P3 is pin 1 SDA/TX, pin 2 SCL/RX, pin 3 GND, pin 4
      VCC. This agrees with Kevin's cable (red +, black −, blue SCL, green
      SDA), with the JST pin numbering, and with J2 (pad 1 SDA, pad 2 SCL,
      pad 3 GND, pad 4 supply). Check 9's pin-order half is closed.
    - The port has its own 2N7002 level shifters (Q4, Q5). On the
      connector side, R16 and R17 (10 kOhm) pull SDA and SCL up to the
      rover's VCC. VCC also feeds an HM6245 3.3 V regulator for the
      Quectel LC29H, whose supply range is 3.1 to 3.6 V.
    Consequences:
    - The rover stays on 5 V. At 3.3 V in, the regulator leaves the
      LC29H at or below its 3.1 V minimum, so the 3.3 V position of JP2
      is not a usable fallback. JP2 is removed, and the rover supply is
      the header 5 V through F3.
    - On 5 V without a shifter, as on the robot today, the Pi's GPIO2 and
      GPIO3 rest at about 3.56 V (10 kOhm to 5 V against the Pi's
      1.8 kOhm to 3.3 V). That is above the Pi's 3.3 V rating all the
      time. A rover that has 5 V while the Pi's 3.3 V rail is off also
      feeds about 0.5 mA per line into that rail. Grok (session
      `4c54ea57-0a48-487f-aff6-09658bef08de`) agreed, and said the old
      bus lock-up is not explained by this.
    - So the two-BSS138 shifter (NXP AN10441 topology, gates on the Pi's
      3.3 V, the Pi on the low side) goes back, as audited in the first
      design. No extra high-side pull-ups: the rover's 10 kOhm to 5 V is
      the high side. At the Pi's default 100 kHz the edges are fine.
    - The two I2C cut jumpers are removed: the pinout is confirmed four
      ways above.
    - Residual (electrical audit item 4, refined by the Grok shifter
      review, session `17a6b264-ec2a-4bc5-be4a-21772be526d7`): if F3
      trips, the rover's rail falls. If that dead rail is a low impedance
      to ground, the shifter conducts and the rover's pull-ups hold I2C
      bus 1 low, which the UPS gauge shares. If the dead rail floats, the
      bus stays high at about 2.9 V and the rover browns out. Fault case
      only.
    - Built as `d5f5a7f` in `gruMoses/hardware-projects`: ERC 0, DRC 0,
      the shifter as audited, J2 pad 4 on the header 5 V through F3.

## Appendix: Technical Names and Technical Verbs

**Technical Names:** WALL-E, Kevin, Pi, Pi 5, Raspberry Pi, HAT, HAT+,
shield, X1202, UPS HAT (B), NVMe, PCIe, HotRC, DS600, F-06A, Havcybin,
Arduino, Nano, Uno, ATmega328P, CH340, OSOYOO, Model X, H-bridge, VESC, CAN,
`can0`, DFRobot, KIT0198, Gravity, PH2.0, Quectel, LC29HDA, LoRa, RTK, GPS,
I2C, SDA, SCL, GPIO, GPIO2, GPIO3, GPIO 6, GPIO 16, INA219, MAX17040, USB,
USB 2, `/dev/ttyUSB0`, `/dev/ttyACM*`, Genesys Logic, OAK-D Lite, PWM, FHSS,
LCD, LED, BIND, EXT-V, CH1 to CH6, D2 to D12, ENA, ENB, IN1 to IN4,
`INPUT_PULLUP`, `micros()`, pin-change interrupt, CSV, 8N1, baud, µs, kOhm,
VCC, MCU, deadband, failsafe, e-stop, FOLLOW_ME, MANUAL, `wall-e`,
`monitor_rc`, `lsusb`, `i2c-bus-guard.service`, `RC_STALE_TIMEOUT_S`,
`ArduinoRCReader`, journal, silkscreen, servo header, stacking header, pogo
pin, pull-up, sketch, firmware, stand, rubber band, microwave oven, ID
EEPROM, GPIO 0, GPIO 1, VBUS, USB hub, OAK, prototype, A0, avrdude,
`arduino-builder`, avr-gcc, FQBN, bootloader, ICSP, DTR, CH340C, RP2040,
Pico, Grok, flash image, MD5, `millis()`, Howes Models, C-06A, servo lead,
pin header, female header, DIP-30, continuity, meter, power row,
`RC_NO_SIGNAL_US`, `RC_NO_SIGNAL_DISARM`, `rc_no_signal`,
`rc_signal_restored`, `NOSIG`, `rc_channel.h`, `rc_signal.py`, no signal,
debounce, re-arm latch, person stop hold, vision-hang restart, pack-low
latch, AVR, toolchain, branch, `main`, `dcc550e`, `5f275d0`, CRC, CRC-8,
hexadecimal, polynomial, hold last good, held low,
`latch_clear_low_persist_s`, `vision_hang_rc_off_s`, `ArduinoModelXDriver`,
control tick, slew limiter.

**Technical Verbs:** arm, disarm, enumerate, latch, mix, parse, power off,
stack, deploy, flash, upload.
