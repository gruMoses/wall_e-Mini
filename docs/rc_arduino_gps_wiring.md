# RC receiver, Arduino and RTK GPS wiring

Status: draft, 2026-11-13. This document records the wiring as built on the
robot before the X1202 bench work. It is the input for a Pi 5 shield
specification (section 11). It does not design the shield.

WARNING: Nobody measured the robot for this document. The robot's Pi is on
the X1202 bench, and the Arduino is not connected to it (`lsusb` on
2026-11-13 12:52 shows only the root hubs). The sources are the code, the
logs, the HotRC DS600 manual and the DFRobot wiki. Section 10 lists each open
check.

## 1. Confidence levels

- **Confirmed:** the code reads the part, a log shows the behaviour, or the
  maker's manual gives the value.
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
| Receiver | HotRC F-06A: 6 channels, PWM, 4 V to 9 V DC | Inferred. Page 6 of the DS600 manual names the F-06A. Nobody read the label on the robot. |
| RC bridge MCU | ATmega328P board, probably an Arduino Nano clone with a CH340 USB serial chip | Inferred. The sketch needs an ATmega328P. The board enumerates as `/dev/ttyUSB0`, which is a USB serial bridge, not the native USB of an Uno R3. `android_bt_protocol.txt` calls it a Nano. |
| Fallback motor driver | OSOYOO Model X H-bridge | The code is Confirmed (`pi_app/hardware/arduino_modelx.py`). Whether the board is fitted is Unknown. |
| USB hub | Genesys Logic, USB IDs 05e3:0610 and 05e3:0626, own supply from the main pack | Confirmed (`docs/ups_hat_b.md`) |
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
| CH6 signal | none | Not used | Inferred: the sketch reads 5 channels. |
| + and − | Unknown | Receiver power | Section 6 |

Electrical facts:

- The F-06A has two output modes (manual page 6). Blue LED off: analogue
  servo mode, 50 Hz, 1000 µs to 2500 µs. Blue LED on: digital servo mode,
  333 Hz, 500 µs to 2500 µs. A 3 s press of BIND changes the mode. The logs
  show 992 µs to 2004 µs. The mode on the robot is Unknown.
- The sketch measures each pulse with `micros()` (4 µs steps) in a
  pin-change interrupt. It changes a width outside 800 µs to 2200 µs to
  1500 µs before it sends the line.
- D2 to D6 use `INPUT_PULLUP` (internal pull-up of 20 kOhm to 50 kOhm to
  5 V).
- The manual does not give the signal voltage. An ATmega328P at 5 V reads
  a high level at 3.0 V or more (0.6 × VCC). A 3.3 V output thus has 0.3 V
  of margin. VERIFY (section 10, check 5).

## 5. Arduino to Model X (fallback)

| Arduino pin | Model X pin | Use |
|---|---|---|
| D10 | ENB | Left PWM |
| D7 | IN3 | Left direction 1 |
| D8 | IN4 | Left direction 2 |
| D9 | ENA | Right PWM |
| D12 | IN1 | Right direction 1 |
| D11 | IN2 | Right direction 2 |

The Model X is a hobby motor driver board from the first prototype. Its code
is in the initial commit (2025-08-15), before the VESCs. The VESCs on CAN are
the drive path. The Pi uses the Model X path only when `can0` does not exist
(`pi_app/app/main.py`, "VESC not detected"). A comment in the sketch says that the IN1/IN2
polarity "may need swap depending on wiring". Nobody recorded a check of the
right-side direction.

## 6. Power and ground

- The Arduino gets 5 V from port 2 of the USB hub. The hub has its own
  supply from the main pack. The Pi-to-hub cable has its 5 V wire cut
  (2026-11-05). Thus the Pi does not supply the hub, and the hub does not
  supply the Pi.
- The receiver power feed is Unknown. The F-06A takes 4 V to 9 V DC on its
  power row (manual page 5). The manual note gives 12 V as the maximum. An
  October note in `docs/NEXT_SESSION.md` assumed the Arduino 5 V pin. Nobody
  checked it.
- The receiver and the Arduino must have a common ground for the PWM
  signals. The USB ground connects the Arduino ground to the Pi ground.
- The RTK rover gets 5 V from the Pi header (section 8).

## 7. Arduino to Pi (USB serial)

- Port: `/dev/ttyUSB0`. The reader tries `/dev/ttyUSB*` first, then
  `/dev/ttyACM*`.
- Rate: 115200 baud. 8N1 is Inferred (the Arduino default).
- Arduino to Pi: one line each 20 ms, `ch1,ch2,ch3,ch4,ch5`, in µs. After a
  reset, the sketch waits 1 s and sends `RC Reader initialized`.
- Pi to Arduino: `M,<left>,<right>`. The bytes are 0 to 254, neutral 126.
  The sketch limits them to 0 to 255.
- The Pi puts a monotonic time stamp on each line that it parses. An RC
  frame older than 1.0 s is stale (`RC_STALE_TIMEOUT_S` in
  `pi_app/control/controller.py`).

## 8. RTK rover to Pi (4-pin Gravity connector)

| Rover pin | Signal | Pi header | Confidence |
|---|---|---|---|
| + | Supply | Pin 2 or pin 4, 5 V | 5 V header supply is Confirmed. The pin number is Unknown. |
| − | GND | A GND pin | The pin number is Unknown. |
| C | SCL | Pin 5, GPIO3, SCL1 | Confirmed: the driver uses bus 1. |
| D | SDA | Pin 3, GPIO2, SDA1 | Confirmed |

- Connector: Gravity 4-pin, PH2.0 (2.0 mm pitch). The kit has a PH2.0-4P
  cable (Inferred from the kit listing). The wiki does not give the pin
  order. Read the order on the board silkscreen.
- Mode switch: I2C. Confirmed: the driver reads the rover at 0x20 on I2C.
- Supply range: 3.3 V to 5 V (DFRobot wiki). This answers the "VERIFY" in
  `docs/ups_hat_b.md`: the rover can operate from the Pi's 3.3 V pin. Nobody
  measured its current. `docs/power_module_spec.md` estimates 0.5 W.
- Other devices on bus 1: the UPS HAT (B) INA219 at 0x42 on the robot, and
  the X1202 MAX17040 gauge at 0x36 on the bench. `i2c-bus-guard.service`
  unlocks a stuck bus (`docs/i2c_bus_guard.md`).
- The Pi 5 has fixed 1.8 kOhm pull-ups to 3.3 V on GPIO2 and GPIO3. If the
  rover board pulls SDA and SCL up to its own 5 V supply, the lines rest
  above 3.3 V. VERIFY (section 10, check 9).
- LoRa: 915 MHz. The base on the barn roof sends the corrections.

## 9. Safety findings

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
   check 7).
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

## 10. Open checks

WARNING: Do checks 6 and 7 with the robot on the stand, wheels free, and
`wall-e` stopped. Then the motors cannot move.

1. **Arduino board.** Connect the Arduino to a Pi and run `lsusb`. A CH340
   shows as `1a86:7523`. Photograph the board, both sides.
2. **Firmware.** Compare the firmware on the Arduino with
   `arduino_rc_reader/arduino_rc_reader.ino`. An agent started this check on
   2026-11-13. Its result goes here.
3. **Receiver label and output mode.** Read the label. Record the blue LED:
   off is the 50 Hz mode.
4. **Receiver power feed.** Follow the + and − leads of the receiver.
   Record where they go.
5. **Signal voltage.** Measure the CH1 signal pin with a meter or a scope,
   transmitter on.
6. **Link loss with the transmitter on.** Run
   `python3 -m pi_app.cli.monitor_rc`. Put the arm switch up. Hold CH1
   forward with a rubber band. Put the transmitter in a closed metal box
   (an unplugged microwave oven is sufficient) for 5 s. Record ch1 to ch5.
   Do it again with the stick released.
7. **Lead loss.** Run `python3 -m pi_app.cli.monitor_rc`. Put the arm switch
   up. Disconnect the CH3 signal lead at the Arduino. Record ch3. Finding 1
   expects ch3 to stay near 2000 µs.
8. **Model X.** Record whether the board is fitted. Kevin did not know the
   board on 2026-11-13, so it is probably not fitted.
9. **Rover pin order and pull-ups.** Read the silkscreen. With the rover on
   5 V and the Pi on, measure the SDA and SCL voltage at rest.
10. **Rover current.** Measure it at 5 V and at 3.3 V.
11. **Header pins.** Record which 5 V pin and which GND pin the rover uses.

## 11. Inputs for the Pi 5 shield specification

Kevin, 2026-11-13: the target is a Pi 5 shield. This section gives the
requirements, the constraints and the open decisions. It does not design the
shield.

Requirements:

- **S1. RC inputs.** Read 6 PWM channels. Accept 500 µs to 2500 µs (both
  F-06A modes). Accept 50 Hz and 333 Hz frames. Resolution: 4 µs or better.
- **S2. No-pulse detection.** Report "no signal" for a channel that has no
  pulse for a set time (proposal: 100 ms, five 50 Hz frames). The Pi must
  treat "no signal" on CH3 or CH5 as a disarm. This closes finding 1.
- **S3. Signal levels.** The Pi 5 GPIO is 3.3 V and is not 5 V tolerant.
  The receiver signal level is Unknown. Each RC input must thus accept 3.3 V
  and 5 V signals, and each signal to the Pi must be 3.3 V.
- **S4. Receiver power.** Supply 5 V through a fuse, with a common ground.
  The receiver range is 4 V to 9 V DC, 12 V maximum. The current is Unknown.
- **S5. Receiver connectors.** One 3-pin servo header (S, +, −; 2.54 mm
  pitch) for each of CH1 to CH6, so that the existing receiver leads
  connect.
- **S6. RTK rover connector.** One Gravity PH2.0 4-pin socket on I2C bus 1
  (GPIO2, GPIO3). The rover supply goes off when the Pi halts: use the Pi's
  3.3 V, or a 5 V that the Pi's 3.3 V switches. The pull-up voltage at the
  Pi pins must not be more than 3.3 V. Nothing on the shield may keep the
  Pi's 3.3 V rail up after a halt (refer to "Sneak feeds" in
  `docs/power_module_spec.md`).
- **S7. Pins and addresses that are in use.** Do not use GPIO 6 (X1202 power
  loss), GPIO 16 (X1202 charge control) or GPIO2 and GPIO3 for other
  functions. Do not use I2C addresses 0x20, 0x36 and 0x42 on bus 1.
- **S8. Mechanical.** Pi 5 HAT+ outline. The X1202 is under the Pi on pogo
  pins, and the NVMe carrier is on top. The shield must stack above the
  NVMe carrier on a stacking header and keep the PCIe ribbon and the fan
  clear. VERIFY the carrier model and its height.

- **S9. USB link.** The 40-pin header has no USB lines. The shield needs a
  USB connector and a short cable to a USB port of the Pi (D4).
- **S10. One 5 V source.** Supply the microcontroller and the receiver from
  one 5 V source. Do not connect the VBUS of the USB cable to that 5 V.
  Reason: on 2026-11-04 the USB hub fed 5 V in both directions through its
  cable, and the Pi carried the hub, the OAK and the Arduino
  (`docs/ups_hat_b.md`).

Decisions (Kevin, 2026-11-13):

1. **D1. A microcontroller on the shield reads the RC channels**, as the
   Arduino does today. The Pi does not read the PWM directly.
2. **D3. The CAN interface stays separate** for now.
3. **D4. The microcontroller connects to the Pi by USB.**

Open decisions:

1. **D2. Model X outputs.** Recommendation: remove them. The Model X is a
   hobby motor driver board from the first prototype. Its code is in the
   initial commit (2025-08-15), before the VESCs. Kevin does not know the
   board, so it is probably not fitted (check 8).
2. **D5. HAT+ ID EEPROM.** Recommendation: do not fit one. The ID EEPROM
   is a small memory chip on GPIO 0 and GPIO 1 (header pins 27 and 28).
   The Pi reads it at boot to identify the board and to load its settings.
   The Raspberry Pi HAT+ specification requires it only for a board that
   uses the HAT+ name. The robot's own code sets up each device, so the
   EEPROM gives nothing here. If the NVMe carrier has an ID EEPROM, a
   second one at the same address on the same pins conflicts (VERIFY the
   carrier).

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
EEPROM, GPIO 0, GPIO 1, VBUS, USB hub, OAK, prototype.

**Technical Verbs:** arm, disarm, enumerate, latch, mix, parse, power off,
stack.
