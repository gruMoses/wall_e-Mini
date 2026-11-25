# Charger plug: holding the main pack near 50 %

A TP-Link Kasa EP10 Wi-Fi plug ("Wally") powers WALL-E's charger (XM2703,
54.6 V 4 A). The robot switches it, so the pack does not sit at 100 % between
drives, an unhealthy pack is not charged, and a growing cell imbalance is
flagged. Set up 2026-11-16. The measurements behind every number are in
`docs/charger_plug_research_2026-11-16.md`.

## Parts
- `pi_app/hardware/kasa_plug.py`: the plug driver. Original Kasa protocol
  (TCP 9999, XOR cipher), plain sockets, no library, no cloud. The plug is
  known by its MAC; when the address stops answering, the driver finds the
  MAC with a UDP broadcast. No DHCP reservation is necessary.
- `pi_app/control/charge_policy.py`: the decision, pure and tested.
- `pi_app/control/charge_manager.py`: a thread. Every 10 s it reads the BMS,
  the VESC pack voltage, the armed state and the plug, runs the policy and
  switches the plug. It keeps the mode, a latched fault, the time of the last
  full charge and the plug address in `logs/charger_state.json`. It tells
  the main loop when the plug is on in a running charge
  (`holds_plug_on()`, see "Armed" below).
- `config.py` `ChargerPlugConfig` (`config.charger_plug`): every threshold.
- Web: `GET /api/charger`, `POST /api/charger/mode` with `{"mode": ...}`,
  `POST /api/charger/clear_fault`. The buttons are on the main page. The
  `/debug` board shows the same data read-only.

## Modes
| Mode | What the plug does | Ends |
|---|---|---|
| `hold` (default) | On when the rested cell voltage is 3.70 V or less (about 43 %). The top-up is sized to 57 % from the pack's rest curve and counted in amp-hours: about 1 Ah, 17 min, six times a day with the main switch on. Charges start at least 30 min apart. | never |
| `full` | On until the pack is full by voltage (54.3 V, or a cell at 4.17 V) and the charge current is under 0.3 A for 2 min. A top-up when the rested cells fall to 4.08 V. | by itself: when the cells were above 4.00 V and rest at 3.80 V or less (a drive), or 48 h after the command |
| `on`, `off` | Manual. While the robot is armed, `on` follows the rules in "Armed". | never |

WARNING: Do not change the hold trigger to the BMS percentage. The BMS
undercounts the idle drain by about 16 %. A hold on its percentage lets the
real level fall by about 20 points a day while the page shows 50 %.

"Rested" means: plug off, current 0.5 A or less, 5 min after the last charge
and 2 min after the last load above 1 A. The cell voltage is (min + max) / 2.

## Health: when the robot will not charge
A fault latches. The plug stays off in every mode until you clear the fault
(CLEAR FAULT on the main page, or `POST /api/charger/clear_fault`). It
latches again if the cause is still there. A fault survives a restart.

A fault condition must show on 3 polls in a row (about 30 s) before it
latches. The plug goes off on the first poll. The reason: the BMS reader
does not check frame checksums, and one garbled frame read 281.5 A on
2026-11-16. For the same reason a charge current above 6 A is not counted.

| Fault | Trigger |
|---|---|
| `bms_flag` | The BMS reports Charge FET Failure, Discharge FET Failure, Cell Mismatch, Cell Voltage Diff Trip, Temp Sensor Diff Trip, Cell OVP Trip, Pack OVP Trip or Charge OCP Trip |
| `cell_low` | Lowest cell under 3.00 V at rest, or under 2.50 V at any current |
| `cell_spread` | Cell spread 100 mV or more in the middle, 60 mV or more at the top: 5 min at rest, or 1 min under charge |
| `cell_count` | The BMS does not report 13 cells |
| `bms_data` | Pack voltage and cell voltages disagree for 3 polls |
| `over_temp` | Pack at 50 C or more |
| `temp_rise` | Pack 10 C warmer than at the start of the charge (normal: 0 to 2 C) |
| `charge_overrun` | A full charge longer than 4 h, or the plug still on 5 min after the policy switched it off |
| `charge_overfill` | More than 8.5 Ah into the pack in one charge |
| `charge_no_rise` | 0.6 Ah into a hold charge and the cells rose less than 10 mV (normal: 45 mV) |

After `cell_low` the pack must be charged by hand. Put the charger straight
into the wall, stay with it, then clear the fault.

An inhibit pauses `hold` and `full` and clears by itself: pack under 5 C
(under 10 C for a full charge) or over 45 C, a BMS charge-temperature trip,
the charge FET open, one sample of disagreeing BMS voltages. The armed rule
is in "Armed" below.
A cell at 4.23 V or more ends a full charge at once; that charge counts as
complete, with no second try.

BMS warnings do not stop a charge. This pack shows "Cell OVP Warning" and
"Pack OVP Warning" at every full charge.

## Dead-man timer in the plug
While the plug is on in `hold` or `full`, the manager keeps a countdown rule
in the plug itself: "off in 5 min". It arms the rule again every minute. If
the Pi halts, hangs or loses the Wi-Fi link, the plug ends the charge
without the robot. Manual `on` has no countdown, except while the robot is
armed during a running charge. A timer that you set in the
Kasa app is replaced. Tested on the plug on 2026-11-16. The countdown is
also kept while the robot is armed. It is not armed again while the manager
wants the plug off: if the OFF switch fails, the countdown that is already
in the plug ends the charge within 5 min (before 2026-11-25 a failed OFF
pushed it out every minute).

## Cell imbalance
The BMS balancer starts at 4000 mV per cell (about 83 %). A 50 % hold never
balances. A full charge does: the cells line up at the top, and the balancer
keeps working at rest for about 3.5 h after the charge, while the cells stay
above 4.0 V.

This pack has one cell group about 10 % weaker than the others. Its normal
rested spread is about 30 mV in the middle and under 10 mV at the top. Below
3.55 V per cell the weak group falls away (330 mV at 3.35 V), so the spread
is not evaluated below 3.65 V.

| Zone (rested cell voltage) | Normal | Notice | Warning | Fault |
|---|---|---|---|---|
| Middle, 3.65 to 3.95 V | 30 mV | 45 mV | 60 mV | 100 mV |
| Top, 3.95 V and above | 8 mV | 20 mV | 35 mV | 60 mV |

A level must hold for 5 min. On a notice or a warning: tap CHARGE FULL
(Kevin's rule: the robot flags, he decides). The robot also gives a notice
when 30 days pass with no full charge. If the spread in the middle is still
high after a full charge, the weak group has lost more capacity; balancing
cannot repair that.

## BMS dark
The BMS sleeps about 1 h after its current reaches 0.0 A. That occurs only
when the charger holds a full pack. With the plug off, the 0.3 A idle load
keeps it awake (23.6 h without a gap on 2026-11-11). If the link goes dark
anyway:

1. After 2 min with no reading the plug goes off.
2. Nothing more occurs unless all of these are true: the VESCs give a pack
   voltage and a heatsink temperature from 8 to 45 C, and the last BMS
   reading had no inhibit.
3. Wake pulse: after 10 min, the plug goes on for 3 min if the VESCs read
   52.0 V or less. A charger 2 V or more above the pack wakes a Daly BMS.
   Next pulses after 30 min and 1 h; 3 a day at most.
4. Rescue: if the VESCs read 46.8 V or less (about 25 %), the plug goes on
   for 12 min, at most once an hour and 10 times a day. The pack then holds
   near 25 to 35 % and does not run flat.
5. Both stop until the BMS is back when the VESC voltage does not rise 0.2 V
   in the first 2 min of a pulse (charger not connected, or a stuck reading).
6. The rescue stops until the BMS is back when the pack is at its old
   voltage again less than 90 min after a rescue. A healthy pack keeps a
   rescue for about 3 h. A pack that does not is not charged blind.

In the dark the robot sees no cell voltage and no pack temperature. The BMS
hardware protections, the charger's 54.6 V limit and the dead-man timer are
then the guards. Kevin accepted that for these bounded cases (2026-11-16).

## Armed: the charger inhibit (Kevin, 2026-11-25)
The charger inhibit holds the motors at neutral. It has two sources:
- The BMS: charge FET on and more than 2.0 A, 2 polls in a row.
- The plug: the plug is on in a running charge, in any mode. The main loop
  reads it on every tick.

The plug source covers the end of a charge. There the current falls under
2.0 A while the charger is still on.

The rules:
- No charge starts while the robot is armed.
- A charge that runs when the robot arms goes on. In `hold` and `full` all
  its normal stop rules still apply: the amp-hour target, full and tapered,
  the faults, the temperatures, the cell limits, no current for 5 min, BMS
  dark for 2 min, the countdown in the plug. PLUG ON has none of these
  except a latched fault and, while armed, the countdown in the plug: it
  goes on until you tap PLUG OFF.
- "Runs" means: the BMS showed 1.0 A or more of charge current on 2 polls
  while the robot was disarmed.
- The charge is kept only while the plug source holds the robot. A charger
  is never left on for an armed robot that is free to drive.
- Once a charge is stopped while the robot is armed, it is over. If the plug
  does not switch off, the manager keeps sending OFF, does not arm the
  countdown again, and the robot stays held while the plug reads on. This
  holds also after a disarm and after a mode change (PLUG ON does not switch
  it back on), until the plug is seen off.
- Anything else goes off at the arm, as before. That is a plug that is on
  with no current (the lead is not on the robot), a charger that has not
  started yet, a wake pulse or a rescue in the dark, a plug that the Kasa
  app or Alexa switched on while the robot was armed.
- When the charge ends, the plug goes off and the plug source releases at
  once. The motors ramp up from neutral. A held stick does not step to the
  motors.
- Disarming changes nothing.
- These rules hold in every mode, PLUG ON included (2026-11-25). PLUG ON
  starts nothing while the robot is armed. A plug that is on in PLUG ON
  with no charge goes off at the arm, and on again at the disarm. A charge
  that runs at the arm in PLUG ON goes on and holds the robot until you tap
  PLUG OFF. While armed it has the countdown.
- A mode change does not free the robot. After PLUG OFF, PLUG ON or another
  mode, the plug source still holds while the plug reads on.
- The plug does not answer: the plug source keeps the last answer for 6 min
  (the 5 min countdown and 1 min for a refresh in progress), then releases.
  By then the plug's countdown has ended the charge, and the manager no
  longer keeps it for an armed robot. A hung manager thread also releases
  after 6 min.
- A charge can start to count as running while the plug does not answer
  (the manager goes by the last known state). The plug source is then set
  on that poll, once. Further polls with no answer do not extend it.
- With `charger_inhibit_enabled` off (`config.bms`), there is no plug
  source, and arming stops a charge as before.
- After a restart (every deploy) the manager must see the current again on
  2 polls before a charge counts as running: about 25 to 35 s, because the
  BMS reconnects first. The arm interlock covers the start: the robot does
  not arm until the arm switch has been seen off. If you cycle the switch
  in that time, the plug source does not hold the robot, and the plug goes
  off within 10 s, as before this change.
- A restart also forgets that a charge was stopped while armed. If that
  plug did not switch off, the new process takes it as a running charge
  again once it sees the current on 2 polls. It then holds the robot while
  the plug reads on, and the normal rules apply. A latched fault survives
  the restart.

WARNING: A charge that runs when you arm holds the robot still until it
ends. A hold charge takes about 17 min. A full charge takes about 1.5 h
from 50 %.

To drive during a charge:
1. Tap PLUG OFF on the main page. The plug goes off and the robot is free.
2. Take the charger lead off the robot.
3. Drive. Afterwards tap HOLD 50 %. PLUG OFF stays set until you do.

If you take the lead off first and then arm, the robot waits for the
no-current rule: about 10 s when the charge has run 5 min, otherwise until
it has run 5 min. PLUG OFF ends the wait at once. In PLUG ON there is no
such rule: the robot waits until you tap PLUG OFF.

Known limit: with the plug off, nothing knows that the lead is on the robot.
In `hold` the plug is off about 22 h of 24. If you arm then with the lead
on, the robot can drive off with it. The same holds after the end of a
charge, in the first 20 to 30 s of a charge (before the current shows on 2
polls), during a wake pulse or a rescue in the dark, and when a charger
does not start on a nearly full pack. Take the
lead off before you arm.

## Other behaviour
- Plug on but no current for 5 min (the charger lead is not on the robot):
  plug off, next try after 15 min, notice on the page.
- The same in `full` with the cells at 4.08 V or more: the charger does not
  start on a nearly full pack. The plug goes off and the charge counts as
  the top, with no retry until the top-up level. Seen on 2026-11-16: 6.5 min
  with the plug on at 53.9 V and no charge current, charger and lead in
  place (Kevin). CHARGE FULL on a pack above about 53 V can therefore do
  nothing until the pack has come down.
- Charge current with the plug off for 2 min (the charger is on another
  outlet, or the plug's relay is stuck): warning. The robot cannot stop that
  charge.
- BMS and VESC pack voltage more than 1.5 V apart at rest: warning.
- OFF is never delayed. An automatic ON waits 60 s after the last switch.
- System shutdown (ch5 e-stop halt, main switch off, `poweroff`): plug off
  in `hold` and `full`. A `wall-e` restart (every deploy) leaves the plug
  alone. A hard power loss of the Pi mid-charge leaves the plug on in
  mode `on` only; the charger then ends the charge itself at 54.6 V. In
  `hold` and `full`, the dead-man timer in the plug ends the charge in 5
  minutes or less (see "Dead-man timer in the plug"). So does `on` while
  the robot is armed during a charge kept in `on` (see "Armed").

## Operating it
- Phone: main page on port 8080, panel "Charger Plug": HOLD 50 % / CHARGE
  FULL / PLUG ON / PLUG OFF, the alerts, and CLEAR FAULT when one is latched.
- Command line:
  ```bash
  curl -s http://192.168.86.54:8080/api/charger
  curl -s -X POST -H 'Content-Type: application/json' -d '{"mode":"full"}' http://192.168.86.54:8080/api/charger/mode
  ```
- Journal: `journalctl -u wall-e | grep -i "charger"`. Each switch, mode
  change, alert and fault is a WARNING line.

## Gotchas
- The hold never discharges anything. A full pack stays full until the next
  drive or until the idle load brings it down (about 13 h from 100 % to the
  band with the main switch on).
- A pack at 50 % gives about half the range. Say "charge it fully" the day
  before a long run; 50 % to full takes about 1.5 h.
- The percentage on the page is "about N % by cell voltage", from the rest
  curve of this pack, within about 3 points. The BMS percentage is shown
  next to it and drifts; a full charge sets it back to 100.
- Alexa and the Kasa app see the plug too. In `hold` and `full` the manager
  decides again within 10 s, so a switch from there is undone. To take the
  plug by hand use PLUG ON / PLUG OFF on the robot page.
- Phone alerts come from the home dashboard (Telegram), which reads
  `GET /api/charger`. That part lives in the home-automation repo.

## Independent review (Grok, session a41299a1, 2026-11-16)
Adopted: the dead-man timer; "full" by voltage and a real charge taper, not
by the BMS percentage; a high cell ends a full charge with no retry;
over-voltage and charge over-current trips latch; the spread fault also
under charge; the checks on blind charging (rules 2, 5 and 6 above); 3 wake
pulses a day; 30 min between hold charges; the no-rise fault; the 3.00 V
rest limit; the 48 h limit on `full`.

Not adopted, with the reason:
- A series contactor with a heartbeat. It is hardware. The countdown in the
  plug covers a dead Pi and a lost link; it does not cover a welded relay.
- A latch at 4.17 V per cell. A normal full charge of this pack ends at
  4.19 to 4.20 V.
- No rescue at all. Kevin chose the rescue; it has the checks above.
- No hold charge below 15 C. The published limit for fast charge is 5 C, and
  a hold that stops at 15 C lets the pack run down in a cool garage.
- "The hold window is inside the noise." Measured: 1 Ah moves the rested
  cells by 76 mV here, and the cell reading has 1 mV steps.
- A latch on a temperature rise at idle. There is no ambient reference yet.

## Kevin's calls (2026-11-16)
1. The logic runs on the robot, as a scripted routine.
2. The plug is found by MAC. No DHCP reservation.
3. Do not charge an unhealthy pack. Flag imbalance.
4. Balancing: the robot flags, Kevin taps CHARGE FULL. No automatic full charge.
5. BMS dark: wake pulses, and the low-voltage rescue on the VESC reading.
6. Full mode returns to hold by itself after the robot has been used.
7. The main switch is normally on while the robot sits on the charger.
8. Faults and warnings also go to his phone (Telegram, home dashboard).
