# Charger plug: research and measurements (2026-11-16)

This is the evidence for the numbers in `ChargerPlugConfig` and the rules in
`docs/charger_plug.md`. Kevin asked for it on 2026-11-16: find the plug by
MAC, do not charge an unhealthy pack, flag imbalance, and check whether
balancing needs a charge to 100 %.

## 1. Data
The battery fields of every `run_*.log` and `arm_*.log` on the robot (83
files, 2.8 GB), one sample each 20 s: 19 899 samples from 2026-11-08 to
2026-11-16, of which 13 112 have a fresh BMS reading (from 2026-11-10; the
BMS battery type was set to Ternary that day). "Cell voltage" below is
(min + max) / 2. "Rested" is current 0.45 A or less, 10 min after any current
above 1 A.

## 2. The BMS percentage drifts on the idle load
The BMS shows -0.3 A at idle, but its percentage falls as if the load were
0.21 A (three idle periods of 1.1 h, 6.3 h and 13.5 h: 0.209, 0.211 and
0.215 A). The charger then shows what was really missing:

| Idle period before the charge | BMS at charge start | Amp-hours to full | Real level at start | BMS error |
|---|---|---|---|---|
| 1.1 h (11-15 16:11) | 81 % | 1.37 + 0.15 Ah = 22 % | 78 % | +3 |
| 13.5 h (11-15 19:50) | 59 % | 3.46 + 0.15 Ah = 52 % | 48 % | +11 |
| 6.3 h (11-16 10:45) | 81 % | 1.57 + 0.15 Ah = 25 % | 75 % | +6 |

(The 0.15 Ah is the tail under 1.0 A, which the session sum leaves out. 7.0 Ah
pack. The charger current, 3.6 A, is measured to 0.1 A, so the amp-hours are
good to about 3 %.)

The BMS reads about 0.8 point too high for each idle hour. In a hold on the
BMS percentage, each cycle the BMS sees 10 points go out when about 12 went
out, and the charger puts 10 back. The real level falls about 2 points per
cycle, about 20 points a day, and the page still shows 50 %. So the hold
uses the rested cell voltage, which does not drift, and counts amp-hours only
while charging, where the BMS current is accurate.

The same rested voltage also read different percentages on different days:
3.75 V was 52 % on 11-12 and 60 % on 11-16.

## 3. Rest curve of this pack
Rested cell voltage against the BMS percentage, 11-10 to 11-16 (medians).
The charge counts of section 2 put the curve about 2 to 3 points high in the
middle (3.738 V counted as 48 %, the table gives 50 %).

| Percent | Cell V | Pack V | | Percent | Cell V | Pack V |
|---|---|---|---|---|---|---|
| 7 | 3.40 | 44.6 | | 57 | 3.776 | 49.1 |
| 12 | 3.49 | 45.6 | | 66 | 3.82 | 49.7 |
| 17 | 3.56 | 46.2 | | 71 | 3.87 | 50.3 |
| 27 | 3.61 | 46.9 | | 76 | 3.92 | 51.0 |
| 37 | 3.67 | 47.6 | | 82 | 3.98 | 51.7 |
| 42 | 3.70 | 48.0 | | 87 | 4.04 | 52.5 |
| 47 | 3.72 | 48.4 | | 92 | 4.09 | 53.1 |
| 52 | 3.75 | 48.8 | | 100 | 4.20 | 54.6 |

Hold: on at 3.70 V (about 43 %), top-up sized to 57 %. Under charge at 3.6 A
the cells read about 30 mV higher than at rest (49.0 V at the start of the
11-16 charge from a real 48 %), so the backstop under charge is 3.83 V.

## 4. Cell spread: one weak group
Rested spread (max - min) against the cell voltage, all dates:

| Cell V | Median | 90 % | Max | | Cell V | Median | 90 % | Max |
|---|---|---|---|---|---|---|---|---|
| 3.35 | 329 mV | 350 | 354 | | 3.75 | 28 mV | 30 | 32 |
| 3.40 | 260 | 294 | 303 | | 3.80 | 23 | 26 | 27 |
| 3.45 | 183 | 214 | 223 | | 3.85 | 19 | 21 | 22 |
| 3.50 | 114 | 143 | 150 | | 3.90 | 14 | 17 | 19 |
| 3.55 | 47 | 74 | 84 | | 3.95 | 9 | 11 | 14 |
| 3.60 | 33 | 41 | 43 | | 4.05 | 10 | 10 | 12 |
| 3.65 | 34 | 35 | 36 | | 4.15 | 5 | 7 | 9 |
| 3.70 | 33 | 34 | 37 | | 4.20 | 7 | 9 | 11 |

The pack is balanced at the top and spreads as it empties. That is a
capacity difference, not a charge imbalance: one group has about 10 % less
capacity (30 mV at 50 % is about 5.5 points on a 5.5 mV per point slope).
Balancing cannot remove it. At 7 % the weak cell was at 3.17 V with the
average at 3.35 V, so the hold must keep the pack out of the bottom, and the
imbalance flag must compare with this normal: notice at 45 mV in the middle,
not at 30.

## 5. Balancing
- BMS setting: start 4000 mV, 10 mV difference, 1.0 A, active
  (`docs/battery_settings.md`). The Daly active-balance manual gives two
  conditions, the start voltage and the voltage difference, and no
  charging-only condition.
- The logs agree. Each charge ends with a spread of 10 to 11 mV, which is
  the balancer threshold, and the spread stays at 3 to 11 mV for the 6 h
  after the charge with the charger off.
- So Kevin is right that a hold at 50 % never balances: 4000 mV is about
  83 %. The cells line up only at the top. A full charge is also the only
  event that sets the BMS percentage back.
- With the plug off after a full charge, the cells stay above 4.0 V for about
  3.5 h on the idle load. At 1.0 A that is ample balancing time. No soak at
  the charger's float voltage is necessary.

## 6. Charges seen
| Start | Minutes above 1 A | Pack V | Amp-hours | Highest cell | Spread | Temperature |
|---|---|---|---|---|---|---|
| 11-15 17:20 | 29 | 51.9 to 54.4 | 1.37 | 4194 mV | 15 to 11 mV | 22 to 22 C |
| 11-16 09:22 | 63 | 49.0 to 54.4 | 3.46 | 4195 mV | 33 to 11 mV | 22 to 24 C |
| 11-16 17:05 | 31 | 51.4 to 54.4 | 1.57 | 4193 mV | 13 to 10 mV | 24 to 24 C |

After the current falls to 1.0 A it reaches 0.4 A in 10 min and 0.1 A in
30 min. The only BMS flags in the whole record are Cell OVP Warning and Pack
OVP Warning (at every full charge) and SOC Low Warning. The charge FET was
never open in this record. Pack temperature: 22 to 27 C.

## 7. BMS sleep and wake
- `docs/charger_inhibit_review_2026-11-10.md`: the link goes dark 45 to
  62 min after the current reaches 0.0 A (5 of 5 cases). That occurs only
  when the charger holds a full pack.
- On the idle load the BMS stays awake: 23.6 h without a gap on 11-11/12,
  13.5 h on 11-15/16.
- Daly (control strategy page): sleep after 3600 s with no current and no
  communication. Wake on a discharge of about 2 A or more, on a charger at
  least 2 V above the pack, on communication or on the button. The MOSFETs
  stay on in sleep.
- At 50 % the charger (54.6 V) is about 6 V above the pack, so a wake pulse
  works there. On a full pack it does not, and it is not needed.

## 8. Limits from outside sources
- Temperature (Battery University BU-410): no charge below 0 C; fast charge
  from 5 C to 45 C; many chargers stop at 50 C. The 4 A charger is 0.57 C
  for this pack, so the software window is 5 to 45 C, and 10 C for a full
  charge (cold and a high state of charge is the plating case). The BMS
  itself stops at 1 C and 50 C.
- Deep discharge (CALCE, J. Electrochem. Soc. 2020 and 2025): below the
  lower limit, about 2.5 V, copper from the anode collector dissolves and
  can short the cell later. A cell under 3.00 V at rest (the BMS cuts off
  at 3.00 V under load), or under 2.50 V at any current, therefore latches a
  fault; a person must decide about that pack.
- Spread: rule-of-thumb tables give 0 to 30 mV as good, 50 to 100 mV as
  "needs attention" and above 100 mV as a weak cell or a bad connection.
  These are not a standard. Section 4 is the better reference for this pack.
- Balance charges: e-bike practice is a full charge every few weeks, because
  most BMS units balance only near the top. Kevin's rule: the robot flags,
  he taps CHARGE FULL.

## 9. Replay
The new policy, in hold mode, run over all 19 899 samples with the plug held
off: no fault, no inhibit, no spread alert, and one charge decision, at
3.700 V rested on 11-12 10:55 during the run-down (about 43 %).

## 10. Review and what it changed
Grok attacked the design the same evening (session a41299a1). The list of
what was adopted and what was not is in `docs/charger_plug.md`. Two of its
points were bugs: "full" could complete on a half-empty pack when the BMS
percentage read 100 and the charger lead was off, and a cell at the 4.23 V
stop made the relay cycle about once a minute. One point changed the design:
the plug now carries its own off-countdown (tested on the plug: a rule set
over the local protocol counts down and switches the relay), so a dead Pi
cannot leave the charger on.

Replay with the final policy: hold mode, no fault and one charge decision as
in section 9; full mode over the three charges of section 6, complete at
18:04, 10:40 and 17:51, each 2 min after the current fell to 0.3 A at 54.5 V.

## 11. Not known, not tested
- The cell model, so the true capacity (7.0 Ah assumed) and the datasheet
  temperature limits.
- A wake pulse on a sleeping BMS at 50 %. Daly's rule says it works.
- The rescue top-up. It needs a dark BMS and a pack under 46.8 V.
- A hold cycle on the robot. The first one is due when the pack reaches
  3.70 V per cell.
- The dead-man countdown ending a real charge (the countdown and the relay
  action were tested with the plug off and with an on-rule).
- The BMS protections in sleep. Daly says the MOSFETs stay on; the
  over-voltage and temperature trips were not tested asleep.

## Sources
- Daly, "Smart BMS control strategy": https://dalybms.com/news/daly-smart-bms-control-strategy
- Daly Smart Active Balance BMS manual: https://manuals.plus/ae/1005010148694914
- Battery University BU-410, charging at high and low temperatures: https://batteryuniversity.com/article/bu-410-charging-at-high-and-low-temperatures
- CALCE, copper dissolution in overdischarged cells: https://calce.umd.edu/news/story/understanding-the-nature-of-copper-dissolution-in-overdischarged-lithiumion-batteries
- Current Connected, SOC drift: https://www.currentconnected.com/learn/soc-drift-lithium-batteries
- Spread rule of thumb: https://www.bestpcbs.com/blog/2026/07/how-much-can-voltage-vary-in-a-bms/
- Electric Bike Review forum, battery balancing: https://forums.electricbikereview.com/threads/battery-balancing.41631
