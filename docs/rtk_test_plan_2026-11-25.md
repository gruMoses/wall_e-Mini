# RTK test plan for 2026-11-25: does PV power break the fix?

Background: `docs/rtk_float_investigation.md`. This plan replaces the "PV off for 20–30 minutes" design in its section 2.3 for today. It uses the data of 2026-11-24, read again from the run logs and the PV history.

## 1. What the 2026-11-24 data adds

Corrections to the runbook text (section 2.2):

- The 14:58–15:02 fix was not at the afternoon spot. The robot stood about 80 m from it (parked 14:33–15:22). The afternoon spot has three fixes: 16:03, 16:10 and 19:04 (the last one 2.7 m away).
- Every park in the open gave a fix, but only after a long wait: 25 min (14:33 to 14:58), 38 min (15:25 to 16:03), 17 min (18:47 to 19:04). The specification is under 10 s.
- The fix never survives a drive. It dropped 4 s and 13 s after the wheels started. So float during a drive (2026-11-08, and most of the "−5 h to −3 h" row) says nothing about the time of day.
- PV at the fix starts: 76 W, 91 W, 119 W, 0 W. Both losses with the robot standing still came on a rising PV: 15:01:59 at 106 W, 16:09:37 at 121 W. The step to 343 W came about 1 minute after the 15:01:59 loss, not before it.
- A load of about 500 W cycles on the container system (10 to 45 min on). At 16:18 it started with the sun low: PV stayed at 56 W, the battery gave 233 W, and the fix held. Same result as 2026-11-07: the inverter's AC side does not break the fix.
- Late corrections (differential age 2 s or more): about 10 % of the samples at the afternoon spot 15:26–16:20, none at 18:48–19:18, none at the 80 m spot.
- At midday the battery was full and the load was 25 W, so PV sat at 43–86 W from 12:30 to 13:30. That is below every PV value at which the fix was lost. The robot had no corrections then (not at the spot), so there is no midday data.

## 2. Causes, ranked

1. **PV-side noise (MPPT and string wiring) with a low threshold, about 100 W.** For: 2026-11-07 (parked 4 h 18 min in float, fixed when PV fell to about 100 W); all fix starts at 119 W or less; both standing losses at 106–121 W rising; the fix at 0 W after sunset; the window follows sunset, not the sidereal day. Against: no known reason for a threshold near 100 W; no data yet separates it from sun elevation; 2026-09-19 15:27 fixed on a sunny day (PV unknown, and a full battery could have curtailed it). Only this cause predicts: a fix at solar noon while PV is below 90 W, a loss when PV is pushed above 300 W, and no loss with PV off and the same load on the inverter.
2. **Weak ambiguity resolution (L1-only observations or thin correction epochs over LoRa), plus sampling bias.** For: 17–38 min to fix, loss within seconds of any drive, most daytime data is from drives. Against as the only cause: 2026-11-07 was parked for hours. Predicts: a fix within 45 min of parking at any hour, also in the morning while PV charges at 0.5–3 kW. Probably true as the underlying weakness that lets a small disturbance win.
3. **Sun elevation or solar radio noise.** For: fits the sunset window as well as PV does. Against: the 14:58 fix with the sun at about 27°; a 90 m baseline cancels the ionosphere. Predicts: float at solar noon whatever PV does, and no reaction to any switch.
4. **Rover-side reception (LoRa margin, missed packets, antenna).** For: the late corrections above; drive losses came with ages of 2–3 s; section 2.4. Against: the 16:03–16:22 fix held with late corrections present. Predicts: losses follow late corrections, not PV.
5. **Temperature.** Against: fixed at 14:58, near the warmest hour; the base unit is shaded. Predicts: float through the warm hours whatever PV does.
6. **Roof multipath at the base, satellite geometry.** Against: fix times do not move 4 min earlier per day. Predicts: the same clock pattern at one spot day after day, minus 4 min.
7. **Base coordinate or survey-in.** Cannot make a daily pattern. It matters for waypoint offsets only.
8. **Sampling bias alone.** Ruled out by 2026-11-07.

## 3. Is PV-off still the right test today?

Yes, but not alone, and not as the first step.

- With the battery full, PV is already curtailed to the load (43–86 W at midday yesterday). Switching it from 70 W to 0 W changes little. If the rover fixes at 70 W, PV-off has nothing left to show.
- The stronger step is the reverse: push PV **up** with a load of 500–1500 W while the battery is full. That needs no DC switching. PV-off then serves as the control (same load, PV silent) and as the fallback if the rover stays float at midday.
- Park in the morning. The charge phase (PV 0.5–3 kW) against the curtailed phase is a free comparison, and it tests cause 2.
- Risk: none of these steps restarts the base. A base restart starts a new survey-in and shifts waypoint coordinates. The short-baseline test and the tap on the base header P1 wait: the tap opens the base.

## 4. Today's steps (Kevin)

The assistant only reads (journal, run log, PV history). It sends no motion command. Do not push code to `main` during the test (a `wall-e` restart makes a log gap).

1. By 09:30, drive the robot to the afternoon spot of yesterday (where it stood 15:25–16:22 and 18:47–19:20). Disarm. Tell the assistant. It reports the distance to yesterday's position; move again if that is more than 3.5 m.
2. Do not move the robot until the test ends. Every move costs 20–40 min.
3. WARNING: Do not switch off the inverter or the battery breaker at any step. The base runs from the inverter, and a base restart shifts the waypoint coordinates.
4. Morning (no action): PV charges the battery. The assistant logs. Write down when the 500 W load starts and stops, if you see it.
5. Wait for battery 100 % and PV below 90 W (yesterday: 12:30). The assistant tells you. **Quiet 1:** change nothing for up to 45 min. The phase ends when the rover is fixed for 5 min.
6. **Noisy 1:** switch on a load of 500–1500 W on the container system (a heater, thermostat at maximum). PV must rise above 300 W. Hold 15 min. Then switch the load off.
7. **Quiet 2:** load off, up to 45 min, until fixed for 5 min.
8. WARNING: Do not open a DC disconnect under load unless it is load-break rated. A DC arc at string voltage does not go out like an AC arc. Prefer to disable PV charging in the inverter menu. If only a switch exists, use a load-break-rated PV switch, and only while PV is curtailed.
   **Control:** stop the PV input (PV 0 W). Then switch the load on. Hold 15 min. Then load off, PV input on again.
9. PV recharges the battery for some minutes after step 8. Wait until PV is below 90 W again and the rover is fixed for 5 min (up to 45 min).
10. **Noisy 2:** load on, 15 min, then off. End of the test. Leave the robot parked until after sunset if you can: the evening fix proves the spot worked today.

Fallback A: no fix in Quiet 1 after 45 min. Do the warning of step 8, then stop the PV input for 45 min (PV 0 W, no extra load). Then PV on for 15 min. Do this off-on cycle two times.

Fallback B: the battery is not full by 13:00 (cloud). PV is then not curtailed. Use Fallback A directly.

Time: about 2.5 h after the battery is full; the best window is 11:30 to 14:30.

## 5. What the assistant logs

- Each minute: fix quality, satellites, HDOP, differential age, `sog_mps`, and the distance from the parked position (from `logs/latest.log`); `pv_power`, `load_power`, `battery_soc`, `battery_power` (PV history).
- Each `fix_quality` transition from the journal, to the second, with the PV value before and after.
- Per phase: time to fix, share of samples at quality 4, share of samples with a differential age of 2 s or more.
- The uncontrolled cycles of the 500 W load: each one is a free extra "noisy" sample.
- A message to Kevin at each phase change, and at once if the corrections stop (quality 2) or the robot moves.

## 6. Pass / fail

Definitions: fixed = quality 4 on 90 % or more of the samples in 5 min. Quiet = PV below 90 W. Noisy = PV above 300 W. PV from 90 W to 300 W does not count for either side.

- **PV noise confirmed** when all three hold: (a) a fix within 45 min in Quiet 1 and in Quiet 2; (b) in Noisy 1 and in Noisy 2 the fix is lost within 3 min of PV passing 300 W and stays float for 10 of the 15 min; (c) in the Control the rover stays fixed (quality 4 on 90 % or more of the 15 min).
- **PV noise confirmed, at low power too** (Fallback A): a fix within 45 min of PV 0 W in both cycles, and float again within 5 min of PV on, lasting 10 min, in both cycles.
- **PV noise rejected:** the rover stays fixed through both Noisy phases; or it fixes in the morning for 5 min or more with PV above 300 W; or it stays float for 45 min at PV 0 W in both cycles of Fallback A and then fixes in the evening at the same spot.
- **Not decided:** anything else, for example a loss in the Control (then the load or the inverter is involved), or one cycle each way. Short losses of 1–2 min happen without a cause (4 on 2026-11-07, 1 yesterday); that is why a loss must last 10 min and repeat.

## 7. Why the test works although the inverter powers the base all day

The solar system has two halves. The output half makes AC from the battery; it runs day and night and feeds the base. The input half is the MPPT charge stage and the long PV wires to the ground mount; it works only when the panels produce. The rover fixes in the evening while the output half runs at full work (600 W on 2026-11-07, 500 W yesterday at 16:18), so the output half is not the suspect. The input half goes quiet at night. The test makes the input half quiet or busy at midday while the output half and the base stay on.

## 8. Cheaper diagnostics for the next test (proposals, nothing done)

1. Log per-satellite C/N0 with the signal id (L1 against L5) from the rover. The driver (`pi_app/hardware/rtk_gps.py`) reads only the position registers and the RMC/VTG sentences (registers 83–88); it has no GSV read. The vendor library has a full-NMEA buffer (`get_all_gnss()`, register 91 per the runbook); not verified that it carries GSV. First a one-time probe with Kevin's word, then a logger outside the control path. C/N0 at 500 W against 50 W answers the PV question in minutes, and the signal ids answer cause 2.
2. A summary line in the run log per minute: count of samples with a differential age of 2 s or more. The data is already there; it tests cause 4.
3. Park-and-wait record: time from standstill to fix, at any hour, kept in one table. It tests cause 2 without any switch.
4. PV history beyond 48 h: a small job on the home-automation Pi that appends `pv_power` each minute to a file. Old fix times then get their PV value (2026-09-19 15:27 stays unknown today).
5. The listener beside the robot for one afternoon: its burst count against the rover's late corrections (cause 4). Receive only; the payload stays unread.
6. Listen-only tap on the rover's GNSS output (LC29H pin 20, runbook 2.6): full NMEA with GSV, no base restart. Bench work on the rover board.
7. Later, with a planned base restart: the tap on the base header P1 (message set, L5 content, coordinate), and the short-baseline test (runbook section 4).
