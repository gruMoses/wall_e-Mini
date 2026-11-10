# Status audit, 2026-11-10

This document records an audit of the documents against the code on `main` at commit `ec57850`. It uses Simplified Technical English where practical.

Method: five read-only review agents (safety and arming; follow-me and vision; heading, GPS and waypoint navigation; power and services; the backlog), a Grok check of the settings in `CLAUDE.md` against `config.py`, the full test suite on the Mac, and the live robot after it came back on the network at 10:31.

No code changed. This commit corrects stale statements in the documents only (section 7).

## 1. Summary

- Tests: 1,880 pass and 6 skip. One fails on the Mac only (`test_camera_ws_route_registered_only_when_frame_source_present`), because `flask-sock` is not installed there.
- Settings: Grok compared 55 settings that `CLAUDE.md` names with `config.py`. All agree (session `01a1c633-303e-72d3-b343-3fbb3573fdb0`).
- Wiring: each protection that the documents describe is in the live control path as described. Checked: the follow-me stale-detection gate, the colour-stall watchdog, the vision-hang restart with its gate and budget, the arm fail-safe, the startup interlock, auto-disarm, the netting mute, the MANUAL obstacle curve, the stick expo, and the heading sign (no second inversion).
- Safety: there are eight gaps (section 2). None is fixed.
- The software pack-low latch cannot trip (section 3).
- The UPS battery does not hold the Pi (section 4).
- Live robot at 10:35: `wall-e` on `ec57850`, camera connected (detections 15 fps), GPS 21 satellites with no corrections, BMS connected, the four services active, no errors in the journal, disk 12 percent used.

## 2. Safety gaps, in priority order

Each fix is a motion-safety change. Get a Grok safety review and Kevin's approval before each push. Do not put two of them in one push. The order below is the result of a Grok second opinion (session `01a1c63a-1c96-7360-8abe-8b1e259b1a72`), which moved item 2 up and put item 3 before item 4.

STATUS 2026-11-10 18:00: items 1, 2, 3, 4 and 5 are corrected and deployed (section 9; item 1 at 17:50, after four versions failed the safety review, section 9.4). Items 6, 7 and 8 are open. A new gap came from the logs of 2026-11-07: the IMU stream can stop without a fault; its correction is deployed (section 9.3).

1. **The person stop releases at close range.** `_apply_safety_tier_override` in `pi_app/hardware/oak_depth.py` is calculated again at each depth poll and has no memory. Below about 0.4 m the person box is cut by the frame edge and the depth sample is not valid (`no_support`, `far_veto` or `ambiguous`). The backstop for a detection without depth needs a box width of 0.40 or more (`safety_stop_bbox_width`). The corridor ignores pixels at or below 350 mm, so it sees past the person. Result: one bad frame releases the stop. Evidence: 2026-11-08 14:27:40, FOLLOW_ME, the stop was off for about 0.6 s with Kevin at 0.3 to 0.9 m. Direction for the fix: hold the stop after a close stop-tier detection, and release it only when the depth shows a clear corridor or on an operator action. A release on a timer alone repeats the fault. NOTE: Kevin ruled that the pivot during the stop is acceptable. That ruling does not include this release.
2. **The waypoint API accepts any speed.** `/api/nav/start` and `/api/nav/go` in `pi_app/web/waypoint_nav_ui.py` write the client values `cruise_speed` and `arrival_radius` into `nav._cfg` with no limit and no authentication. A value of 127 is full speed. A negative value reverses. The values stay for later missions. Fix: clamp to the range of the sliders and refuse a negative speed.
3. **WAYPOINT_NAV does not protect its heading.** (a) An OAK reconnect during a run makes `_check_imu_frame_continuity` reset the GPS heading offset to 0. The run continues in the raw gyro frame, because the lock is checked only at activation. (b) `waypoint_nav_activation_blocked` skips the fix-quality check and the lock check when no GPS reading exists yet, for example directly after a restart. (c) ALIGN has no timeout, and nothing checks that the heading is fresh.
4. **WAYPOINT_NAV has no stale-detection gate.** FOLLOW_ME leaves when the detections are older than 1.5 s. WAYPOINT_NAV does not. If the colour camera stalls, the run continues on the depth corridor only (stop at 0.4 m) and the person stop is blind. After 3 restarts the watchdog latches `color_stall_fault`. No control code reads `color_stall_fault` or `detections_stale`.
5. **The RC-stale watchdog uses the wall clock.** `Controller.process` calculates `rc_age` from epoch time (`pi_app/control/controller.py`, near line 1648). A backward step of the clock makes a dead RC link look fresh. The phone override age in `pi_app/app/main.py` has no lower limit either. The main loop already uses `time.monotonic()` for its own RC check. This Pi's clock does step at boot (the RTC resets, then NTP corrects it).
6. **The pack-low latch cannot trip.** Refer to section 3.
7. **The charger inhibit is blind in two conditions.** (a) `is_charging()` needs the charge FET on and more than 2.0 A. During the constant-voltage taper and after a cell overvoltage trip it is False with the charger connected. (b) When the BMS Bluetooth link is dark, it fails open after 30 s (the link was dark for 42 hours before the 2026-11-08 drive). The discharge-FET guard in `main.py` keeps the last polled value with no staleness check. Auto-disarm and the startup interlock cover the parked case only in part.
8. **The UPS watcher does not retry a power-off.** `run_loop` in `bin/walle_ups.py` sleeps 600 s after a `"shutdown"` result. The 30 s retry and the 60 s forced power-off are in `step()`, so they do not run until that sleep ends.

Lower priority, also open:

- OAK not found at startup: there is no obstacle controller and no IMU. MANUAL has no obstacle scaling and no heading hold. WAYPOINT_NAV activates but gives zero commands, because there is no heading. A camera that fails after startup gets the 10 percent stale floor instead, so the two cases are not consistent. Kevin's decision is pending (`docs/depthai_upgrade.md`, section 6).
- A failed boot gravity check only logs a warning. Kevin's decision is pending (same section).
- An e-stop switch that is high at boot powers the Pi off about 5 s after the start, and no RTC alarm is set.
- Blind trail pursuit keeps a forward floor of 5 bytes for a maximum of 3 s (`pi_app/control/follow_me.py`).
- These web endpoints have no authentication and no local-only check: `/api/follow_me`, `/api/follow_me/params`, `/api/nav/*`, `/api/manual/netting_mute`.
- A push that changes `bin/` restarts `wall-e` but does not reinstall the root services. Those services run from the checkout, so the new code starts at their next restart or at the next boot.
- CONFIRMED later on 2026-11-10, refer to section 8: the defect below is real, and it has three users in FOLLOW_ME. The first text of this item follows. Possible defect, not verified: follow-me trail pursuit on GPS odometry. `pi_app/control/gps_odometry.py` keeps the heading as a math angle (0 = east, counter-clockwise positive). `pi_app/control/pure_pursuit.py` expects a clockwise heading with +y to the right. A trail point on the left can give a right steer. It did not run in the logs of 2026-11-04, 2026-11-06 and 2026-11-08: all follow-me ticks were `pursuit_mode` `direct`, with `odom_source` `gps`. Write a test before you trust trail mode.

## 3. The pack-low latch

Facts:

- `config.py` has `voltage_shutdown_threshold_v = 39.0`. Its comments say that the BMS cuts at about 37.7 V.
- The BMS pack undervoltage trip is 40.0 V and the cell trip is 3.000 V (`docs/battery_settings.md`, read from the Daly application on 2026-11-05).
- `_get_pack_voltage()` in `pi_app/hardware/vesc.py` returns the maximum of the two VESC readings. The right VESC reads about 0.9 V high. The function has no freshness check, so a reading that stops updating stays in use.
- Thus the BMS opens first. When it opens during a drive, drive and braking stop at once and the Pi halts 5 s later.

Recommendation from the investigation agent, with a Grok second opinion (session `01a1c654-e125-7882-a580-4500ff4b2ba2`). Not applied; Kevin decides:

1. Threshold 44.0 V (3.38 V for each cell). Worst case: 44.0 V, minus 0.1 V reading offset, minus 0.3 V for 30 s of drain, minus 2.6 V sag (40 A through 65 mΩ) gives 41.0 V at the BMS. A threshold of 43.0 V leaves no margin.
2. Use the minimum of the two readings, and only readings newer than 0.5 s. Grok adds: the two readings must agree within about 1.5 V; if they do not, refuse new throttle but do not latch.
3. Keep the +2.0 V release (46.0 V). Grok adds: release only with the throttle at zero or after a disarm.
4. Grok adds: limit the speed first, then stop. The latch commands RPM 0 with no slew limiter today, which is a hard stop from 1.6 m/s.
5. Grok adds: a second trigger on the BMS lowest cell (3.30 V), latched until a charger or a disarm.
6. Not agreed: the delay. The agent keeps 30 s. Grok wants 3 to 5 s. With the VESC limits of today (40 A for each side), a short delay can trip on hard acceleration at low charge. Keep 30 s until VESC part A is applied and the bus voltage logs show the real sag.
7. Apply VESC part A before this change or with it.

The data is thin: BMS samples only (0.1 V steps, one each 8 s), a maximum logged current of 5 A, and no bus-voltage logs near 44 V. The tests that pin 39.0 V and `max()` are in `pi_app/tests/test_vesc_telemetry.py`.

## 4. UPS battery incident, 2026-11-08 to 2026-11-10

The full record is in `docs/ups_hat_b.md`. Summary:

- Kevin switched main power off on 2026-11-08 at 19:33. The pack fell at once from 7.5 V to 6.85 V at −0.84 A. The halt was clean. The pack was flat 9 minutes later, after 6 poll wakes.
- On 2026-11-10 Kevin switched main power on at about 08:30. The Pi stayed in standby until he pressed its power button at 10:30.
- Thus a deliberate power-off of more than some minutes ends in a flat UPS pack, and the Pi does not start by itself afterward.
- Next check (Kevin): measure each UPS cell with a multimeter.

## 5. Not working or degraded

- RTK is fixed mostly near sunset (`docs/rtk_float_investigation.md`). Waypoint navigation and the GPS heading lock are not usable at midday.
- Hand gestures do not work as deployed. `pi_app/control/gesture_control.py` has not changed since 2026-09-20.
- Video recording is off while gestures are configured (`pi_app/hardware/oak_depth.py`, the condition `hand_queues is None`).
- The armed JSON log drops events that last one tick: follow-me entered and exited, mode changes, charger-inhibit changes. `should_log_tick` in `pi_app/app/log_gating.py` applies only the 0.1 s gate while armed. `test_log_gating.py` pins that behaviour.
- The BMS Bluetooth link sleeps for long periods.
- GPS does not recover when `wall-e` starts while I2C bus 1 is locked: `RtkGpsReader.detect()` runs one time at startup.
- `pi_app/cli/calibrate_imu.py` reads an attribute that `ImuReader` does not have. The tool collects no samples.

## 6. Waiting for Kevin

1. VESC part A in the phone application (`docs/battery_settings.md`, section 4). Not applied.
2. The PV-off test for RTK, and the 915 MHz listener board.
3. The regen stop measurement with the charge FET open.
4. Two decisions in `docs/depthai_upgrade.md`, section 6.
5. The multimeter check of the UPS cells (section 4).
6. The field validation of the GPS heading lock: Kevin will do it again later.
7. The east-field test plan (`docs/field_test_2026-11-03.md`): still wanted, but not now.

Kevin's rulings on 2026-11-10: the BMS targets are applied. The new UPS cells stay. WAYPOINT_NAV rolls through an intermediate waypoint with no stop (the code already does this).

## 7. Stale statements

Corrected in this commit: `CLAUDE.md`, `docs/NEXT_SESSION.md`, `docs/depthai_upgrade.md`, `docs/oak_d_lite_capability_audit.md`, `docs/rtk_float_investigation.md`, `docs/wifi_watchdog.md`, `docs/teleop.md`, `docs/bms_fail_open_memo.md`, `docs/backup_pump_design.md`, `docs/ups_hat_b.md`, `docs/i2c_bus_guard.md`, `docs/battery_settings.md`, `docs/performance_optimization_todo.md`, `docs/field_test_2026-11-03.md`, `docs/services.md`, `docs/pi_power_architecture.md`, `docs/gearing_memo.md`. In each case the code was the newer and correct side.

Not corrected, because the text is in a code file and a push of a code file restarts `wall-e`. Correct each one with the next change to that file:

- `config.py`: the stick expo comment gives the per-track formula that the review rejected (the code in `pi_app/control/mapping.py` is ratio-preserving); the `camera_fps` comment calls the value reserved, but it is in use; the pack-low comments say about 37.7 V.
- `pi_app/hardware/vesc.py` and `pi_app/control/controller.py`: the same pack-low numbers in comments.
- `bin/walle_ups.py` (docstring) and `bin/install_walle_ups.sh`: a 7.6 V setpoint; the value in the unit files is 7.5 V.
- `bin/i2c_bus_guard.py` (docstring): it names the UPS HAT (B) for the 2026-09-25 incident; that was the UPSPlus.
- `pi_app/web/waypoint_nav_ui.py`: the `/api/nav/go` docstring says that motion starts when the robot arms. It does not: the mode drops on the first disarmed tick.
- `docs/walle_one_sheet.html` still shows the UPSPlus HAT.

Housekeeping: the worktrees and branches in `docs/NEXT_SESSION.md`, section 4, item 8 (2026-11-10 note) have all their content in `main`. Remove them only with Kevin's approval.

## 8. Odometry on the drives of 2026-11-06 to 2026-11-08 (added 2026-11-10)

Kevin asked how the odometry worked on the last drives. One read-only agent examined the armed logs from 2026-11-04 to 2026-11-08. The main claims were then checked against the code and the logs. Nothing was changed on the robot or in the code.

The drives: 2026-11-06 17:21, 2026-11-07 12:17 and 17:59, 2026-11-08 12:36, 12:41 and 14:25 (MANUAL and FOLLOW_ME, no WAYPOINT_NAV, 80 to 130 m each). 2026-11-04 was a stand test: the wheels turned, but the GPS position stayed in 0.2 m.

### 8.1 Defect, confirmed: GPS odometry has the wrong frame for its users

- `GpsOdometry` (`pi_app/control/gps_odometry.py`) keeps the heading as a math angle (0 = east, counter-clockwise positive). Its `camera_to_world` and `PurePursuitController.compute` (`pi_app/control/pure_pursuit.py`) use `(-sin, cos)` as the vector to the right of the robot. In a math-angle frame that vector points to the left.
- Check with the classes of the repository: put a trail point 3 m ahead and 1 m to the right, then pivot the robot 30 degrees to the right. The point is then to the left. Dead-reckoning odometry gives a steer of −1.8 bytes (left, correct). GPS odometry gives +6.6 bytes (right, the wrong direction). A pivot to the left is wrong also.
- `update_gyro` writes the gyro heading into the heading on each tick. Thus the course-over-ground correction in `update_gps` stays for one tick only. In the logs, `odom_theta_deg` is equal to 90 minus `imu.heading_deg` (median difference 0.00 degrees). The gyro heading is relative to the orientation at boot. The agent measured 59 to 78 degrees between that frame and the GPS track on these drives (not checked again). The positions are true GPS positions, so the heading and the positions do not agree.
- No test covers `GpsOdometry`. The file has not changed since 2026-03-15.

### 8.2 The users of that frame

FOLLOW_ME is an autonomous mode. `_pick_odometry` selects GPS odometry when the GPS has a fix of quality 1 or better, thus always outdoors.

1. Trail pursuit, when the person is more than 5.2 m ahead (`direct_pursuit_distance_m` 4.0, multiplied by 1.3).
2. Blind trail pursuit, for a maximum of 3 s after the target is lost: forward motion, with a steer limit of 32 bytes (approximately 22 degrees for each second).
3. The search pivot after that. It takes its direction from the trail tangent in the same frame.

These paths did not run. The 55 armed logs on the robot have 3,309 ticks with `pursuit_mode` `direct` and no tick with `trail` or `search`. On the follow-me drives the target distance was 3.0 m at most, and the target was not lost for longer than 0.13 s. Thus the defect has had no effect in the field. It also means that trail following has no field validation after the heading sign correction of 2026-09-19.

### 8.3 Other findings

- Wheel speed is approximately 20 to 25 percent more than the GPS ground speed on straight segments (medians 1.19 to 1.26 for each drive, RTK float, both speeds above 0.8 m/s). The comment at `trail_speed_scale_mps_per_byte` in `config.py` records the same difference from March. Slip and a scale error are not separated. If it is a scale error, the top speed of "1.61 m/s" is approximately 1.3 m/s on the ground. The follow-me speed loop uses the wheel speed as the true speed.
- The GPS heading lock did not engage on any drive (`heading_align.locked` is false on each row). It needs RTK fixed while the robot moves. RTK fixed occurred only while the robot was parked (2026-11-07).
- The agent measured a change of approximately 15 degrees between the gyro heading and the GPS course in 108 minutes on 2026-11-08. The scatter at RTK float is approximately 20 degrees for each point, so this number is weak.
- GPS odometry gets a new position one time each second and holds it between fixes. It accepts each fix of quality 1 or better. A change of fix quality moved the position 1.3 to 2.1 m while the robot was parked.
- Camera odometry (VIO) is in no drive log, because `OakDetectionConfig.vio_enabled` is False. The probe files of 2026-11-06 are on the robot (`logs/vio_probe_20261106_*.csv`).

### 8.4 Recommendation (APPLIED 2026-11-10)

APPLIED 2026-11-10: Kevin said "Yes, turn it off". `FollowMeConfig.trail_follow_enabled` is False in `config.py`. Checks before the push: the full test suite gave the same result as before (1,880 passed, and the one known failure on the Mac for `flask-sock`); a replay of 2,309 real follow-me frames from three drives gave the same motor bytes with the switch on and off; Grok reviewed the diff (session `01a1c719-b3ab-7b62-a30f-534496344ff6`: safe, no item to correct). Grok's notes: if the person was near the centre of the frame when the target was lost (`|_last_target_x|` 0.05 or less), the robot stays still for the 3 s instead of a pivot; the status shows `SEARCH_ROTATE` during that time. The repair of `GpsOdometry` stays open. The text of the recommendation follows.

Set `trail_follow_enabled` to False until `GpsOdometry` is repaired, has tests, and has a field test. With the switch off, the trail and the two odometry objects are not made. FOLLOW_ME then uses direct pursuit, which is the only mode in the logs. A lost target then gives a pivot with zero forward motion toward the last camera position of the person, until 3 s after the last valid detection, then a stop.

Grok (session `01a1c70b-2a64-7190-838b-976e2fbe3992`) agrees with this step. It rejects dead-reckoning odometry as the interim step, because that turns on a forward path that has no field validation. Its caution: the search direction must not come from GPS odometry. The code agrees with that when the switch is off (`_trail` is None, so the search uses `_last_target_x`).

Proposed position in section 2: second, after the person stop. The trigger is a usual event: the operator goes out of view, or gets more than 5.2 m ahead. This is a motion change. It needs a Grok review of the diff and Kevin's approval before the push, and the push restarts `wall-e`.

The cheapest test for the wheel scale, the gyro drift and the heading lock together: one straight MANUAL run of approximately 20 m along a tape at RTK fixed.

## 9. Corrections of 2026-11-10 (afternoon)

Kevin said that there were tokens to use. Agents prepared the corrections in separate worktrees. Each correction got a review, a Grok safety review and the full test suite. Kevin approved the push of the five corrections in 9.1.

### 9.1 Deployed

Each commit was pushed alone. The next commit was pushed only after the robot had the previous one (ancestry check, service active, no traceback in the journal).

| Commit | On the robot | Change |
|---|---|---|
| `2ebdace` | 16:25 | Log: while armed, a tick with an event always writes a line. Before, events on ticks that the 10 Hz gate skipped were lost. |
| `eea14b5` | 16:30 | Item 2. `/api/nav/start` and `/api/nav/go` clamp the speed to 10–80 and the radius to 0.3–3.0, and refuse a speed of 0 or less. The Grok review found one more defect: the routes replaced the route before they asked the gate, so a refused start changed a mission in progress. Now the gate is asked first, and a late refusal restores the mission (`snapshot_mission`). Grok sessions `01a1c759`, `01a1c760` (not safe), `01a1c765` (safe). |
| `cd6c9ee` | 16:35 | Item 5. The RC reader stamps each frame with the monotonic clock (`last_update_mono_s`). The phone override file has the monotonic stamp and the boot ID, because `/tmp` can keep a command from an earlier boot. Grok session `01a1c778`. |
| `8bfbfb5` | 16:40 | Item 3. The start is refused with no GPS reading, a stale reading, no heading, a stale heading (0.5 s) or no lock. Each tick of a run checks the heading, and one ALIGN episode stops after 10 s. Grok session `01a1c779`. |
| `949c0b4` | 16:55 | Item 4. WAYPOINT_NAV leaves on a stale detection stream, as FOLLOW_ME does. Same Grok session. |

Checks on the robot: the waypoint page shows the 10–80 range, a negative speed gets HTTP 400, and a start indoors is refused (`gps_quality_not_trusted`).

### 9.2 Not verified

- Arming with the transmitter on. Commit `cd6c9ee` changes the RC path, and the transmitter was off during the deploys. Test an arm and a disarm before the next drive.
- A loss of the heading lock, a stale heading or a colour stall during a real waypoint run.
- The measured IMU sample age was 0.142 s at most on two drives, against the 0.5 s limit. A waypoint run with a higher load is not measured.

### 9.3 New gap: the IMU stream can stop without a fault

- On 2026-11-06 at 19:34:37 the OAK IMU stream stopped in a healthy session (process start 17:54:30). Depth and detections continued, the reconnect count stayed at 0, and the IMU queue counters did not change again. The journal had one line: `OAK IMU samples stale (age=0.511s > 0.500s): heading frozen`.
- The heading stayed frozen for 38.8 h. The two drives of 2026-11-07 had `integrate_status` `stale` and a heading of 0.000 on each tick (968 ticks with motion). The heading hold gave no correction, because the heading and the target were the same. If the stream stops during a turn, the hold keeps a constant correction of 35 bytes at most.
- A colour-stall session rebuild on 2026-11-08 at 10:21:56 brought the IMU back in the same process. The cause in the camera is not known.
- Shorter stale episodes that recovered are in the journal on other days (most on 2026-10-22 to 2026-10-24, approximately one each 4.4 s).
- Correction DEPLOYED on 2026-11-10 with Kevin's approval, one commit for each push: `185f239` (on the robot at 17:05) rebuilds the session when no IMU message arrives for 10 s with fresh depth (one restart budget for the colour watchdog and the IMU watchdog: 3 in 15 min, then a latched `imu_stall_fault`); `67b3f04` (17:10) gives no heading-hold correction from a stale heading and captures the target one time on recovery; `07cbc74` (17:15) adds `imu_age_s`, `imu_stale` and `imu_watchdog` to the health data, a journal line each minute, a status-line text (`IMU STALE`) and JSON events (`oak_imu_stall`, `oak_imu_stall_restart`, `oak_imu_stall_fault`, `oak_imu_recovered`). Grok sessions `01a1c798` and `01a1c79a`: safe, no item to correct. Checks on the robot: no IMU restart after the warm-up, and `/api/telemetry` shows `camera_health.imu_age_s` 0.068, `imu_stale` False, `imu_watchdog` True.
- Not verified, because only a real stall shows it: that a session rebuild brings a dead IMU stream back (one case, 2026-11-08), the rebuild time, MANUAL at the 10 percent floor during the rebuild, and the depthai teardown on this restart path. The journal now has a "resumed after N s" line for stale episodes of 2 s or more; use it to check the 10 s limit.

### 9.4 Item 1, the person stop hold: deployed at 17:50

DEPLOYED 2026-11-10 17:50 with Kevin's approval: commit `c999b0a` (`pi_app/control/person_stop_hold.py`, wiring in `oak_depth.py`, `main.py` and `controller.py`, config values `person_stop_hold_*`). The fifth Grok review (session `01a1c7ce`) found it safe. The full suite passed on the commit (2,093 tests, and the one known failure on the Mac).

How it works, in FOLLOW_ME and WAYPOINT_NAV:

- A stop trigger engages the hold. A trigger in the last 1.0 s also engages it at the start of an autonomous mode. While the hold is active the obstacle distance is 0.0, so the forward part of the command is zero; pivots and reverse pass.
- The hold follows one object, the one that caused the stop (the chain). On each poll with a fresh detection packet, a box continues the chain only if it is the one candidate: same group (person or animal), centre within 0.30 of the frame width, seen in 0.6 s, and plausible in range (1.5 m/s and 0.25 m) and in area. One box in the same place (0.10) continues the chain with a different label (YOLO called the operator "horse" at close range).
- Release by evidence: the chain's own box seen beyond 1.0 m (range "ok", 0.05 from the left and right edge) on 3 polls, then 5 polls in sequence with each stop-tier box in view seen beyond. The corridor is not evidence (on 2026-11-08 it read 2.9 to 4.7 m with Kevin at 0.3 to 0.9 m).
- The hold becomes operator-only, until it ends, when the chain is lost for more than 0.6 s, when two boxes are candidates, when a second object is not seen beyond on 2 polls, when a far box is in the chain's place while the chain is missing, or when a second object triggers the stop.
- The operator ends a hold with the arm switch off (live RC link), with FOLLOW_ME off and on, or with stop and GO on the waypoint page. A hold that is active at a change to MANUAL (also the automatic exit on stale vision) stays until the two commanded tracks are at neutral for 0.25 s.
- MANUAL has no new hold by default (`person_stop_hold_in_manual` False). Do not set it to True without Kevin's word.
- If the control loop does not write its levels for 1 s, the reader uses the strict values and writes one WARNING. At each start the journal shows `levels never written` and then `levels fresh again`; that is normal.

Replay of three drives (log lines at 10 Hz as polls): three holds, all in FOLLOW_ME. The hold of 2026-11-08 14:27:39.9 covers the 0.66 s gap; it became operator-only (a second trigger box at 14:27:41) and ended when Kevin left FOLLOW_ME. The other two released on the object seen beyond; the start of following was 0.3 to 0.7 s later than before.

Checks on the robot after the deploy: no traceback, the hold state is in the JSON log (`obstacle.person_stop_hold`, inactive), detection latency p50 0.196 s and depth 14.5 fps (the same as the runs before it).

Not verified: the hold on the robot with a person (it needs a supervised FOLLOW_ME test: step inside 0.8 m, step to the side, walk away), the timing at 15 polls each second, and how frequently the hold becomes operator-only in the yard. The journal lines `Person stop HOLD engaged`, `released` and `operator-only (reason)` and the log field show each case.

History of the reviews:

- Three versions failed the Grok safety review (sessions `01a1c76a`, `01a1c783`, `01a1c79d`). The first version released on a corridor reading after an inset box, ended at an automatic change to MANUAL, and lost a trigger at a mode change. The later versions followed objects in a list, and the list let a different object release the hold; it also had no limit on its size.
- Rulings kept from the review: no new hold in MANUAL by default (`person_stop_hold_in_manual` False). Reason: with the netting over the lens the corridor does not show clear, so a MANUAL hold would stay until the arm switch is cycled each time the camera sees Kevin near the robot. A hold that is active at a change to MANUAL stays until the sticks are at neutral for 0.25 s. A trigger in the last 1.0 s engages the hold at the start of an autonomous mode.
- The fourth version (one chain, no list; session `01a1c7bd`) still released on a clear corridor after the object was seen beyond and then left the view. A person can come back inside 0.8 m in that time with no detection. The deployed version has no corridor release in the autonomous modes.

### 9.5 GPS odometry repair: prepared, not deployed

Branch `worktree-agent-a51d633112c7c0737`. One frame contract for the two odometry classes and each user, dead reckoning as the default source of the trail, and tests. With `trail_follow_enabled` False the motor commands are the same as on `main` (2,309 replay frames; Grok session `01a1c77e`). It needs a rebase on `main`. The field checklist in the agent's report applies before the switch goes on: a stall check for dead reckoning, a straight run on a tape, pursuit beyond 5.2 m and a lost target in an empty area.

## 10. Technical Names

boot ID, monotonic clock, worktree, odometry, dead reckoning, pure pursuit, trail pursuit, math angle, course over ground, eRPM, VIO, Grok, VESC, BMS, Daly, OAK, RTK, GPS, IMU, UPS, HAT, RTC, NTP, I2C, RC, PV, FOLLOW_ME, WAYPOINT_NAV, MANUAL, ALIGN, `wall-e`, `main`, `flask-sock`, FET, e-stop, slew limiter, latch, taper, sag, worktree, endpoint, clamp, standby, multimeter, pivot, corridor, backstop.
