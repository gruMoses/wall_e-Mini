# Status audit, 2026-11-25

This document replaces `docs/status_audit_2026-11-10.md` as the document to read before you plan work. The old audit stays as the history of the corrections of 2026-11-10.

Method: one read-only pass. The code on `main` at `eb4a9af` (the last code change is `101aecf`, 2026-11-16), the git history since 2026-11-10, the full test suite on the Mac, and the robot over SSH (read-only: git, journal, log files). No code changed. Nothing was sent to the robot. No second-model review was made of this audit.

WARNING: The safety rules in `CLAUDE.md` hold whatever this document says. Ask Kevin if the wheels are blocked before each command that can move them.

## 0. Deployed later on 2026-11-25 (Kevin's decisions, each Grok-reviewed, full suite green)

1. Section 2 item 6, hard-stop turn: FIXED, `574e19c` + `f40bef0`, all modes. Forward stops at once; the turn ramps; no track crosses 126 in one tick. Live 10:25.
2. Section 2 item 2 / N1, charger with the robot armed: FIXED for a running charge, `9f4b1c4`..`0ae2af1`. A charge running at the arm goes on and holds the robot; tap PLUG OFF to drive. No charge starts while armed (PLUG ON included). Remaining limit (plug off, lead on): `docs/charger_plug.md`. Also fixed: a failed OFF no longer re-arms the plug countdown (that bug was on main). Live 10:30.
3. Section 2 item 4, OAK missing at boot: FIXED, `a9c6fc2`..`d028d1d`. The reader waits for the camera; MANUAL at the 10 % floor until each camera session's first detection packet, and whenever person detections are stale (more than 1.5 s), even on fresh depth (Kevin). Live 10:35; first packet 4 s after start.
4. Arming with the transmitter on: closed by Kevin (`abd3732`).
5. Phone app over Tailscale HTTPS (`docs/phone_app_tailscale.md`), app nav and battery percent `777d754`, `c68c370`. Live 10:40.

## 0a. Found overnight 2026-11-25/26 (robot in the garage, disarmed)

1. NEW GAP, depth stream dead with the session alive. depthai logged `no request for this response: XLINK_READ_REL_RESP` at 20:51:38 and the depth stream stopped; detections, RGB and the IMU kept running (`depth_age_s` 42,324 s at 08:36, `depth_fps` 0, `vision_hung` False). No watchdog covered it: the colour watchdog watches NN packets, the IMU watchdog IMU messages. MANUAL sat on the 10 % stale floor for 11.8 h and FOLLOW_ME / WAYPOINT_NAV were refused, until a disarmed `wall-e` restart at 08:39 (Kevin's OK) brought depth back at 15 fps. The same error killed depth on 2026-11-22 08:43 for 25.6 h (until the 11-23 10:18 restart). Since 11-15 it fired 5 times: 3 times on the IMU stream (the IMU watchdog rebuilt the session in about 8 s) and 2 times on depth. Fix: a depth stall watchdog (`depth_stall_*` in `OakDetectionConfig`, `OakDepthReader._depth_stall_after_poll`, `pi_app/tests/test_oak_depth_stall.py`). It ends the session after 5 s without a depth frame while detections are fresh, shares the 3-per-15-min budget, waits (does not latch) when other watchdogs filled the budget, and gives up after 3 rebuilds in a row that did not bring 60 s of continuous depth. Five Grok passes. Status: see the commit; not yet seen on a real stall.
2. Auto-disarm worked: armed and idle 600 s, disarmed at 17:54:11.
3. Charge hold worked all night: 5 cycles from 3.700 V per cell rested to 3.83 V under charge, about 16 min each, every 3 h. At 08:07:03 the BMS reported -304.7 A for one poll (a bad BLE frame). That broke the charging streak, so `is_charging()` read False; the plug inhibit held the robot until PLUG OFF at 08:07:16, and the current read -0.3 A at 08:07:21. The plug inhibit (`0ae2af1`) covered exactly this case. Open, low: reject an implausible BMS current instead of letting it reset the streak.
4. RTK: no corrections in the garage all night (14.8 h at quality 2, correction age 0). The TinySA in the garage saw the base's bursts at about -89 dBm in the same evening (`docs/rtk_float_investigation.md` section 2.3c).

## 1. Summary

1. **Highest open gap: the Arduino still runs the old firmware.** By its code, a channel that loses its pulse keeps its last value. If the receiver loses power or a lead comes off, the Pi sees the last stick, arm and e-stop values with no end. The corrected sketch is in the repo and is not flashed (section 2, item 1).
2. **New gap: the charger plug makes the charger inhibit blind most of the day.** In `hold` the plug is off about 22 hours of 24, with the lead on the robot. An arm during a charge switches the plug off, and the inhibit releases about 20 s later (section 3, item N1).
3. Of the eight gaps of 2026-11-10: items 1 to 6 are fixed in the code and are on the robot. Item 7 (charger inhibit) is open by Kevin's decision and is now wider. Item 8 is obsolete (the code was deleted).
4. Most of the new protections have not seen their real event on the robot (section 4). The drive of 2026-11-24 gave the first field evidence for arming and for the person stop hold.
5. Tests: 2,782 pass and 6 skip on the Mac (243 s). No failure.
6. Robot at 05:54 on 2026-11-25: `wall-e` on `eb4a9af`, no `.deploy_hold`, services `wall-e`, `walle-power`, `i2c-bus-guard`, `wifi-watchdog`, `walle-clock-floor` active, `walle-ups` gone, charger mode `hold`, no charger fault, up 1 day 19 h.

## 2. Open safety gaps, in priority order

Each correction is a motion-safety change. Get a Grok safety review and Kevin's approval before each push. One correction for each push.

1. **HIGH. The flashed Arduino firmware holds the last pulse width.** The flashed image is `dcc550e`. Nothing in it times out a channel (`docs/rc_arduino_gps_wiring.md`, section 9, finding 1). The serial lines continue, so the RC-stale watchdog does not see it. Effect: a receiver that loses power, or a CH1/CH2/CH3/CH5 lead that comes off, leaves the robot armed on the last stick values, and the e-stop channel is frozen also. The only stop is the main switch. Not measured on the robot (check 7 in that document is open). The correction is complete on the Pi side and is deployed (`fccfdfe`, `1681f39`, `cf78a73`, `903d897`): a no-signal value on CH3 or CH5 disarms, a dead track lead makes the two tracks neutral. It does nothing until the new sketch sends the no-signal value.
   WARNING: Flash only with `wall-e` stopped and with Kevin's word. Stopping the service needs his go-ahead in the same turn.
2. **MEDIUM. The charger inhibit does not see a connected charger.** Old item 7, now wider. Refer to section 3, item N1. Kevin ruled out the charger latch and the "charger unplugged" button; this audit does not propose them.
3. **MEDIUM. Web endpoints accept commands from each device on the network.** No authentication and no local-only check on `POST /api/follow_me` (`pi_app/web/oak_viewer.py:2654`), `/api/follow_me/params` (2728), `/api/manual/netting_mute` (2677), `/api/nav/*` (`pi_app/web/waypoint_nav_ui.py:2626`, 2766), and the new `/api/charger/mode` and `/api/charger/clear_fault` (2620, 2633). A request can start FOLLOW_ME when the robot is armed with a target in view; the CH4 latch does not block a web start (`3b92e4e`: "operator acts"). The robot must be armed on the RC first, and the waypoint values are clamped.
4. **MEDIUM. OAK not found at startup: MANUAL has no person stop.** `OakDepthReader.detect()` runs one time (`pi_app/app/main.py:201`). With no camera there is no obstacle controller, no 0.8 m stop and no IMU until the next restart. By the code, FOLLOW_ME has no controller and no target, and WAYPOINT_NAV is refused for the missing heading (`heading_unavailable`, `controller.py:841`). The vision-stale gate is not active with no camera (`controller.py:632-637`), so the heading check is the only guard there. Not tested. A camera that fails after startup gets the 10 percent floor, so the two cases are not consistent. Kevin's decision is pending (section 5, item 4).
5. **MEDIUM. The discharge-FET guard and the charger inhibit use old BMS data.** The FET guard acts on the last polled value with no age check (`pi_app/app/main.py:981-1013`); since `da40080` it only prints one warning. The inhibit fails open after 30 s without a poll (`pi_app/hardware/bms.py:262`). In `hold` the BMS stays awake on the idle load, so a dark link is now less usual.
6. **LOW to MEDIUM. A hard stop passes the turn part of the command with no slew.** When the obstacle scale is 0 and the command is forward, the slew limiter is bypassed (`controller.py:2949`, 2973-2983). The forward part is zero, but the difference between the tracks goes to the motors in one tick. In MANUAL, a stick held forward and to one side at the 0.8 m stop becomes a pivot at once. This code is from April; it is not a change since 2026-11-10. Kevin accepted the pivot during the stop (2026-11-08); the missing ramp was not part of that ruling.
7. **LOW. A failed boot gravity check only logs a warning** (`docs/depthai_upgrade.md`, section 6). Decision pending.
8. **LOW. Pack guard, known limits.** A voltage reader that is stuck high in the plausible range gives the restricted state (autonomous modes stop, MANUAL at 32 bytes), not a latch (accepted 2026-11-11, comment in `config.py`). The delay is still 30 s (`config.py:261`); no bus log near 44 V exists.
9. **LOW. An e-stop switch that is high at boot powers the Pi off about 5 s after the start.** With the X1202 the Pi then stays off until the input is off for 30 s.
10. **LOW. A push that changes `bin/` does not reinstall the root services.** `walle-power` runs from `/usr/local/lib/walle-power`; reinstall it by hand (`bin/install_walle_power.sh`).
11. Accepted residual (no action): with the transmitter off, the receiver failsafe reads as switch-off on CH3, so the robot arms again when the transmitter returns with the switch up.

## 3. New since 2026-11-10

### N1. Charger plug and charger inhibit (new, MEDIUM)

- The inhibit needs the charge FET on and more than 2.0 A for 2 polls (`config.py:1542-1543`). With the plug off there is no current, so there is no inhibit.
- In `hold` the plug is on about 17 minutes, six times a day. Journal since 2026-11-16: 52 `Charger plug ON`, 57 `charger_inhibit ENGAGED`. The inhibit works during a charge, 13 to 21 s after the plug goes on.
- The manager stops a charge when the robot is armed (`pi_app/control/charge_policy.py:383`; `main.py:978` gives it the armed state). The plug goes off in 10 s or less, the current falls, and the inhibit releases at the next BMS poll. Thus an arm on the charger removes the protection that the charge gave.
- Effect: the robot can drive away with the charger lead connected, in MANUAL or in FOLLOW_ME. Damage to the lead, the charger or the socket; a pulled cable near a person.
- The slew correction (`cce9b06`) makes the release ramp from neutral. That part is correct.
- `docs/charger_plug.md` says "Armed: no charge starts and a running one stops". It does not say what this does to the inhibit.

### N2. Charger plug, limits that a restart clears (new, LOW)

- The state file keeps the mode, the fault, the last full charge and the plug address only (`pi_app/control/charge_manager.py:165`). The counts of wake pulses and rescue charges for each day, the "does not hold the charge" block, the 30 minutes between hold charges and the amp-hour count of the charge in progress start again at each `wall-e` restart (each deploy).
- A crash loop cannot charge blind: no action occurs in the first 2 minutes without a BMS reading, and the first wake pulse waits 10 minutes.
- A hold charge that a restart interrupts is still limited by the cell voltage under charge and by the dead-man timer in the plug.

### N3. Charger plug, mode `on` (new, LOW)

- `on` has no dead-man timer, no armed rule and no temperature inhibit (`charge_policy.py:375-382`). A latched fault still holds the plug off. The mode is set by a request with no authentication (item 3 in section 2). The BMS and the charger's 54.6 V limit are the guards.

### N4. Observations, not gaps

- The plug did not answer for long periods: 37 min on 2026-11-20, 29 min and 3.7 h on 2026-11-22, 8.4 h on 2026-11-24. Its address changed between `192.168.84.100` and `192.168.84.20`. During that time the hold cannot top up. The dead-man timer covers a plug that was on.
- From 2026-11-12 to 2026-11-15 `wall-e` restarted 8,092 times with "Arduino RC not detected" (the Nano was off the robot for the shield work). Not a fault of today.
- Wake pulses and rescue charges: none in the journal. No charger fault has latched.

## 4. Deployed, not verified on the robot

The only armed logs on the robot since 2026-11-10 are seven of 2026-11-24 (MANUAL and FOLLOW_ME, no WAYPOINT_NAV).

1. **Arming with the transmitter on** (after `cd6c9ee`). Evidence: the stand on 2026-11-15 (three arms, 326 s), and the field on 2026-11-24 (arm, drive, FOLLOW_ME, re-arm after the auto-disarm at 16:37). Kevin has not closed the item (section 5, item 3).
2. **Person stop hold** (`c999b0a`). Field evidence, 2026-11-24: engaged at 16:23:16 (carry-in) and 16:26:50 (trigger), released on `seen_beyond` after 1 s and 3 s. Not seen: the operator-only state (0 lines in the journal), a hold carried into MANUAL. Not done: the supervised test (step inside 0.8 m, step to the side, walk away).
3. **IMU-stall watchdog** (`185f239`, `67b3f04`, `07cbc74`). No stall event in the journal since the deploy. A rebuild on a real stall is not seen.
4. **Pack guard** (`2c193c1` to `f0104d7`). No real trip. The restricted state appeared only while the power went down (2026-11-13, 2026-11-15). Not seen: the trip ramp, limp home, the stop of an autonomous mode.
5. **WAYPOINT_NAV gates** (`eea14b5`, `8bfbfb5`, `949c0b4`). No waypoint run since 2026-11-10.
6. **RC-stale re-arm latch and the CH4 latch on each FOLLOW_ME exit** (`3b92e4e`, 2026-11-14). No RC-stale event in the journal. The stand test of 2026-11-11 was before this change.
7. **RC no-signal reactions.** They need the new sketch (section 2, item 1).
8. **Slew ramp after a stop release** (`cce9b06`). No direct check on the robot.
9. **Charger plug**: the dead-man timer was tested on the plug (2026-11-16). The health faults, the wake pulse and the rescue did not run.
10. **Auto-disarm**: worked on 2026-11-24 at 16:37 (600 s idle).
11. **`walle-power`**: bench tests passed; a real input-loss halt on 2026-11-23 at 10:18 is in `/var/lib/walle-power/events.jsonl`. Open: the plug retention on the barrel jack, the warm-cell load test before summer 2027.

## 5. Decisions waiting for Kevin

1. **Flash the new Arduino sketch.** Recommendation: yes, before the next drive away from the stand. Do the bench checks of `docs/rc_arduino_gps_wiring.md` section 9.1 at 333 Hz. It needs `wall-e` stopped.
2. **Charger inhibit with the plug (N1).** Choices: (a) accept it as a known limit and write it in `CLAUDE.md`; (b) also let a charge in progress continue when the robot arms, so the inhibit stays while current flows (no charge starts while armed, as today). Recommendation: (b), then (a) for the rest. It needs a Grok review.
3. **Close "arming with the transmitter on".** Recommendation: close it; stand and field evidence agree.
4. **OAK missing at startup.** Recommendation: start the camera reader anyway and let its supervisor wait, so MANUAL gets the 10 percent floor as in the other case.
5. **Failed gravity check.** Recommendation: it disables heading hold and waypoint navigation until a restart (Grok, 2026-11-05).
6. **Supervised test of the person stop hold.** Recommendation: do it at the next drive; it is short. Kevin starts it.
7. **Pack guard stand test** (trip ramp, limp, autonomous stop) and the 30 s delay. Recommendation: test on the stand; keep 30 s until a bus log near 44 V exists.
8. **Web endpoints (section 2, item 3).** Recommendation: accept on the home network for now; make the charger and follow-me POST routes refuse a peer that is not on the local subnets when that work is convenient.
9. **Barrel plug retention on the X1202.** The condition "before the first drive" is in `CLAUDE.md`. The robot drove on 2026-11-24. Say if it is done.
10. **Hard-stop turn with no slew (section 2, item 6).** Recommendation: keep the immediate stop of the forward part, and ramp the turn part in the autonomous modes at least. Ask Grok first.
11. Still parked by Kevin, no action: the regen stop measurement with the charge FET open, the field validation of the GPS heading lock, the east-field test plan, trail following (stays off).

## 6. Status of each item of the old audit

| Old item | Status | Evidence |
|---|---|---|
| 2.1 Person stop releases at close range | FIXED, partly verified | `c999b0a`; `pi_app/control/person_stop_hold.py`; `config.py:617` (enabled), `:626` (`person_stop_hold_in_manual` False), `:630` (1.0 m) |
| 2.2 Waypoint API accepts any speed | FIXED | `eea14b5`; `waypoint_nav_ui.py:2399` (`_parse_nav_params`), `:2459` (`snapshot_mission`) |
| 2.3 WAYPOINT_NAV heading | FIXED, not verified | `8bfbfb5`; `controller.py:856-892` (start gate), `:916-957` (each tick, ALIGN 10 s) |
| 2.4 WAYPOINT_NAV stale detections | FIXED, not verified | `949c0b4`; `controller.py:891`, `:936` |
| 2.5 RC-stale watchdog on the wall clock | FIXED | `cd6c9ee`; `controller.py:58`, `:2271`. Added `3b92e4e`: a stale link sets the re-arm latch (`:2285`) |
| 2.6 and 3. Pack-low latch cannot trip | FIXED, not verified | `2c193c1`, `75072a7`, `4b0f4d4`; `config.py:245` (44.0 V), `:375` (limp on), `:382` (42.0 V); `pi_app/hardware/vesc.py` |
| 2.7 Charger inhibit blind | OPEN by Kevin's decision, wider | section 3, N1. Case (b) is less usual: the BMS stays awake in `hold` |
| 2.8 UPS watcher does not retry a power-off | OBSOLETE | `bin/walle_ups.py` deleted (`8037d3d`). `bin/walle_power.py` has a force ladder (30 s, 60 s) |
| OAK not found at startup | OPEN | `main.py:201` |
| Gravity check only warns | OPEN | `docs/depthai_upgrade.md` section 6 |
| E-stop high at boot | OPEN, changed | no RTC poll now; the X1202 stay-off rule applies |
| Blind trail pursuit floor | INACTIVE | `config.py:1068` `trail_follow_enabled` False |
| Web endpoints with no authentication | OPEN, two more routes | section 2, item 3 |
| `bin/` push does not reinstall services | OPEN | by design; `CLAUDE.md` says so |
| GPS odometry frame defect | FIXED as inert code | `cd9086e`, `be768cc`; trail following off |
| 4. UPS battery incident | OBSOLETE | the X1202 and `walle-power` replaced the UPS HAT (B), go on 2026-11-15 |
| 5. Armed log dropped events | FIXED | `2ebdace` |
| 5. GPS not found when the bus was locked at start | FIXED, on main | `b8cfd11` |
| 5. `calibrate_imu.py` | FIXED, on main | `99db43d` |
| 5. RTK fixed mostly near sunset | OPEN | three fixes parked in the open on 2026-11-24 (`docs/rtk_float_investigation.md`) |
| 5. Hand gestures | OPEN | `gesture_control.py` last changed 2026-09-20 |
| 5. Video recording off with gestures on | OPEN | `pi_app/hardware/oak_depth.py:2783` |
| 5. BMS link sleeps | BETTER | in `hold` the idle load keeps it awake |
| 6.1 VESC part A | DONE | applied over CAN on 2026-11-11 (`886bbb6`, `docs/battery_settings.md:99`) |
| 6.5 UPS cell check | OBSOLETE | board replaced |
| 7. Stale comments in code files | FIXED, on main | `d439ee3` |
| 9.2 Arming with the transmitter on | Evidence, not closed | section 4, item 1 |
| 9.3 IMU stall | FIXED, not verified | section 4, item 3 |
| 9.6 Slew after a stop release | FIXED | `cce9b06` |
| 9.6 Wheel ground scale | DONE | `962454a`, `1e3a26f`; `config.py:432` (0.815) |
| 9.6 Charger latch, ramp on the first command | RULED OUT by Kevin | do not propose again |

## 7. Stale statements

Where a document and the code disagree, the code is correct in each case below. This audit changed no other file.

`CLAUDE.md` against the code:

1. Line 95 and the pack-low item (line 36) point to `docs/status_audit_2026-11-10.md`. Point them to this document. The pack-low item can go: 44.0 V is in `config.py:245` and on the robot.
2. "Arming with the transmitter on is not tested": stand and field evidence exist (section 4, item 1). Kevin's call.
3. "Not yet tested with a person on the robot" (person stop hold): the hold engaged and released two times in the field on 2026-11-24. The supervised test is still open.
4. `CLAUDE.md` does not say that an arm during a charge switches the plug off and releases the inhibit (N1). Add it when Kevin has decided.
5. `CLAUDE.md` does not name the re-arm latch after an RC-stale link, or the CH4 held-low rule after each FOLLOW_ME exit (`3b92e4e`). Both are in the code.
6. "Before the first drive: hold the barrel plug in the jack": the robot drove on 2026-11-24. The status is not recorded.
7. Every setting that `CLAUDE.md` names agrees with `config.py`: `camera_fps` 15.0, `max_usb_speed` "HIGH", `imu_source` "oak_d", `oak_yaw_rate_scale` 1.0, `trail_follow_enabled` False, `person_stop_hold_in_manual` False, `safety_stop_radius_m` 0.8, `voltage_shutdown_threshold_v` 44.0, `wheel_ground_scale` 0.815, `snap_first_command` True.

Other documents:

8. `docs/rc_arduino_gps_wiring.md`, section 9: "fixed in code, not deployed ... on a branch, not on `main`". The Pi side is on `main` and on the robot. Only the sketch is not flashed.
9. The same document, section 8: "the UPS HAT (B) INA219 at 0x42 on the robot, and the X1202 ... on the bench". The X1202 is on the robot; the UPS HAT (B) is retired.
10. `docs/charger_plug.md`, "Other behaviour": "A hard power loss of the Pi mid-charge leaves the plug on". In `hold` and `full` the dead-man timer in the plug ends the charge in 5 minutes or less. The sentence is true for mode `on` only.
11. `docs/status_audit_2026-11-10.md`: section 1 says "None is fixed", section 3 says the latch cannot trip, section 6 says VESC part A is not applied, section 9.6 says the pack-low latch is not deployed. All four are out of date.
12. `docs/NEXT_SESSION.md`: the first paragraph sends the reader to the old audit and says VESC part A is not applied.
13. `docs/known-issues-history.md` (lines 5 and 18) and `docs/fuse_plan_2026-11-15.md` point to the old audit.
14. `docs/walle_one_sheet.html` still shows the UPSPlus HAT.
15. Memory note `project-prepared-fixes-20261110.md`: "RC dropout disarms and re-arms by itself" (stand test, 2026-11-11). Since `3b92e4e` a stale link needs the arm switch low for 0.2 s, then high.
16. The remote branch `origin/fm-pump` is still listed. The record of 2026-11-14 (`8d18576`) says three remote branches were deleted; check if this one was meant to stay.

## 8. Technical Names

Arduino, Nano, sketch, firmware, flash, failsafe, RC, CH3, CH4, CH5, e-stop, latch, slew limiter, pivot, corridor, dead-man timer, Kasa, BMS, FET, VESC, CAN, OAK, IMU, RTK, GPS, X1202, `walle-power`, `wall-e`, `main`, `hold`, FOLLOW_ME, WAYPOINT_NAV, MANUAL, ALIGN, limp home, restricted state, monotonic clock, journal, endpoint, authentication, subnet, Grok, regen, barrel jack.
