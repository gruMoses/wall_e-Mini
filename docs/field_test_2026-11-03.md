# East-field test — 2026-11-03

Handoff written 2026-11-02 18:55 America/Denver (CST), revised the same
evening after a pass over the web UI, the map, the heading path, the
reconnect watchdog, and the docs that disagree with the code. The Pi was
offline. Nothing in this session was deployed, and nothing here was run
on the robot.

This file is the spec. The test page, the reachability halt, and the
routines were **not implemented** on 2026-11-02. Implement them from this
document. Do not re-research the repo except to confirm a line this file
cites.

A Fable agent and Claude Code should start here, then `CLAUDE.md`, then
`docs/NEXT_SESSION.md`. Motion code still needs a Grok safety review before
any push to `main`.

## What Kevin does

He has a full day. His part is power, placement, and one tap per routine.

1. Install the new UPS and boot the Pi before any test. Confirm
   `wall-e.service` is up and the phone can open `http://192.168.86.54:8080/`.
   Cycle the RC arm switch off, then on. The startup interlock stays set
   until ch3 has been seen low. If ch1–ch4 sit at a dead 1500 while ch5
   still moves, the receiver leads are the problem described in
   `docs/NEXT_SESSION.md` (2026-10-09, still open). The phone cannot arm
   the robot. Fix that before driving out.
2. Drive the robot with the RC transmitter, or with `/drive` once it is
   armed, into the largest cleared area, just east of the barn and east
   of the house. Those fields were plowed on 2026-11-02. They are bumpy
   and clear of obstacles. Stop near the middle. Sticks centered. Armed.
   Mode MANUAL. `/drive` is only the drive-out. It is not the test runner.
3. Stand about 3 m to the side of the robot, RC in hand, outside the
   circle a pivot will sweep. The front camera hard-stops on a person
   inside 0.8 m, and a 90° pivot swings that view. Open
   `http://192.168.86.54:8080/tests`. Run **Preflight**. If it passes,
   run **Heading**, then **Straight**, then **Box** only if the page
   offers it. One routine at a time.
4. Leave when the page says the run is saved. Analysis can happen while he
   does something else, from the saved folder.

Armed and idle in MANUAL for 10 minutes disarms the robot and latches the
arm switch (`auto_disarm_idle_s` 600). A routine that is actually moving
is not idle. A long pause between routines is. If it disarms, cycle the
arm switch off and on before the next start. The page should show that
latch instead of a generic failure.

He does not mark headings, shoot photos, copy numbers, or watch a terminal.

If he has to walk away mid-run, he disarms with the RC. The Wi-Fi halt is
an extra stop, not a reason for him to leave the robot unattended.

## Where

- Place: plowed ground east of the barn and east of the house. Kevin places
  the robot. The test does not drive there by itself.
- `property_map.jpg` is in the repo root on the machines that have it. It is
  gitignored, about 27 MB, and larger than 48 MP, so an agent image viewer
  will refuse it. `map_calibration.json` is also gitignored. It was **not**
  on the Mac at 18:55 on 2026-11-02. It may already exist on the Pi.
- `waypoints.json` is not in the repo. Saved phone routes live in
  `data/routes/` and that directory is not in this checkout. There is no
  named east-field course. Do not create a property-wide waypoint file.
- The only surveyed lat/lon in the repo is a 2026-07-12 yard cluster
  around 30.29070, −95.29423 (`pi_app/tests/test_gps_heading_align.py`).
  That is the yard, not this field. The RTK base note puts the barn roof
  about 90 m from that yard. Do not drive those points.
- The JPEG is 9512 × 15530. Code has no barn, house, or field label.
  Nothing in the repo is a geofence. The 15 m bubble below is new, and it
  belongs to this test.
- The box routine builds an 8 m square from the GPS fix where Kevin stopped.
  A 15 m bubble around that fix is the fence. The diagonal of an 8 m square
  is about 11.3 m, so the square fits inside the bubble.
- If `/api/map/calibration` reports `calibrated: true` once the Pi is up,
  show the live dot on `/map` during preflight so a person can see that the
  dot is in the east field. A missing calibration does not fail preflight.
  Kevin's placement plus the 15 m bubble is the fence.

## Power

The UPS is new hardware, installed the morning of the test: a Waveshare
UPS HAT (B), watched by `bin/walle_ups.py` (`walle-ups.service`, plus the
early-boot `walle-ups-wakecheck.service`). It replaces the UPSPlus HAT and
its daemon, which were deleted. The watcher halts the Pi 5 s after mains
loss, and the RTC alarm wakes the Pi about every 60 s to check the mains.
The design, the hardware facts, and the bench readings are in
`docs/ups_hat_b.md`. The UPSPlus history in `docs/NEXT_SESSION.md` (the
frozen MCU of 2026-09-27) is closed. It does not describe this supply.

Preflight records whatever `GET /api/ups` returns and continues either way.
The body has the walle-ups schema: `input` (`present`, `absent`, `unknown`,
or `above_supply`), `batt_v`, `batt_ma`, `soc_pct`, `state`, `mode`,
`halt_enabled`, and `ts`. When the status file is missing or old, the route
still answers 200 with `available` false or `stale` true. That is a note in
the result JSON, not a blocker for the heading test. See
`docs/ups_hat_b.md`, section 7.

Leave `usb_max_current_enable` as it is.

Do not power-cycle the Pi after the robot is in the field unless the
service is dead. A service restart clears the GPS heading lock.

## If the east field is only RTK float

Quality 5 is a normal outcome farther from the barn-roof base. It is not
a software failure. Preflight still passes. Heading still runs. Straight
runs on the IMU and is marked `gps_inconclusive`. Box stays off. That is
a complete morning. Do not lower `min_rtk_quality` to make Box move.

## Control-loop contract

The web thread never chooses motor bytes. It sets a request. `Controller.process()`
is the only writer, on the control tick, in this order:

1. If reachability is down and a field routine is active, or reachability
   is down and the mode is `WAYPOINT_NAV` or `FOLLOW_ME`: leave that mode
   for MANUAL before computing a command. Same shape as
   `_leave_follow_me_if_vision_stale`. A healthy link does not do this.
   Box is waypoint nav on purpose, and it keeps running while the probe
   is good. Do not sit in `WAYPOINT_NAV` at zero after a loss. Stale GPS
   already does that. The Wi-Fi halt must not copy it.
2. If a field routine is active and the linear RC sticks are more than 6
   bytes off neutral, or a phone override is non-neutral: abort the
   routine, mode MANUAL, and let that tick follow the sticks. A phone
   file sitting at 126/126 is not an abort. `/drive` writes that file
   whenever its session is armed.
3. Otherwise compute the routine.

There is no `FIELD_TEST` mode string unless you add one. Prefer two
existing paths, and add a mode only for the pivot and the unlocked
straight, which have no existing button.

**Heading pivot, and Straight when the lock is not held.** Add mode
`FIELD_TEST`. In the command branch of `process()` (`controller.py`,
the `if self._mode == "WAYPOINT_NAV"` block near the autonomy command),
handle `FIELD_TEST` the way waypoint nav is handled:

- The routine object returns `(v_cmd, yaw_cmd)` in `[-1, 1]`.
- `mix_to_bytes()` in `pi_app/control/waypoint_nav.py` turns that into
  left and right bytes. Positive yaw is a right turn.
- Build an `AutonomyCommand` (`pi_app/control/state.py`) with
  `source="FIELD_TEST"`.
- For a pivot (`v_cmd == 0` and `yaw_cmd != 0`), set the same flag
  waypoint ALIGN uses (`wp_in_align`) so `_apply_imu_compensation` is
  cleared on that tick. Stacking heading-hold on a commanded pivot
  fights the pivot.
- For a straight (`yaw_cmd == 0` and `v_cmd > 0`), do call
  `_apply_imu_compensation` with `steering_input` 0, and treat
  `moving_ok` / `equal_ok` the way `WAYPOINT_NAV` does, so the hold
  actually applies. Capture the target once at routine start. Do not
  write a new target every tick.
- Obstacle scaling, the 0.8 m stop, disarm, charger inhibit, pack-low,
  and the slew limiter stay where they are, after this block. A pure
  pivot is not forward motion (`left + right` is not greater than
  `2 * 126`), so the corridor will not fold it into a curve. The person
  stop still forces distance 0 and scale 0.
- `FIELD_TEST` is not MANUAL. `compute_throttle_scale` already receives
  `is_manual=(self._mode == "MANUAL")`, so this mode keeps the linear
  waypoint curve, the stale-depth stop, and no creep floor. Netting mute
  updates with `is_manual` false, so it cannot engage during a routine.
  Do not pass `is_manual=True` to soften furrows.
- Slew rates for `FIELD_TEST` use the waypoint-nav accel and decel, not
  the manual rates (`_slew_rates_for_mode`).
- Timeouts, because `_update_armed_idle` only runs in MANUAL. A stuck
  routine will not auto-disarm. Heading: 20 s per pivot. Straight: 60 s.
  Box: 240 s. On timeout, abort, mode MANUAL, verdict `abort`, reason
  `timeout`. Zero the autonomy command through the slew limiter.

**Straight when the lock is held, and the whole Box.** Do not use
`FIELD_TEST`. Call `set_waypoints` on the live `WaypointNavController`
with points from a spherical step, then `activate_waypoint_nav()`.
Copy the step into `pi_app/control/field_test.py`. Do not import
`_offset_gps` from `pi_app/web/waypoint_nav_ui.py`. Control code should
not depend on the web package. The function to copy is the standard
spherical destination (earth radius 6,371,000 m, bearing clockwise from
true north). Unit-test it: from (30.29, −95.294), bearing 0°, 8 m, the
latitude increases by about 7.2e-5 degrees and the longitude barely
moves. Bearings are true-north from the locked corrected heading. Raw
`heading_deg` is boot-relative and is the wrong input. Pass cruise 30
and arrival 1.0 on that call. The phone slider defaults do not apply
if the test page never opens `/navigate`.

On Box start, copy the waypoint list that is already loaded (it may be
empty). On finish or abort, `deactivate_waypoint_nav()` and
`set_waypoints` of that saved list. `deactivate_waypoint_nav()` only
changes the mode. Leaving the square in the live controller means a
later GO on `/navigate` will drive those points, including after Kevin
has driven home.

While a routine is running, `POST /api/nav/start`, `/api/nav/go`, and
`/api/follow_me` return 409 `field_test_active`. The routine's own
waypoint activation is the exception, and it goes through the controller
method, not through a second HTTP client.

Grep `controller.py` for `"WAYPOINT_NAV"` and `"MANUAL"` when adding
`FIELD_TEST`. The obstacle call already treats every non-MANUAL mode as
the strict curve. Do not add `FIELD_TEST` to a branch that relaxes
MANUAL. Do add it next to `WAYPOINT_NAV` where `moving_ok`, `equal_ok`,
and the slew rates are chosen. An unknown mode that falls through to the
RC stick branch will drive the sticks and ignore the routine.

`/calibrate` returns 409 for the same reason. If calibration mode is
already on, routine start returns 409 `calibration`.

The routine object lives on the controller, next to the waypoint
controller. The web blueprint calls `request_start` / `request_stop`.
`process()` calls `routine.tick(now, heading_deg, gps)` and then mixes.
`tick` returns the command or a terminal result. It does not touch
motors, the GPIO relay, or the VESC.

Heading error uses the shortest wrapped delta in `(-180, 180]`. A
crossing of 0/360 is not a 350° error.

## Routines

All four run on the Pi, inside the existing control loop. The phone only
starts, stops, and displays. The browser must not send motor bytes.

Every motion routine uses the same command path as waypoint nav: normalized
`(v_cmd, yaw_cmd)` through the controller, the mixer, the slew limiter,
obstacle avoidance, and the person/animal hard stop
(`ObstacleAvoidanceConfig.safety_stop_radius_m = 0.8`). A test thread must
not call the VESC or the Arduino driver.

Shared abort, on the next control tick, through the slew limiter:

- RC stick more than about 6 bytes off neutral (same band as the arms-up
  twitch gate).
- Disarm, charger inhibit, calibration mode, or the phone e-stop on `/drive`.
- Person or animal inside 0.8 m. Birds stay on the existing rule.
- Depth stale, or `vision_hung`.
- Reachability halt (below).
- Test-page heartbeat missing for 3 s.
- Routine timeout.
- GPS position more than 15 m from the fix captured at the start of the
  routine, once a trusted fix exists.

On abort the mode returns to MANUAL, autonomy commands go to zero, and the
result JSON records the reason. After one of these aborts, a fresh RC link
keeps the robot armed so Kevin can drive it home on the sticks. The
reachability halt does not disarm by itself.

WARNING: The RC ch5 e-stop is not one of these aborts. It stops the motors
at once, disarms, and schedules `sudo shutdown -h now` 5 s later
(`Controller.process` and `ThreadedShutdownScheduler` in
`pi_app/control/controller.py`). The Pi powers off. After a ch5 e-stop the
robot cannot be driven home on the sticks. Recovery is a Pi power-on: press
the Pi power button, or switch the UPS off and on. The service starts again
at boot, so the GPS heading lock is gone. The phone e-stop on `/drive` does
not shut the Pi down.

Obstacle avoidance stays on. Furrows may cut the throttle. Record
`obstacle_distance_m`. If the corridor holds the robot stopped for more
than 3 s, end that routine as `obstacle_held`. Leave the corridor enabled.
Do not engage netting mute.

Driving routines pass `cruise_speed` 30 explicitly (about 0.4 m/s). The
config default is 40. Since 2026-11-03 the phone sliders on `/navigate`
and the defaults of `/api/nav/start` and `/api/nav/go` read
`config.waypoint_nav` (cruise 40, arrival **1.0 m**). Before that the
arrival slider said 0.5 m. This test passes arrival **1.0 m** itself so a
furrow does not count as arrival, and so a tap on `/navigate` cannot
silently substitute the slider. Pivot yaw uses the existing ALIGN law:
proportional, `pivot_yaw_cmd` 0.35 at 90° or more, floor `pivot_yaw_min`
0.18. Do not raise these for the test. Do not copy gain numbers out of a
doc. Live heading-hold gains are in `ImuSteeringConfig` in `config.py`.

### 1. Preflight — no motion

Pass requires all of:

- Armed, mode MANUAL, sticks inside the neutral band, web teleop not
  holding `/drive`.
- IMU heading is publishing. Record `heading_deg`, integrate status, and
  whether ZUPT is active.
- OAK session is up, RGB preview age is under a second, depth age is under
  the existing stale limit, `vision_hung` is false.
- One RGB frame saved from the running pipeline (see Camera).
- Reachability probe succeeds.
- Control loop is running (telemetry SSE is fresh).

Record and do not fail on:

- GPS fix quality, satellite count, HDOP, lat, lon. Quality 4 is required
  only for Straight's cross-track score and for Box.
- `heading_offset_locked`. A service restart for the new UPS starts
  unlocked. The drive out to the field can lock it if the drive was
  forward, MANUAL, RTK fixed, and fast enough. See the heading-lock
  section below.
- `/api/ups` body.
- Map calibration present or absent.

### 2. Heading — pivot in place

Purpose: check the sign and the magnitude of a turn against the IMU, and
leave pictures a later vision pass can agree with.

Sequence, from the heading captured at the start (`H0`):

1. Save an RGB frame.
2. Pivot clockwise (positive yaw) until the heading has increased by 90°.
   Right turn is positive. `heading_deg` is compass-style, clockwise
   positive, relative to boot, in `[0, 360)`. See `CLAUDE.md` IMU section.
   Emit `v_cmd = 0` and a positive `yaw_cmd` through the controller, the
   same shape as waypoint ALIGN. There is no `pivot(degrees)` API. Do not
   POST `/api/teleop/session/drive`. Phone teleop turns IMU heading hold
   off, and it is the wrong path for a measured pivot. A pure pivot is
   not treated as forward, so the obstacle curve does not fold it into a
   curve. The 0.8 m person stop still applies.
3. Stop. Wait until the heading is steady (`integrate_status` `zupt`, or
   yaw rate near zero). Save a frame. Record the heading error. If the
   status stays `zupt` while the tracks are actually turning, fail
   `zupt_stuck`. Do not stop `wall-e.service` to run
   `oak_yaw_chalk_test`; that program takes the camera.
4. Pivot back by 90° (counterclockwise). Stop. Save a frame. Record the
   error back to `H0`.

Pass: each stop is within 8° of its target, and the first pivot's heading
increased. The 8° band is a test tolerance. It does not change
`align_threshold_deg` (12°).

Fail the routine and stop the session if the heading decreases during the
clockwise pivot. That is a sign error. Fix it at the IMU reader and
re-chalk. Do not add another inversion downstream. `invert_output` stays
false. `oak_yaw_rate_scale` stays 1.0. Do not restore 0.46.

Timeout: 20 s per pivot. GPS is not required. The GPS heading lock is not
required. Do not write a new heading-hold target on every tick. MANUAL
heading hold re-captures on the return to neutral; leave that behavior
alone.

### 3. Straight — short drive on a fixed heading

Purpose: directional hold on the plowed ground.

Capture the heading at the start. Drive forward about 8 m holding that
heading. Use GPS distance when fix quality is 4. When it is not, integrate
a conservative distance from wheel RPM if both RPM channels are present,
and mark `gps_inconclusive`. If neither GPS nor RPM can measure 8 m, abort
as `no_distance`.

Hold the heading inside the controller. Two acceptable implementations:

- If `heading_offset_locked` is already true and the fix is quality 4,
  one relative leg through `POST /api/nav/go` (or `set_waypoints` of a
  single point 8 m along the locked heading), cruise 30, arrival 1.0.
  Waypoint DRIVE already runs the IMU steering PID.
- If the lock is not held yet, command a straight forward input on the
  MANUAL heading-hold branch (both tracks forward, the compensator
  re-captures at the start of the routine). Do not write
  `/tmp/wall_e_bt_latest.json`. A phone-teleop straight drive disables
  heading hold, so it cannot be this routine.

Real stick motion aborts and returns authority to Kevin. The routine must
not keep a synthetic stick override after abort.

Save an RGB frame at the start, every 2 s, and at the stop.

Pass when fix quality is 4: heading stayed within 10° of the start, and
the GPS cross-track at the stop is under 1.5 m. The 1.5 m band is loose
because the field is furrowed.

Pass when GPS is inconclusive: heading stayed within 10°, and the result
says the cross-track was not measured. That pass does not unlock Box.

This drive is also the chance to lock GPS heading. The existing
`GpsHeadingAligner.update_cog()` locks on its own when its gates pass. The
routine does not have a separate "wander until locked" phase.

### 4. Box — only if the page enables the button

The button is enabled only when all of these are true:

- Heading passed in this session.
- Straight passed with fix quality 4 (not the inconclusive pass).
- Current fix quality is exactly 4.
- `heading_offset_locked` is true.
- Reachability is good.
- Still inside the 15 m bubble around the straight-run start, or a fresh
  preflight has just captured a new start fix in the field.

Course, built in memory from the start lat/lon and the locked true-north
heading. Do not write `waypoints.json`. Do not call `load_waypoints` on
the property file.

- Leg 1: 8 m forward along the heading at the start of the box.
- Leg 2: 8 m to the right.
- Leg 3: 8 m back.
- Leg 4: 8 m to the left, back to the start.
- Arrival radius 1.0 m, passed in the request. Config and the
  `/navigate` slider default agree (fixed 2026-11-03).
- Cruise 30, passed in the request.

Run it through `WaypointNavController` and
`Controller.activate_waypoint_nav()`. Four relative `/api/nav/go` calls
also work if each one is waited to ARRIVE and the bubble is checked
between legs. `set_waypoints` of four points in one activation is the
cleaner run. Do not load a file. The existing activation refuses
`gps_quality_not_trusted` and `heading_alignment_not_locked`, halts on a
stale GPS fix older than 3 s, and commands zero motion if quality leaves
4 during the run. RTK float (quality 5) does not count as fixed. On this
test, a reachability loss must leave WAYPOINT_NAV entirely. Stale GPS
today zeros the command and stays in the mode; do not copy that for the
Wi-Fi halt.

Keep every waypoint protection. Obstacle avoidance and the 0.8 m
person/animal stop stay on. Releasing the hold-to-go button on
`/navigate` does not stop a run. The test page Stop button calls
`deactivate_waypoint_nav()`. Do not use `/calibrate` for any of this.
Heading PID tune writes motor bytes around obstacle avoidance.

Save an RGB frame at each corner. Pass: all four points arrived, the robot
never left the 15 m bubble, and the finish is inside the arrival radius of
the start. A reachability loss, a stick, a person, or a stale fix is an
abort, not a pass.

If the lock is missing after Straight, the button stays off and the result
is `skipped_no_lock`. That is not a failure of the heading routine.

## Heading lock lifetime

`docs/gps_heading_alignment.md` and the code comments agree with the code
since 2026-11-03. Trust that doc and `Controller._on_armed_session_ended`
in `pi_app/control/controller.py`: the lock is kept across disarm and RC
stale for the life of the process. A ch5 e-stop halts the Pi, which ends
the process, so the lock does not survive it. The lock drops on an IMU
frame discontinuity (`oak_reconnect_count` or `count_cum_reset` changes)
and on a service restart.

`Controller.waypoint_nav_activation_blocked()` returns
`heading_alignment_not_locked` until `GpsHeadingAligner.locked` is true,
when the aligner is enabled and GPS and IMU are both present. Defaults in
`config.py`: aligner enabled, `min_rtk_quality` 4 matched exactly.

A forward drive at RTK fixed can lock during the trip from the barn into
the field. Preflight shows the flag. Nobody needs a separate calibration
lap unless the flag is still false after Straight.

## Reachability halt

This is a new check. It is not the NetworkManager reconnect watchdog.

`docs/wifi_watchdog.md`, `bin/wifi_watchdog.py`, and
`bin/wifi-watchdog.service` are the reconnect design. The reworked script
is on `main` (`f4e209f` on 2026-09-28, `e31067b` and `df1187d` on
2026-10-04). `wifi-watchdog.service` is installed and active on the Pi
(verified 2026-11-03). The `wwr outdoor` fallback profile exists, and the
`preconfigured` profile now has `auth-retries` -1. The review findings
that stopped the first draft are fixed: the script never takes down a link
whose gateway has answered, and `bin/add_wifi_profile.sh` prompts for the
password. `docs/wifi_watchdog.md` section 5 still says `preconfigured` is
the only profile and has `auth-retries` 0. That is out of date.

The watchdog and the new halt are separate. The watchdog repairs the Wi-Fi
link. The halt only stops the robot's autonomy when the internet probe
fails. The halt code must not call `nmcli`, change a Wi-Fi profile, or
restart `wifi-watchdog.service`.

The new halt answers Kevin's request: when the robot cannot reach the
internet, automated tests and waypoint navigation stop.

- Probe `http://connectivitycheck.gstatic.com/generate_204` and expect
  HTTP 204, timeout 1.5 s. That is Google's own connectivity check, which
  is the "something like google.com" he asked for, and it does not depend
  on ICMP. If the HTTP client errors, one TCP connect to `8.8.8.8:53` is
  the fallback for that attempt. Do not ping the LAN gateway. A live
  gateway with a dead upstream must still count as down.
- Run the probe on a background thread. The 30 Hz loop only reads a
  snapshot. A hung probe counts as a failure (fail closed).
- One failure does not trip. Two failures in a row, at least 5 s apart,
  trip the halt. A single blip must not cut a pivot.
- While the halt is tripped: abort the active field routine if one is
  running, call `deactivate_waypoint_nav()` so the mode is MANUAL again,
  and if FOLLOW_ME is active leave it the same way vision-stale does.
  This test does not start follow-me. A ch4 switch left high does not
  re-enter by itself. Refuse `activate_waypoint_nav()`, follow-me
  activate, and any routine start with reason `wifi_unreachable`.
  If calibration mode is on, abort it too. That wizard writes motors
  directly.
- MANUAL stick control stays available when the RC link is fresh. The
  phone deadman (250 ms, server side, in `pi_app/web/teleop.py`) already
  zeros phone drive when the phone cannot reach the Pi. Do not add a
  second phone deadman. The google check is upstream reachability. The
  phone can still be on the LAN while upstream is dead, and in that case
  the test still stops. Do not disarm a fresh RC link.
- Clear the halt after two successes in a row. Clearing it does not resume
  a routine. Kevin starts the routine again from the page.
- Log one event, `wifi_unreachable`, with the probe error and the mode.
  No secrets, no `nmcli` output.
- Also stop the routine when the test page has not heartbeated for 3 s.
  That covers a dead phone while the robot still has upstream. Stick
  motion aborts either way.

Suggested place: `pi_app/control/reachability.py`, polled from
`pi_app/app/main.py`, read by the controller at the same point that
already gates waypoint activation. Unit-test the flag with a fake probe.
Do not open sockets in the unit test.

## Camera

The OAK-D Lite allows one process. The running `wall-e.service` already
owns it. Grab frames from that process.

- Still for the archive: `OakDepthReader.get_latest_rgb_frame()` in
  `pi_app/hardware/oak_depth.py`. It returns `(bgr_frame, monotonic_ts)`.
  Encode JPEG in the test recorder. This is the front RGB camera.
- The phone preview is the annotated MJPEG at `/stream/rgb`
  (`recorder.get_latest_annotated_jpeg`). The test page can show that
  stream. Save the raw preview for the archive, and also save the
  annotated JPEG when it is available, under a second filename.
- Do not construct a second depthai pipeline. Do not change
  `OakDetectionConfig.camera_fps` (15.0 is load-bearing).
- Do not run MediaPipe, a cloud vision API, or a large model inside the
  control loop. A slow call there stalls heading and has already stalled
  the motors once.

Each saved frame gets a sidecar row in the result JSON: filename,
monotonic time, wall time, routine, step, `heading_deg`, yaw command,
lat, lon, fix quality, `obstacle_distance_m`.

Live pass/fail is numeric (heading error, GPS cross-track, arrival).
Vision is evidence for a later pass. A vision disagreement must not be
required to command the motors. Tomorrow's Fable agent reads the folder
after the run, or between routines if Kevin leaves the page open. The
prompt for that pass is at the bottom of this file.

## Test page

New page: `GET /tests`.

Phone-sized. Big targets. One routine visible as the running one.

Header link "Tests" next to Map / Navigate / Calibrate / Debug in
`pi_app/web/oak_viewer.py` (the dashboard header around the existing
`/map` link). Add the same link on `/drive` (`pi_app/web/teleop.py`),
in a corner that does not cover the joystick, so Kevin can leave the
drive page without typing a URL. `/map` and `/navigate` only link back
to the dashboard; the dashboard link is enough for those.

Suggested files:

- `pi_app/web/field_tests_ui.py` — Flask blueprint, same shape as
  `create_map_blueprint` / `create_nav_blueprint`.
- Register it in `OakWebViewer` next to the other blueprints in
  `pi_app/web/oak_viewer.py` (the block that logs "Property map registered
  at /map").
- `pi_app/control/field_test.py` — routine state machine. It produces
  `(v_cmd, yaw_cmd)` or delegates the box to the existing waypoint
  controller. It does not talk to motors.
- `pi_app/tests/test_field_test.py` — no hardware.
- `pi_app/tests/test_reachability.py` — no sockets.

API, all JSON:

- `GET /api/tests/status` — current routine, step, heading error, GPS
  quality, lock flag, reachability, last abort reason, whether Box is
  allowed.
- `POST /api/tests/start` with `{"routine": "preflight"|"heading"|"straight"|"box"}`.
  Reject with 409 unless armed, MANUAL, sticks neutral, no e-stop, no
  other autonomy, reachability good, and (for box) the enable rules above.
- `POST /api/tests/stop` — abort reason `operator`.
- `POST /api/tests/heartbeat` — the page posts this at least every 1 s
  while a routine is running.
- `GET /api/tests/runs` — list saved runs.
- `GET /api/tests/runs/<id>` — the result JSON.
- `GET /api/tests/runs/<id>/frames/<filename>` — one JPEG.

`/api/telemetry` allows 6 SSE clients. A test page that also opens the
dashboard, the map, and navigate can hit that cap and get 503. Poll
`GET /api/tests/status` about twice a second for the run. Use
`/api/telemetry` only for the idle HUD, and close it when a routine
starts if the cap is tight. A dedicated progress stream is optional and
should look like `/api/cal/progress`, not a seventh telemetry client.

Results on the Pi, under `logs/field_tests/<utc-stamp>_<routine>/`:

- `result.json`
- `frames/*.jpg`

`logs/` is gitignored. The files stay on the Pi. Read them there, or copy
them off the Pi after the run. Do not commit images.

Show the running numeric result on the page as it updates, and the saved
result after the routine ends. Image review can be the same day, after
the folder exists.

## What this test does not include

Follow-me, gestures, arms-up, netting mute, a tour of the whole property,
a change to steering gains, a change to camera FPS, and any edit to the
UPS watcher (`bin/walle_ups.py`). Animal slow-zone work stays as it is.

## Implementation order

1. Reachability flag and its unit tests. Wire the refuse path into
   `activate_waypoint_nav` before any routine can drive.
2. Field-test state machine and unit tests, with a fake clock, fake IMU,
   fake GPS, and a fake probe. Cases to assert, by name:
   - `test_pivot_sign_fail_stops_session` — heading decreases on a
     clockwise command, verdict fail, no second pivot.
   - `test_pivot_wrap_across_north` — 350° to 20° is +30°, not −330°.
   - `test_straight_without_lock_does_not_use_teleop` — the command
     source is `FIELD_TEST`, and nothing writes the teleop override file.
   - `test_box_refused_on_float_or_unlocked` — quality 5 or an unlocked
     heading leaves the button off and does not call `set_waypoints`.
   - `test_bubble_abort` — a fix 16 m from the start aborts.
   - `test_timeout_returns_manual` — a pivot still running at 20 s aborts.
     Auto-disarm will not save you. It ignores non-MANUAL modes.
   - `test_stick_and_heartbeat_abort` — linear stick 7 bytes off neutral
     aborts. A 126/126 phone file does not. A heartbeat older than 3 s
     aborts.
   - `test_reachability_leaves_waypoint_mode` — two probe failures leave
     `WAYPOINT_NAV` for MANUAL and refuse the next start. One failure
     does not. Recovery does not resume the routine.
   - `test_offset_north_8m` — the copied spherical step, as above.
   Obstacle distances stay in the existing avoidance function. Do not
   reimplement 0.8 m or 0.4 m in the test module.
3. Blueprint and page. A desktop browser test of the page is possible
   against a Flask test client. The Pi is the first full test.
4. Frame grabber writing into `logs/field_tests/`. On a laptop without an
   OAK, the grabber no-ops and records `frame_missing`.
5. Grok safety review of the motion diff before a push to `main`.

`bin/auto_deploy.sh` pulls `origin/main` every 5 minutes once the Pi is
on the network, and restarts the service when the diff is not docs-only.
The Pi was offline on 2026-11-02, so a push waits until it reconnects and
then deploys. Do not push motion code until Kevin approves it. A
docs-only change on `main` pulls without a service restart. Pause a
deploy during the field test with `touch /home/pi/wall_e-Mini/.deploy_hold`
on the Pi, and remove that file when the test is over.

Run the unit tests on the laptop:

```bash
python3 -m pytest pi_app/tests/test_field_test.py pi_app/tests/test_reachability.py -q
```

The full suite on the Pi, after the code is there and the robot is
disarmed:

```bash
cd /home/pi/wall_e-Mini
python3 -m pytest pi_app/tests -q
```

Use pytest, not `unittest discover`. Several test files use plain `def test_`
functions and no `TestCase`. `unittest discover` collects 0 cases from them and
reports no error.

## Conflicts already checked

Trust the code and `CLAUDE.md` over an older paragraph in a doc.

| Topic | Trust | Ignore |
|---|---|---|
| Heading sign | `CLAUDE.md` and `config.py`: clockwise positive, `invert_output` false, yaw scale 1.0, source `gyro_y`, `imu_source` `"oak_d"` | Scale 0.46, a second inversion, or restoring `imu_source` `"auto"`. The chalk table in `docs/heading_tuning.md` that shows about −90° for a clockwise turn is the pre-2026-09-19 sign |
| Which IMU | `config.py` `imu_source = "oak_d"` | An older note that `imu_source` is `"auto"` or that an external I2C IMU is in use. `CLAUDE.md` was corrected 2026-11-02 and `docs/imu_steering.md` 2026-11-03 |
| Plant gain | Heading derivative, about 0.65–0.71 deg/s per byte, from the 2026-09-20 run. This test does not retune gains. Live PID numbers are in `config.py` | A fit from `imu.yaw_rate_dps`, or a PID number copied from a doc (`docs/imu_steering.md` used to print `kp` 1.7 / `max_correction` 220 and now points at `ImuSteeringConfig`) |
| Camera rate | `camera_fps = 15.0` | A doc that calls 8 fps or 30 fps the current rate |
| Waypoint GPS | Quality exactly 4. Float 5 is refused. Stale after 3 s halts | Any note that float is good enough to drive |
| Heading lock across disarm | Kept for the process lifetime. Cleared on service restart, IMU frame break, or a ch5 e-stop (the Pi halts). Code: `Controller._on_armed_session_ended`. The lock path is `update_cog()`, and the path need not be perfectly straight | A note that disarm clears the lock, or that `cog_deg` is logging-only. The docs and code comments that said so were corrected 2026-11-03 |
| Wi-Fi | New Google reachability halt in this file. The reconnect watchdog is on `main`, installed and active on the Pi; `preconfigured` `auth-retries` is -1 | A note that the watchdog is shelved or not deployed. `docs/wifi_watchdog.md` section 5 (`auth-retries` 0). A LAN-gateway ping as a WAN check |
| Straight drive | Controller heading hold, or one waypoint leg after the lock | `/api/teleop/session/drive` or `/tmp/wall_e_bt_latest.json`. Teleop disables IMU heading hold |
| Heading tool | Controller yaw, same shape as ALIGN | `/calibrate` heading PID tune, which writes motors around obstacle avoidance. `oak_yaw_chalk_test` while `wall-e` is running |
| UPS | Waveshare UPS HAT (B), watched by `bin/walle_ups.py` (`docs/ups_hat_b.md`). Record `/api/ups` and move on | Any note about the UPSPlus HAT, its frozen MCU, or `scripts/upsPlus_power_daemon.py` (all retired 2026-11-03) |
| Person stop | 0.8 m in every mode, including these routines | Putting the removed MANUAL person-cap back on the obstacle curve |
| Stale vision | Follow-me is not in this test. If you touch detection code anyway: a stale list stays stored so a close person can still force the stop. Follow-me drops out on freshness | Returning `[]` from `get_person_detections()` to "clear" a stall |
| Map | Kevin's placement and a 15 m GPS bubble | A pixel guess on `property_map.jpg`. The yard cluster 30.29070, −95.29423. Placeholder 30.123, −95.456 in the nav API docstring |
| Arrival radius | 1.0 m, passed by this test. Config and the `/navigate` slider default agree | An older note that the `/navigate` slider default is 0.5 m |

## Result JSON

One file, `logs/field_tests/<utc-stamp>_<routine>/result.json`. The vision
pass reads this shape and nothing else.

```json
{
  "routine": "heading",
  "started_at": "2026-11-03T15:04:12Z",
  "finished_at": "2026-11-03T15:04:40Z",
  "verdict": "pass",
  "abort_reason": null,
  "start_fix": {"lat": 0.0, "lon": 0.0, "quality": 4, "age_s": 0.4},
  "heading_lock": false,
  "imu_source": "oak_d",
  "steps": [
    {
      "name": "pivot_cw",
      "target_delta_deg": 90.0,
      "measured_delta_deg": 88.4,
      "heading_start_deg": 12.0,
      "heading_end_deg": 100.4,
      "settle": "zupt"
    }
  ],
  "samples": [
    {
      "t_mono": 0.0,
      "heading_deg": 12.0,
      "v_cmd": 0.0,
      "yaw_cmd": 0.35,
      "lat": null,
      "lon": null,
      "fix_quality": 4,
      "obstacle_distance_m": 2.5,
      "obstacle_scale": 1.0,
      "frame": "frames/000.jpg"
    }
  ],
  "notes": []
}
```

`verdict` is `pass`, `fail`, `abort`, or `skip`. `samples` is about 2 Hz,
not every control tick. Cap a routine at 200 frames. A missing frame sets
`frame` to null and adds `frame_missing` to `notes`. Numeric pass/fail
does not wait on the JPEG.

## Prompt for the vision pass

Use this after at least one routine has saved a folder. The Pi must be
reachable, or the folder must have been copied off it. Fable tokens are
for this pass, not for driving.

```
Read docs/field_test_2026-11-03.md. Then read
logs/field_tests/ on the Pi (or the copy in front of you).

For each routine folder, open result.json and the JPEGs in order.
Say whether the pictures agree with the numeric result:
- Heading: the background shifts clockwise on the +90 pivot and
  returns on the way back. A heading decrease is a fail even if
  nobody looks at the pictures.
- Straight: the view stays on open plowed ground. A building, a fence,
  or a person growing large in the frame is a fail of the run's safety
  story. Write it up. Do not command the robot.
- Box: each corner still shows the same open field.

Write a short verdict per routine: agree, disagree, or frames missing.
Quote the heading numbers from result.json next to what you see.
Do not change gains, signs, or waypoint files from the pictures alone.
```
