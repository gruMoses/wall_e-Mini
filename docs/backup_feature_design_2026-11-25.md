# Design: back-up in FOLLOW_ME (2026-11-25, revision 3)

Status: DESIGN ONLY. No code exists for the motion. Revision 2 applied the first Grok safety review, revision 3 the second (both in section 11). Kevin answered section 9 on 2026-11-25 (decisions at the top of section 9); question 11 is new and open. Before any push that can move the robot: a Grok check of the code diff (section 10).

Relation to `docs/backup_pump_design.md` (approved 2026-09-24): that document stays the record for the pump signal (its section 5). This document replaces its sections 6 and 7 (motion and safety), because the person stop hold, the RC latches, the pack guard and `wheel_ground_scale` are all newer than it.

## 0. Read first: three things that differ from the backlog text

1. **The trigger is not arms up.** Raised hands leave the frame when Kevin is closer than about 3.5 m to 4.5 m (bench, 2026-09-24, `docs/NEXT_SESSION.md` section 2). The behaviour happens at 1.5 m to 3 m, where arms up reads "down". Motion runs only on a signal that passes the 10-minute false-start trial and works at 2 m. Today that means the pump (`fm-pump`, `dfbda57`, log only, never trialled). Arms up may run log-only for data.
2. **Version 1 reverses straight. It does not keep Kevin centred.** A steer command at low reverse speed drives one track forward: the robot pivots, the front corner sweeps toward a person beside it, and the rear swings over ground no sensor sees (review A1).
3. **The robot reverses only over the straight piece of ground it just drove forward over** (review A2). Kevin faces the camera while he signals; he cannot see the ground in the shadow of the chassis. Grok requires a rear sensor before any run on the ground. Kevin's decision (2026-11-25): no rear sensor required; ground runs are supervised (a spotter who sees behind the robot, the transmitter in Kevin's hand). The sensor gate stays in the design for when one is fitted.

## 1. Behaviour

Plain words: Kevin is being followed in a straight line. He turns, faces the robot, gives the signal and walks slowly toward it. The robot ramps into a slow, straight reverse and holds the range it had at the start. It goes back no farther than the straight segment it just drove. The robot moves only while a fresh "out" frame of the pump is in view, so it stops between pumps and when he drops the signal. Forward then stays at zero until he is clearly seen walking away. Once the robot stands, it may pivot in place to keep him centred, as at the 0.8 m stop. Any doubt ends the movement.

States (new, inside FOLLOW_ME only; the mode stays FOLLOW_ME). The module runs on every tick while the state is not OFF, also after the enable latch drops.

| State | Motors get | Leaves to |
|---|---|---|
| OFF | follow-me command, unchanged | READY when the enable latch is on (section 5, item 4) |
| READY | follow-me command, unchanged | BACKING when every entry gate passes (section 2) |
| BACKING | equal reverse bytes on both tracks (section 3). Both bytes are 126 or less on every tick | SETTLE on a normal end; LOCKED on a fault end; OFF on any FOLLOW_ME exit |
| SETTLE | forward part 0; the follow-me pivot passes (Kevin, 2026-11-25, question 3), under the three limits below | READY on the evidence in section 3.4 |
| LOCKED | forward part 0; the follow-me pivot passes (Kevin, 2026-11-25) | OFF only by FOLLOW_ME off and on, or disarm |

Limits on the pivot in SETTLE and LOCKED (second review):
1. **Stand before pivot.** On every exit from BACKING the ramp target is 126/126. It is held until both slew bytes are at neutral. Only then may the pivot apply. No tick mixes a reverse track with a forward track, and the first SETTLE or LOCKED tick never commands a byte above 126.
2. The pivot needs an RC frame 0.3 s old or less. Without one both tracks are neutral.
3. Inside 1.0 m: see open question 11.

Every exit from BACKING. "Ramp" means the reverse target goes to neutral through the slew limiter (FOLLOW_ME decel 250 bytes/s: 32 bytes in 0.13 s). "Write" means neutral goes to the VESC on that same tick (`motor.stop()` or `set_tracks(126, 126)`) and the slew is held at neutral (`_hold_slew_at_neutral`), so whatever comes next ramps from neutral. A write is the last motor command on its tick: nothing slews toward a stick or a pivot after it. A stop may be immediate; a release never is.

| # | Exit cause | Next state | Motors |
|---|---|---|---|
| 1 | Signal down (no "out" frame for 0.7 s), or its input sample older than 0.3 s | SETTLE | ramp (the speed is already 0, section 2.1) |
| 2 | Segment end, distance cap, time cap, session budget | SETTLE | ramp |
| 3 | A stick outside the neutral band (linear bytes, 6 bytes), or a phone override present | SETTLE | ramp. FOLLOW_ME still ignores the sticks; ch4 gives control |
| 4 | Locked target not fresh for 0.3 s, range not "ok" for 0.3 s, target box at a side edge, target centre beyond 0.30 of the half-frame | SETTLE | ramp |
| 5 | Range below 1.0 m (the smaller of raw and filtered), the 0.8 m stop tier, or the person stop hold (obstacle distance 0.0) | LOCKED | ramp; the hold owns forward |
| 6 | Identity change, any second person or stop-tier animal in view, rear sensor, stall, overspeed, track mismatch, eRPM older than 200 ms, yaw off the segment, heading stale, depth stale | LOCKED | ramp. A rear-sensor hit takes this exit whenever a sensor is fitted, whatever `rear_sensor_required` says |
| 7 | FOLLOW_ME exit: ch4 low, vision stale (1.5 s), pack guard | OFF, mode MANUAL | existing path: slew at MANUAL rates toward the sticks (section 5, item 9 for the hard-stop case) |
| 8 | Web stop (`deactivate_follow_me`) | OFF, mode MANUAL | write, on the first tick that sees it |
| 9 | Disarm, ch5 e-stop, RC stale, charger inhibit, calibration, any other early return | OFF | write. NOTE: the calibration return does not stop the motors today (review A3); the code change adds it |
| 10 | An exception in the new module | LOCKED | write, in the `except` path only (section 5, item 3) |
| 11 | Enable latch cleared (expiry, budget, API) | SETTLE | ramp |
| 12 | RC frame older than 0.3 s (well before the 1.0 s RC-stale disarm) | SETTLE | write. No pivot until a fresh RC frame |

A movement that ended never resumes. A new one needs every entry gate again.

## 2. Entry gates (READY to BACKING)

All must hold on the same tick. Counts are **new camera results** (a new NN sequence number), never 30 Hz control ticks (gesture review 2026-09-20, finding 4).

1. Enable latch on, armed, mode FOLLOW_ME, not calibrating, no charger inhibit, no pack guard block, no emergency.
2. RC frame age 0.3 s or less. Sticks inside the neutral band. No phone override.
3. Detection stream fresh: `get_detection_freshness()` age 0.3 s or less (FOLLOW_ME itself allows 1.5 s). Depth age 0.5 s or less (`stale_timeout_s`).
4. Follow-me has a committed, fresh target with a real `track_id`, `depth_status` "ok", and the same `track_id` for the last 1.0 s. A stable `track_id` is not proof of identity; gate 8 and the jump gates carry that.
5. Target range 1.5 m to 6.0 m. Target box 0.05 or more from each side edge. Target centre within 0.20 of the half-frame.
6. Signal active on the **locked target**, confirmed by the detector's own start rule (pump: 2 peaks in 3.0 s).
7. Signal re-arm: seen at rest on 5 new results since the last movement, and 2.0 s have passed.
8. Alone in view: the locked target is the only stop-tier detection (person, cat, dog, horse, sheep, cow) in the frame for the last 1.0 s. `close_unknown` is false.
9. No stop: obstacle distance above 0.0 and the person stop hold not active.
10. Robot stopped: both track eRPM present, 200 ms old or less, 300 or less, and the emitted bytes at neutral, for 0.5 s.
11. Straight segment (section 3.3): allowed reverse of 0.5 m or more (that needs 1.6 m of wheel distance on the forward leg), heading 0.2 s old or less, current yaw inside the segment band.
12. Rear sensor, if one is fitted: healthy and clear. Not required (Kevin, 2026-11-25); without one, ground runs are supervised only (test plan steps 6 and 7).
13. Not SETTLE, not LOCKED, session budget left, moves left in this arm session.

A refused start is logged with the first gate that failed. The pending start is consumed; it does not wait for the gate to clear.

### 2.1 The signal interface

`active` (bool), `out_now` (the newest input frame is an "out" frame), `sample_ts` (monotonic capture time of that frame), `track_id` (whose body it was read from), `rest_streak`.

1. Pump (`pi_app/control/pump_gesture.py` on `fm-pump`): reads the box of the locked target only. Its "continue" rule keeps `active` for 0.7 s after the last "out" frame (the gap between pumps is about 0.5 s). That bridge keeps the STATE only: the speed is 0 unless `out_now` is true. A new peak inside the bridge moves the robot again without the 2 s re-arm. The robot so moves in pulses with the pumps; hands held out in a low "V" give a steady reverse.
2. Arms up, log only: crop the locked target, not the largest box; keep a sample only when the pose keypoints lie inside the locked box; stamp it with the frame capture time. It gets its own `ArmsUpDetector` instance, not the twitch one.
3. `signal_max_age_s` 0.3 s is the age of the newest input sample. Keep it at 0.3 s even though one dropped frame can then stop a movement: a false stop is the safe error.
4. Stop travel after the hands really go down: about 0.1 m (detection latency about 0.2 s plus the 0.13 s ramp, at 0.33 m/s). The 0.3 m of blind reverse through the bridge in revision 2 is gone.

## 3. The reverse command

### 3.1 Speed

1. `d_keep = max(2.0 m, range at the start)`, fixed for the movement.
2. On each new fresh detection of the locked target with `out_now` true: `v = clamp(0.8 x (d_keep - z), 0, v_cap)`. With `out_now` false: `v = 0`. Here `z` is the larger of the raw and the filtered range: a doubtful range gives less motion. The close-range exits (exit 5) use the smaller of the two: no rule discards a close range.
3. No new detection on this tick: hold the last command, never raise it. After 0.3 s: exit 4.
4. Cap: 32 bytes below neutral, 0.33 m/s on the ground (`speed_loop_mps_per_byte` 0.010232, which includes `wheel_ground_scale` 0.815). The "0.4 m/s" agreed on 2026-09-20 was in the old units, so 32 bytes is that same wheel speed. The code clamps the config value to 0..40 bytes at use.
5. Open loop in bytes; the VESC holds eRPM for a byte. The follow-me velocity PID is not used in reverse.

### 3.2 Steering: none in version 1

1. left = right = 126 - rev. The module asserts both bytes are 126 or less before it returns them; a failed check is exit 10.
2. Kevin drifts off centre (0.30 of the half-frame): exit 4. He lines up again and signals again.
3. A later version may add steer with `|steer| <= rev`, so neither track ever goes forward. That needs its own review.

### 3.3 The straight segment (where the robot may reverse)

1. While FOLLOW_ME drives forward (both emitted bytes above 126), the module sums the measured wheel distance. The segment is credited with **half** of it. Wheel slip on the forward leg reads long, and raw credit would allow reverse over ground the robot never covered. Half the wheel distance was chosen over the GNSS displacement: RTK float or fixed is often absent here (`docs/rtk_float_investigation.md`), it would add a second dependency and an antenna offset, and the wheel figure is always there.
2. The segment restarts at 0 when: yaw leaves a band of plus or minus 3 degrees around the heading at the segment start; the tracks counter-rotate; the heading is older than 0.2 s or its frame changed (`_check_imu_frame_continuity`); the forward leg trips the track-mismatch rule (failure 9); FOLLOW_ME ends; the robot disarms.
3. Ground older than 30 s drops out of the segment: something may have moved onto it.
4. Reverse travel is taken off the segment at full value. Allowed reverse = segment - 0.3 m margin. One number rules gate 11: allowed reverse 0.5 m or more.
5. During BACKING the yaw must stay inside the same band: outside it, exit 6.
6. Geometry: the body is 0.82 m wide (`robot_width_m`), the tracks 0.28 m apart (`pursuit_wheelbase_m`). At 3 degrees the rear drifts 0.13 m sideways over 2.5 m, so the reverse stays inside the swept body width less that strip. The rear retraces ground the whole body passed over, so the rear overhang does not add to the along-track need; the 0.3 m margin is for odometry error. NOTE: the body length and rear overhang are not in `config.py`; measure them and re-check the margin before the code change.
7. WARNING: this rule covers ground that was clear when the camera drove over it. It does not see anything that arrived afterward. Only a rear sensor or the spotter covers that.
8. Expect refusals: follow-me steers all the time, and a pivot at the stop restarts the segment. Test step 3 measures how often a segment is there.

### 3.4 Limits, and the SETTLE release

1. Time: 8.0 s from the first non-neutral byte (hard limit; Kevin accepted 8 s on 2026-09-24).
2. Distance: the smallest of the allowed reverse (3.3), 2.5 m, and the session budget. Two estimates, the larger counts: the signed mean eRPM as m/s (`_actual_speed_mps`, already x 0.815), absolute, summed each 50 ms telemetry poll; and emitted reverse bytes x 0.010232 x dt.
3. NOTE: 0.33 m/s for 8 s is 2.6 m, so the 3 m accepted in September cannot be reached. Do not change one speed constant alone (`docs/wheel_speed_scale_2026-11-10.md`).
4. Session budget: 6.0 m of net reverse; forward travel refills it. It is a cap only. It is not path memory (a pivot neither refills nor reduces it); rule 3.3 is the path memory.
5. SETTLE releases only on hold-quality evidence. A poll is accepted when it is fresh, has the same `track_id`, a box clear of the side edges, `depth_status` "ok", the **smaller** of raw and filtered range at least 0.5 m beyond the range at the stop, and a range within walking speed of the previous accepted poll (1.5 m/s x dt + 0.1 m). Release needs an unbroken run of accepted polls that is at least 1.0 s long and at least 15 polls. So the +0.5 m holds now and held 1 s ago; a far bias cannot walk in over 5 frames. A rejected poll, a missing poll, a jump gate hit or a second stop-tier box resets the run. Time alone never releases it.
6. On the SETTLE to READY edge, before that tick's follow-me output is used: reset `SpeedLayer`, its velocity PID, `SafetyLayer._prev_speed` and the 15 Hz output cache. Forward then ramps from 0.

## 4. Failure modes

| # | Failure | Mitigation |
|---|---|---|
| 1 | Detections stale or frozen (2026-09-25) | Gate 3 and exit 4 at 0.3 s on the freshness clock. The 1.5 s FOLLOW_ME drop-out stays behind it. `get_person_detections()` is not changed: a stale close person still forces the 0.8 m stop |
| 2 | Signal source hung or slow | Input sample older than 0.3 s: exit 1 |
| 3 | Identity swap | One `track_id` for the movement; exit 6 on a change, on a range step above 0.5 m or a centre step above 0.2 between fresh samples, or on any second person box in view, overlapping or not |
| 4 | Signal end missed | "Active" must be seen again and again. A missed frame stops; it never continues |
| 5 | Kevin walks faster than the robot backs | Range below 1.0 m: exit 5, LOCKED, forward 0 (the follow-me pivot passes once the robot stands, Kevin 2026-11-25). The robot does not drive toward him when he reaches it (section 4.1) |
| 6 | Person, child or animal behind | Kevin's decision: no sensor required, supervised runs (spotter, transmitter in hand, per-session latch). Also: straight segment at half credit (3.3), no steer, motion only on a fresh "out" frame, 0.33 m/s, 8 s. A fitted sensor always stops (exit 6). See question 6 |
| 7 | Backs into an obstacle | Stall: measured wheel speed below 30 % of commanded, summed over the movement to 0.5 s after the ramp. Time with no eRPM adds to that sum and never resets it. eRPM older than 200 ms is missing: exit 6 (100 ms locked on one missed 50 ms frame). `RpmPlausibilityGate` trip: exit 6. WARNING: a soft body does not stall the wheels; only a rear sensor or the spotter covers it |
| 8 | Slope, runs away downhill | Measured reverse speed above 0.5 m/s for 0.3 s: exit 6. First ground tests on level ground |
| 9 | Wheel slip, one wheel spins | A track's eRPM differs from its command by more than 50 % for 0.5 s (summed): exit 6. Slip reads long, so the robot stops early; the time cap bounds a slide |
| 10 | RC dropout | RC frame older than 0.3 s: exit 12 (write, no pivot until a fresh frame). The RC-stale and no-signal paths stay and now also write neutral and reset back-up |
| 11 | Transmitter off | ch3 failsafe reads as switch-off: disarm, exit 9 (the accepted re-arm residual in CLAUDE.md is unchanged) |
| 12 | Wi-Fi or phone dropout | Back-up needs no network. The phone STOP works when the link is up (exit 8) and must not be relied on. The RC and the signal are the stops |
| 13 | Pack-low during reverse | `_leave_autonomy_if_pack_guarded` ends FOLLOW_ME; `_apply_pack_limit` gives neutral at once in an autonomous mode. Unchanged |
| 14 | Service restart or crash mid-movement | The VESC stops by itself about 1 s after the last command (up to 0.33 m). After the restart: startup arm interlock, latch off, state OFF. Auto-deploy does not pull while armed |
| 15 | Clock step | Only `time.monotonic()` and capture stamps on the same clock. No `time.time()` in the new code |
| 16 | Forward lunge after the stop | SETTLE keeps forward at 0; release rule 3.4.5; reset 3.4.6; `_last_follow_me_emitted_forward_byte` stays None (section 5, item 3) |
| 17 | Sun, phantom corridor | The corridor is not an input to back-up. It gates forward as before |

### 4.1 The 0.8 m stop while reversing

Today the stop scales only the forward part (`is_forward_motion` in `Controller.process`). A reverse byte passes it untouched, so back-up enforces it itself.

1. A stop-tier trigger or an active hold ends BACKING and refuses a start. Back-up does not "back away" from a close person.
2. Why: inside about 1 m the box is clipped and the range is garbage (2026-11-08). The robot would reverse blind on a target it can no longer identify. A standing robot that Kevin walks up to harms nobody.
3. The whole band from 1.0 m down to 0.8 m goes to LOCKED with forward 0 (the follow-me pivot passes, Kevin 2026-11-25). `end_min_range_m` equals `person_stop_hold_release_m`, and SETTLE there would chatter on range noise.
4. Kevin's ruling of 2026-11-08 (a pivot passes the 0.8 m stop in FOLLOW_ME) is not changed. On 2026-11-25 he ruled the same for SETTLE and LOCKED. The limits in section 1 constrain that pivot; they do not remove it.

### 4.2 The person stop hold

1. Back-up never reverses while the hold is active. The only motion under a hold is the follow-me pivot, which normal FOLLOW_ME already makes under a hold. So the robot still does not drive the range itself, and the hold's plausibility bounds stay true. Do not widen them.
2. Back-up is never release evidence. No timer release, no corridor release, no `force_release`. `person_stop_hold_in_manual` stays False. The logic of `person_stop_hold.py` is not edited.
3. For the code change: its docstring line "neither mode commands reverse" becomes false. Correct the sentence to say that back-up never reverses under a hold.
4. A hold that engages during BACKING: exit 5. It releases only by its own rules.

## 5. Where it plugs in

1. New pure module `pi_app/control/backup_move.py`: states, gates, segment, speed law, limits. No hardware imports; recorded logs replay through it.
2. `pi_app/control/follow_me.py`: one read-only snapshot on `FollowMeController` (locked `track_id`, fresh flag, raw and filtered range, `depth_status`, box, `close_unknown`) and one reset method for 3.4.6 (`SpeedLayer`, velocity PID, `SafetyLayer`, `_cached_left`/`_cached_right` and `_last_output_time`). `compute()` keeps running so the tracker keeps its lock; its bytes are replaced. `_write_trial_record` gets the back-up state and the bytes after replacement.
3. `Controller.process`: call the module at the `_service_twitch_injection` point (after the per-mode command, before the obstacle layer), inside `try/except/finally`. The neutral write is in the `except` path only. `finally` may set LOCKED when the call did not finish; it never commands the motors, and it does nothing on success (a write there would cancel every BACKING command). The assignment at the bottom of `process` that stores `(left + right) / 2 - 126` into `_last_follow_me_emitted_forward_byte` must store None in BACKING, SETTLE and LOCKED (today it would overwrite a None set earlier with a negative value, review A4). The SETTLE release is decided before `_compute_follow_me_autonomy_command` on that tick.
4. Enable latch: `request_backup_enable()` and `POST /api/backup/enable` in `pi_app/web/oak_viewer.py`, 127.0.0.1 only, own lock and fields, nothing shared with `_twitch_*`. Volatile; needs armed. It clears on disarm, expiry (900 s), every FOLLOW_ME exit (ch4, web stop, vision stale, pack guard) and both early returns. The 5-move count belongs to the arm session: a second POST does not refill it. Enable is refused while the twitch latch is on, and `request_twitch_test` is refused while the back-up latch is on.
5. Early returns in `process` (RC stale, calibration, any other): when the state is not OFF, write neutral on that tick and reset back-up. The write is the last motor command of the tick.
6. `deactivate_follow_me` (web thread): sets a flag; the next tick writes neutral (exit 8).
7. `main.py`: feed `get_all_detections()` to the controller for gate 8, and the rear sensor state.
8. Telemetry age: use `left_status_age_s` / `right_status_age_s` for the 200 ms rule (today 0.5 s clears the eRPM).
9. Hard-stop slew bypass (`controller.py:2949`, 2973-2983): it copies `left_in`/`right_in` with no slew. After exit 7 with a held stick and the obstacle scale at the threshold, a track could go from reverse across neutral in one tick. Rule, per byte: a target on the other side of 126 from the slew state, or farther from 126 than the slew state, goes through `_slew_toward_target`; a target on the same side and closer to 126 may step at once. The same bypass passes the turn part in live code today: `docs/status_audit_2026-11-25.md`, section 2, item 6. A separate code change for it is being built on its own branch; back-up must land after it or carry the same rule.
10. Logging: a `backup` block in `pi_app/app/log_gating.py` (state, gate refused, exit cause, `d_keep`, `z`, bytes, segment, both distances) and journal lines for start and end.

Passed through unchanged after the injection point: obstacle layer (forward only), disarm neutral, charger inhibit, slew limiter at FOLLOW_ME rates with `snap_first_follow_me` False, `_apply_pack_limit`, `VescCanDriver.set_tracks`. Also unchanged: `update_safety`, `_leave_follow_me_if_vision_stale`, `_leave_autonomy_if_pack_guarded`, `_apply_safety_tier_override`, the hold, `get_person_detections()`.

Rules from CLAUDE.md, and where each one lands:
- Autonomous input must not cause harm; FOLLOW_ME keeps every protection: sections 0, 2, 3.3, 4.1, 4.2, question 6.
- 0.8 m absolute: 4.1. Hold, no timer or corridor release: 4.2 and 3.4.5 (evidence only).
- Twitch latch not reused: item 4. Slew never bypassed on an exit or release: section 1 and item 9.
- Stale list stays: failure 1. No heading target in MANUAL: back-up never runs in MANUAL and sets none.
- No ramp on the first command after re-arm: not proposed.
- `camera_fps` 15 not changed. Trail follow stays off (`odometry.wheel_speed_mps` counts reverse as 0; fix before both ever run together).

## 6. Config (new frozen dataclass `BackupConfig` in `config.py`)

| Name | Value | Note |
|---|---|---|
| `enabled` | False | AND gate with the runtime latch |
| `max_reverse_byte` | 32 | 0.33 m/s; clamped 0..40 at use |
| `range_gain_per_s` / `keep_min_m` | 0.8 / 2.0 | |
| `entry_min_range_m` / `entry_max_range_m` / `end_min_range_m` | 1.5 / 6.0 / 1.0 | |
| `entry_center_norm` / `exit_center_norm` | 0.20 / 0.30 | no steer in version 1 |
| `max_move_s` / `max_move_m` / `session_budget_m` | 8.0 / 2.5 / 6.0 | |
| `segment_yaw_band_deg` / `segment_credit` / `segment_margin_m` / `segment_max_age_s` | 3 / 0.5 / 0.3 / 30 | section 3.3 |
| `min_allowed_reverse_m` / `heading_max_age_s` | 0.5 / 0.2 | gate 11 |
| `rear_sensor_required` / `rear_sensor_fitted` | False / False | Kevin, 2026-11-25: ground runs supervised. While `required` is False the runtime latch is always needed. A fitted sensor always stops |
| `signal_max_age_s` / `det_max_age_s` / `rc_max_age_s` | 0.3 / 0.3 / 0.3 | |
| `erpm_max_age_s` | 0.2 | |
| `alone_s` / `track_stable_s` / `still_s` / `still_max_erpm` | 1.0 / 1.0 / 0.5 / 300 | |
| `rearm_rest_samples` / `rearm_s` | 5 / 2.0 | |
| `settle_range_gain_m` / `settle_polls` / `settle_run_s` / `settle_walk_mps` | 0.5 / 15 / 1.0 / 1.5 | 3.4.5 |
| `pivot_rc_max_age_s` | 0.3 | pivot in SETTLE and LOCKED |
| `jump_range_m` / `jump_center_norm` | 0.5 / 0.2 | |
| `stall_ratio` / `stall_s` / `overspeed_mps` / `overspeed_s` | 0.3 / 0.5 / 0.5 / 0.3 | sums, not streaks |
| `latch_max_s` / `latch_budget` | 900 / 5 | per arm session |

## 7. Test plan

WARNING: Before any step that can spin the wheels (steps 4 to 7), the assistant must ask Kevin whether the wheels are blocked, and wait for his answer. Ask every time, even if he answered earlier in the session.

WARNING: A push that changes follow-me code restarts `wall-e` and reaches the robot in 5 minutes. Tell Kevin before each such push. Each push that can move the robot needs a Grok safety review of the diff first.

1. **Unit tests** (`pi_app/tests/test_backup_move.py`): each gate alone refuses; each exit row gives its motor result; both bytes 126 or less on every BACKING tick over random inputs; no start on control-tick repeats; a segment restarts on yaw, pivot, stale heading and a forward track mismatch, ages out, and gets half credit; reverse never passes the segment; speed is 0 on every tick without a fresh "out" frame, and a new peak inside the bridge moves again; SETTLE holds through a far bias that walks in over 5 polls, through one missing poll and through a jump; no forward step on the SETTLE release tick; no byte above 126 on any tick until both slew bytes are at neutral after BACKING; the calibration and RC-stale returns write neutral and nothing follows the write; a successful module call is not followed by a neutral write; an exception writes neutral; an RC frame 0.3 s old gives exit 12 and no pivot; a frozen eRPM and a one-good-sample dropout both end the movement; the two latches refuse each other; a second POST does not refill the count; the hard-stop bypass never crosses neutral and never steps away from it (the 2026-11-05 held stick); `enabled` True with the latch off never moves the robot.
2. **Replay**, no motion: the follow-me drives of 2026-11-06 and 2026-11-08 (goats, the 0.66 s close-frame release), the 2026-09-25 frozen log, the 2026-09-24 pump and arms-up logs. Pass: zero would-start on every drive; the frozen log never passes gate 3.
3. **Log only on the robot**: no path to the motors. Journal lines `BACKUP would-start` / `would-stop`, with the segment length. Kevin works in front of the robot for 10 minutes (the hard cases of `docs/backup_pump_design.md` section 9, step 1), then signals 10 times at each of 2 m, 3 m and 4 m, each after a normal follow. Pass: no false start; 9 of 10 starts at 2 m; how often an allowed reverse of 0.5 m or more was there.
4. **Stand, wheels off the ground**, cap 20 bytes. Measure last "out" frame to wheels stopped (target 0.5 s or less); that no track turns forward before both have stopped; stick, ch4, ch3, web stop, calibration entry each to stopped; time cap; RC unplugged; a hand over the lens; a second person walks in.
5. **Rear sensor on the stand**, only if one is fitted: trigger it by hand during a movement, unplug it, short it. Each must end the movement and refuse the next start.
6. **Ground, short.** Not before step 4 passes (and step 5, if a sensor is fitted). Level garage floor, cap 20 bytes, `max_move_m` 1.0, a spotter beside the robot who sees behind it, Kevin with the transmitter in his hand, nobody and nothing behind the robot. With a sensor fitted, push a soft object (a foam block) against it during a movement.
7. **Ground, full**: 32 bytes, 2.5 m, level grass, same supervision as step 6; then one slope run. Check that a turn before the stop refuses the start, and that the reverse ends inside the driven strip (mark the forward track on the ground).
8. Supervision is never dropped by config. While `rear_sensor_required` is False, `enabled` alone never moves the robot: the per-session latch stays mandatory. Use without the latch needs a fitted, tested rear sensor (step 5), `rear_sensor_required` True, and Kevin's word.

## 8. Not in version 1

Steer in reverse, reverse beyond the straight segment, MANUAL, WAYPOINT_NAV, a second person in view, motion on the arms-up signal.

## 9. Open questions for Kevin

Kevin's decisions, 2026-11-25 (in chat):
- Q1: the pump only. Arms up runs log-only.
- Q3: the follow-me pivot passes in SETTLE and LOCKED (forward stays 0). This differs from the recommendation below.
- Q6: no rear sensor required; ground runs supervised (spotter who sees behind the robot, transmitter in hand). This differs from Grok's read and the recommendation below; both reads are kept for the record.
- Q2, Q4, Q5, Q7, Q8, Q9, Q10: accepted as recommended.
- Q11: OPEN (new in revision 3).

1. **Trigger.** Drop arms up for this feature and move only on the pump, after its 10-minute false-start trial? Arms up would still log for data. Recommendation (Grok's, and mine now): yes. Arms up reads "down" at exactly the range where you need the robot to move.
2. **After the stop.** Stay put until you are clearly seen 0.5 m farther away (5 good polls), or go back to the 1.2 m follow distance at once? Recommendation: stay put.
3. **Pivot while stopped.** In SETTLE and LOCKED, both tracks at neutral (no pivot to keep you centred)? You ruled on 2026-11-08 that the pivot passes the 0.8 m stop in FOLLOW_ME; these are new states, so the ruling does not settle it. Recommendation: neutral in version 1.
4. **Straight only.** Version 1 does not steer in reverse, so it does not keep you centred; if you drift off centre it stops. Accept for version 1? Recommendation: yes.
5. **Speed cap.** 32 bytes (0.33 m/s on the ground, the 0.4 you agreed in old units) or 39 bytes (a true 0.40 m/s)? Recommendation: 32.
6. **Rear sensor.** You accepted no rear sensors. Two reads:
   - Mine in revision 1: supervised use with the transmitter in hand is enough for the first ground runs; a rear sensor before the feature is on by default.
   - Grok's: that does not meet "autonomous input must not cause human harm". You cannot see the chassis shadow while you face the camera, and a foot or a small animal does not stall the wheels. A rear sensor and the straight-segment rule before ANY ground run.
   - Recommendation: Grok's. I have no concrete argument against it. Without a sensor the feature stays on the stand.
   - Options, as a proposal only (you buy; prices are rough and not checked): (a) a bumper bar on two microswitches to a GPIO, about 20 to 40 USD, senses contact only, works in sun and mud, simplest to make fail-safe. If one is ever fitted (Grok): two normally-closed switches in series, open = hit, trip on the first open sample, clear slowly, full body width, low enough to meet a leg before a track does; (b) two ToF sensors (VL53L1X class), about 15 to 25 USD each, sense before contact but reach only about 1 m in sunlight and would share I2C bus 1 with the GPS (lock-up history, `docs/i2c_bus_guard.md`); (c) a sealed ultrasonic sensor, about 10 to 100 USD, wide beam, weak on soft or angled targets. Recommendation: (a) first; add (b) or (c) later if you want a stop before contact.
7. **Distance cap.** 2.5 m and 8 s, or 3 m and 10 s? The straight segment will usually be the shorter limit. Recommendation: 2.5 m and 8 s.
8. **Session budget.** Keep the 6 m net-reverse cap on top of the segment rule? Recommendation: yes; it costs nothing.
9. **Animals in view.** Back-up refuses when any other person, dog, cat, goat, sheep, cow or horse is in the frame. With goats around this will refuse often. Recommendation: strict for version 1. (Chickens are not a stop class.)
10. **Too close.** Inside 1.0 m the robot stops and locks until you toggle FOLLOW_ME; it does not keep backing away. Recommendation: keep (Grok agrees).
11. **Pivot close to you (new, open).** You allowed the pivot in SETTLE and LOCKED. Grok asks for one limit on it: while you are inside 1.0 m, cap the pivot at about 8 bytes (a quarter of the normal 32), and hold it at 0 while your range is still falling (you are still walking in). Reason: a corner of a 0.82 m wide body swings toward a person who is stepping up to it. This limits the pivot you allowed; it does not remove it. Recommendation (Grok's, and mine): yes.

## 10. What the Grok check of the code diff must verify

1. No byte above 126 reaches `set_tracks` on any BACKING tick, or on any later tick until both slew bytes are at neutral; include `_apply_pack_limit` rounding and the slew limiter.
2. Every early return and the `except` path write neutral when the state is not OFF, the write is the last motor command of the tick, and no write happens after a successful module call.
3. Exit 12 fires at 0.3 s of RC age, and no pivot passes in SETTLE or LOCKED without a fresh RC frame.
4. The speed is 0 on every tick without a fresh "out" frame; the bridge keeps only the state.
5. The segment gets half credit, restarts on each cause in 3.3 item 2, and the reverse can never exceed it. The body length and overhang were measured and the margin re-checked.
6. The SETTLE release needs the 1.0 s, 15-poll run with the walking-speed bound; the reset in 3.4.6 runs before that tick's follow-me output is used; `_last_follow_me_emitted_forward_byte` is None in BACKING, SETTLE and LOCKED.
7. The stall and mismatch sums never reset inside a movement; eRPM age comes from the status frame age, limit 200 ms.
8. The latch: localhost only, cleared on every FOLLOW_ME exit, count not refillable, mutual refusal with the twitch latch, and mandatory while `rear_sensor_required` is False.
9. The hard-stop bypass follows the per-byte rule (section 5, item 9), or the separate fix has landed first.
10. `person_stop_hold.py`: only the docstring changed; no bound widened; back-up never calls `force_release`.
11. If Kevin says yes to question 11: the 8-byte cap and the hold-at-0 rule inside 1.0 m.

## 11. Grok safety reviews, 2026-11-25 (session 536379f5)

First review (revision 2):

| Finding | Handled in |
|---|---|
| A1. Steer at low reverse speed drove one track forward | 3.2 (no steer, both bytes 126 or less, asserted); state table; question 4 |
| A2. Blind reverse off the driven path | 3.3 (straight segment), gate 11, gate 12 and exit 6 (rear sensor), question 6, test steps 5 and 6 |
| A3. Calibration return left a reverse byte in the VESC | Exits 9 and 10; section 5 items 3 and 5; module runs while not OFF |
| A4. Emitted-byte overwrite and wind-up through SETTLE | Section 5 items 2 and 3; 3.4.6; trial log records the replaced command |
| A5. Hard-stop bypass snaps across neutral | Section 5 item 9, cross-reference to the status audit |
| A6. Frozen eRPM hides a stall; dropouts reset the timer | Failure 7 and 9; `erpm_max_age_s` 0.1; sums, not streaks |
| Latch: clear on every FOLLOW_ME exit, no refill, mutual refusal with twitch | Section 5 item 4 |
| Web stop contradiction | Exit 8, failure 12, section 5 item 6 |
| 1.0 m to 0.8 m band chatters | Exit 5, 4.1 item 3, question 10 |
| Pose crop identity; second person in view | 2.1 item 2, gate 4, gate 8, failure 3 |
| Session budget is not path memory | 3.4.4, question 8 |
| Hold docstring becomes false | 4.2 item 3 |
| Keep 0.3 s signal age | 2.1 items 3 and 4 |
| Section 9: Q1, Q2, Q6, Q8 | Questions 1, 2 and 3, 6, 10 |

Second review (revision 3):

| Finding | Handled in |
|---|---|
| 1. RC older than 0.3 s was not an exit | Exit 12, failure 10, pivot limit 2 in section 1 |
| 2. `finally` would write neutral on success; write must be last | Exit 10, section 1 ("write"), section 5 items 3 and 5 |
| 3. SETTLE release on a far bias | 3.4.5, config `settle_*` |
| 4. Forward wheel slip credits ground never covered | 3.3 items 1, 2 and 4; gate 11 (one number: allowed reverse 0.5 m) |
| 5. Reverse and forward tracks mixed on the first SETTLE or LOCKED tick | Pivot limit 1 in section 1 (stand before pivot) |
| 6. Test step 8 dropped supervision; fitted sensor must always stop | Test step 8, exit 6, config `rear_sensor_*` |
| Cheap rule 1: speed 0 through the pump bridge | Section 1, 2.1 items 1 and 4, 3.1 item 2, exit 1 |
| Cheap rule 3: pivot cap inside 1.0 m | Question 11 (open, Kevin's call) |
| Cheap rule 5: yaw band and overhang | 3.3 items 2 and 6 (plus or minus 3 degrees; length to measure) |
| eRPM age 200 ms | Exit 6, gate 10, failure 7, section 5 item 8 |
| Heading-stale time | 3.3 item 2, gate 11, `heading_max_age_s` |
| 4.1 item 4 and 4.2 wording after Kevin's pivot decision | 4.1 item 4, 4.2 items 1 and 3 |
| Bumper wiring guidance | Question 6, option (a) |
| Hard-stop bypass, per-byte rule | Section 5 item 9 |
