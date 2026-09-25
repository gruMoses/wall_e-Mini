# Next session: start here

Written 2026-09-20 (evening), at the end of the follow-me session.
Updated 2026-09-22 (evening): `fm-armsup` hardened after three Grok safety reviews and merged (sections 2 and 6).
Updated 2026-09-24 (evening): `fb9420e` deployed; the arms-up bench test PASSED on the test stand (section 2, "Bench result").
Updated 2026-09-24 (night): the log-only pump detector is built on branch `fm-pump` and NOT deployed (section 4, item 2). Housekeeping done (item 8).
This document uses Simplified Technical English where practical.

## 1. State of the robot

- 2026-09-22: `main` gains the `fm-armsup` merge (`b758f7a`, section 2). Before that, the robot ran `main` at commit `8eb6cc5` plus documentation-only commits.
- Kevin validated this build in the field at 18:09 on 2026-09-20. His words: "that was gorgeous".
- All changes from 2026-09-20 are live: tracking fix, steering retune, speed-safety logic, honest person-range sampler, vision-latency fix.
- The full record of the day is `docs/follow_me_run_2026-09-20.md`.
- The end goal and the owner's constraints are in `CLAUDE.md` and in the auto-memory file `project-walle-north-star.md`.

## 2. Branch `fm-armsup`: arms-up bench test (deployed 2026-09-24, bench test PASSED)

Purpose: bench test for the "both arms up" gesture. When the robot sees both wrists above the shoulders, it gives one small reverse pulse (22 bytes for 0.25 s, approximately 7 cm). This is the acknowledgement test that Kevin asked for. It is the first step toward the back-up feature (section 4, item 2).

WARNING: This feature commands motion from a camera gesture. Enable it only while Kevin is at the robot with the RC.

How the gate works:

- A deploy does not enable the pulse. Only a runtime latch enables it: `POST /api/arms_up/twitch_test` with the body `{"enabled": true}`. The POST is accepted only from the robot itself (127.0.0.1). The latch is refused unless the robot is armed in MANUAL.
- The latch clears on disarm, e-stop, mode change, calibration, RC stale, 5 minutes, or 3 pulses. A second enable while latched is refused. A restart clears the latch.
- A pulse starts only when the sticks are within 6 bytes of neutral, web teleop is off, the output was neutral for 0.5 s, both wheel eRPMs are present and at or below 300, and the 3 s cooldown is over.
- A stick input, disarm, e-stop, charger, pack-low, calibration or a mode change cancels a pulse. A cancelled pulse does not resume.
- The gesture needs 3 fresh pose samples over 0.4 s. After a pulse, the camera must see the arms down (3 samples) before the next pulse.
- MediaPipe Pose loads at startup and runs only while the latch is on. It does not run in FOLLOW_ME or WAYPOINT_NAV.

Procedure (bench test with Kevin):

1. Kevin approves the push to `main` in the chat while at the robot. Push. Confirm the merge on the Pi with `git merge-base --is-ancestor`. The service restarts when the robot is disarmed.
2. Garage. Clear floor behind the robot. Armed, MANUAL, sticks centred. Kevin stands 2.5 m to 3 m in front of the robot with the arms down.
3. Latency baseline: with the latch off, Kevin stays in view for 30 s.
4. Enable the latch from a shell on the robot:
   `ssh pi@192.168.86.54 'curl -s -X POST -H "Content-Type: application/json" -d "{\"enabled\": true}" http://127.0.0.1:8080/api/arms_up/twitch_test'`
5. Kevin raises both arms above the head for approximately 1 s. The robot must move back approximately 7 cm, one time. One arm up must do nothing. Arms down for 1 s, then both arms up again: one more pulse.
6. Read the arm log: the `arms_up` block (`twitch_count`, `twitch_blocked_reason`, `twitch_cancel_reason`, `pose_ms`, `pose_hz`) and `oak.det_latency_s` with the latch on, compared with the 30 s baseline.

CAUTION: MediaPipe Pose runs only while the latch is on. A latency reading taken while disarmed or with the latch off does not measure the pose worker.

If `det_latency_s` p50 increases by more than approximately 0.05 s with the latch on, reduce `pose_max_hz` before the back-up feature runs the worker in FOLLOW_ME.

Known limit (deferred): detection freshness compares host poll times, not capture times. The crop can lag the person by the NN latency (approximately 0.2 s). The result is a missed trigger, not a false one. Use capture timestamps before the back-up feature.

Bench result (2026-09-24, 18:35 to 18:40, robot on the test stand with the wheels off the ground, `fb9420e`):

- Deploy: `DEPLOYED 39044dc -> fb9420e` at 18:30:06, service restarted. MediaPipe Pose loaded at boot.
- The latch was enabled from the Pi shell at 18:35:16. Pulses at 18:37:07 and 18:38:26. Kevin: "it definitely twitched backwards".
- Each pulse: bytes 110, 104, 112, 126; peak eRPM approximately -3600; wheel travel approximately 9 cm (integrated eRPM, no load); wheels stopped approximately 0.6 s after the start.
- One arm up: 78 distinct one-arm poses in 60 s gave no trigger.
- Re-arm: the second pulse came only after the camera saw the arms down.
- Expiry at 18:40:16 cleared the latch (1 pulse unused), and pose stopped (`pose_hz` 0).
- Latency (armed, MANUAL, person in view): `det_latency_s` p50 0.179 s with the latch off and 0.182 s with the latch on. `vision_work_ms` p50 26.7 to 32.0. Control loop unchanged. Pose inference approximately 75 ms per frame at 10 Hz (static-image mode).
- Camera geometry: at 2.5 m to 3 m, the shoulders are at 0.09 to 0.13 of the preview height and the NN box is clipped at the top. Raised hands are out of view at that range. The trigger fired only when Kevin stood farther back.
- Not tested: a stick cancel during a pulse, the third-pulse budget cutoff, and travel on the ground.

## 3. Rules learned on 2026-09-20 (obey these)

- Identity gates decide whom to follow. They must never hide a closer range from the speed command.
- A rule must never discard a CLOSE range. A veto is permitted only in the far direction.
- Do not fit a plant model from `imu.yaw_rate_dps`. Use the heading derivative. The analyzer does this.
- Count the zeros and the duplicates in a logged signal before you fit a model to it.
- `OakDetectionConfig.camera_fps = 15.0` is load-bearing. Measure `oak.det_latency_s` before and after each change to the OAK pipeline. The measurement works at idle. A field walk is not necessary.
- A diagnosis from a code review is a hypothesis. `nn_input_queue_size = 1` was "the bug" in review, and it changed nothing. The camera frame rate was the cause.
- Get a Grok safety review of each motion-safety diff before the push. The first review on 2026-09-20 found four real defects after 980 tests passed.
- Workflow that worked: diagnosis on the session model, `grok_second_opinion` on the plan, implementation by headless Grok CLI jobs in separate git worktrees with a written specification, diff review, full test suite, Grok safety review, then Kevin approves the push.
- Kevin wants short replies. Give the instruction first.
- (2026-09-22) Measure a code path in the state where it runs. The pose worker runs only while the bench latch is on, so a disarmed "idle" latency reading does not measure it.
- (2026-09-22) Give `grok_review` exact line ranges for a scoped re-review. An open focus used all 25 turns on reading and gave no verdict.

## 4. Backlog, in priority order

0. **INCIDENT 2026-09-25: follow-me drove on a frozen detection. Do not use follow-me until the fix (deploy 1 below) is live.**
   - At 17:43 the OAK colour camera (CAM_A: YOLO input and preview) stalled on the device after 23 h of uptime. The mono cameras and the depth path continued at 15 fps. Chip temperature was 77 C to 78 C before the stall; a thermal cause is not likely.
   - `get_person_detections()` returned the last list (one person, x -0.8, z 2.52 m, track 453) for 40 minutes. `get_health()` reported `detections_stale: True`, but no code read it.
   - At 18:23 Kevin selected FOLLOW_ME. The frozen list counted as a target, so the entry was accepted. Follow-me drove approximately 0.85 m/s with a constant left steer for 44 s (a circle), and again for 1.7 s and 2.2 s. Log: `arm_20260925_182305.log`.
   - The service restart at 18:30 recovered the camera (`det_fps` 14.5, detection age 0.05 s).
   - Also found: the robot stayed armed from 2026-09-24 18:46 to 2026-09-25 18:07 (arm switch on, transmitter on). The auto-deploy held all night for that reason.
   - Fix plan (Grok second opinion accepted): deploy 1 adds a capture-time freshness clock with sequence tracking in the reader, and a controller gate. The gate refuses FOLLOW_ME entry and leaves FOLLOW_ME on the same tick when the detections are older than 1.5 s. The reader return values do not change: an empty list means "detection gap" (coast), and a stale close person must keep forcing the stop tier. Deploy 2 (later, separate) restarts the OAK session when the colour stream is stale for 5 s with depth fresh, with a retry cap and a fault latch.
   - Branch: `fix-vision-stale`.

1. **`fm-armsup` bench test.** PASSED 2026-09-24 on the test stand (section 2). Optional: one pulse on the ground to measure the real travel.
2. **Back-up feature.** Both arms up while Kevin walks toward the robot: the robot reverses, keeps Kevin centred, and keeps its distance. Agreed: trigger is both arms raised (the camera cannot see palms or fingers beyond 0.5 m); reverse speed cap 0.4 m/s; the robot stops when the arms go down (approximately 0.3 s); a maximum distance for each reverse movement; the RC always overrides; there are no rear sensors and Kevin accepts that. Skid-steer yaw sign does not depend on the travel direction, thus the keep-centred steering law stays the same. Write a design first, then get a Grok second opinion and a Grok safety review.
   CAUTION (bench test, 2026-09-24): with the camera as mounted, raised hands leave the frame when Kevin is closer than approximately 3.5 m to 4.5 m. That is the range where the back-up feature must work. Thus the raised-arms trigger is dropped.
   Design draft (2026-09-24): `docs/backup_pump_design.md`. The trigger is a low two-hand "pump" that the person box width shows at all ranges (tested the same evening: 3 to 3.7 x the rest width). Grok reviewed the draft and its eight corrections are in it. The box-based design does not use the pose crop, so the capture-time and largest-box limits of the arms-up worker do not apply. Kevin accepted the four decisions in section 10 of the design.
   Step 1 status (2026-09-24): the log-only detector is built on branch `fm-pump` (`dfbda57`): `pi_app/control/pump_gesture.py`, a `pump` log block, and the journal lines `PUMP would-start (log only)` / `PUMP would-stop (log only)`. 1164 tests pass. Grok review: no path to the motors or the mode, and no loop-stall risk. NOT deployed: Kevin said "not tonight".
   Procedure for the trial:
   1. Kevin approves the push in the chat. Merge `fm-pump` to `main` and push. The service restarts once while the robot is disarmed.
   2. Park the robot. Arm it in MANUAL with the sticks centred. CAUTION: the per-tick log runs at 10 Hz only while the robot is armed. Disarmed, it writes one line every 5 s, which is too slow for the trial.
   3. Kevin works in front of the robot for 10 minutes. Include the hard cases in design section 9, step 1: hands on hips, a carried board, bending, turning, walking toward the robot, a second person or an animal if possible.
   4. Kevin pumps 3 to 5 times each at approximately 2 m, 3 m and 4 m.
   5. Score the would-start journal lines and the `pump` log block. The raw `detections` are in the same log, so other detector variants can be replayed offline.
3. **Video recording is off.** The H.265 encoder is skipped when gestures are configured (`pi_app/hardware/oak_depth.py`, the condition `hand_queues is None` near the `VideoEncoder` block; journal line `OAK recorder: no recording queues available`). Kevin wants recording back. The comment says "ISP limits", but the hand stream replaces the preview stream, thus the count of camera outputs is the same as in the gestures-off configuration. Plan: a config flag, `fps=` on the 1080p output, encoder preset at the camera frame rate, then an idle A/B of `det_latency_s`, `det_fps` and `depth_fps`. Revert if latency increases.
4. **Hand gestures.** Decision: fix them, do not remove them. The RC stays the authority for now. Fix order: (a) mode ownership between RC channel 4 and gestures (an RC-started follow-me must put the state machine in phase ACTIVE so that the stop palm is checked; a gesture start must survive or be refused with a logged reason); (b) count the hold on new hand results, not on 30 Hz control ticks; (c) range: a hand crop from the person box at a higher resolution, or the on-device hand models. Refer to `docs/gesture_review_2026-09-20.md`.
5. **Phantom obstacle on gravel.** In the 15:49 run the corridor read 0.5 m to 0.9 m for 3 s with 9 to 11 percent valid pixels and nothing in front. The 18:09 run had throttle on 3 percent of ticks only. Recording (item 3) gives the footage that this item needs.
6. **Steering gain.** Left at `pid_lateral_kp` 0.5 and cap 32 on purpose. The simulation shows margin, but at forward 110 the outside wheel is at the mixer limit, thus more steer cap is inside-wheel braking. Optional experiment with Kevin present: `POST /api/follow_me/params {"pid_lateral_kp": 0.6}`. It needs no deploy and reverts in seconds.
7. **Lidar.** Recommendation: Slamtec RPLIDAR S2L (IP65, 80 klux, approximately 299 USD). Refer to `docs/lidar_selection_2026-09-20.md`. Verify the prices before an order. Do not start the integration before Kevin has the unit. Kevin also plans a T-bar handle to ride on the back axle: a rider changes the mass, the stopping distance and the yaw plant, thus measure them again.
8. **Housekeeping.** DONE 2026-09-24 with Kevin's approval: all merged worktrees and local branches removed (only `main` and `fm-pump` remain); the two July review documents moved to the macOS Trash; 12 merged branches deleted from GitHub. Restore one with `git push origin <sha>:refs/heads/<name>`: `697a8d5 codex/replace-bare-except-with-logger.exception`, `1c6b80a docs/bms-fail-open-memo`, `16c1ac3 docs/gearing-memo`, `de4cb44 feature/property-map`, `7dc8466 fix/gps-align-lock-hardening`, `6950e14 fix/oak-imu-heading-hardening`, `c7b1734 fix/oak-imu-producer-integration`, `a2628f0 fix/spp-writer-retirement`, `7e6d8dd fix/stream-client-caps`, `dd0f366 fix/ups-startup-hardening`, `d2ba2b0 fm-armsup`, `91f4bb7 review/temperature-debug-graph`. Kept: `research/follow-me-trail-strategies` and `codex/initialize-_last_imu_update-in-controller` (not merged).

## 5. Known residual risks in the live build (from the second safety review)

All of them end in a stop, or they last 1 s or less.

1. A person at approximately 2.5 m with no stereo range and a box that is 0.85 of the frame height or more: the robot does not move closer until the range comes back.
2. A lone detection with an unknown range (for example a chicken) can continue the lock after the operator leaves the frame: the robot continues at a speed that does not increase for a maximum of 1 s, then it stops.
3. A close person whose box is short and whose range is unknown: the speed holds (it does not increase) for a maximum of 1 s.
4. The range status `ambiguous` can occur frequently beyond approximately 5 m, because one pixel of disparity is approximately 1 m at that distance.

## 6. Status of `fm-armsup` at the end of the 2026-09-22 session

- Commits: `d23c252` (detector and twitch), `b36db07` (hardening after review 1), `d2ba2b0` (fixes after review 2). Merge commit on `main`: `b758f7a`.
- Full test suite: 1137 tests, OK (4 skipped).
- Grok safety review 1 found 11 defects: twitch on by default; no RC override during the pulse; a twitch in FOLLOW_ME; a fake hold across a calibration gap; a pulse that resumed after charger or pack-low; the model load on the first armed frame; re-arm after a detection dropout; the unmapped crop; tracking on a moving crop; pose load in follow-me; a stop race.
- Grok safety review 2 found 3 bounded holes: a LAN client could refill the latch; the still check passed with no eRPM; a reverse byte could stay in the VESC across calibration.
- Grok safety review 3: no must-fix item for a supervised bench test.
- The largest confident person box is still the pose target, not the follow-me lock. This is acceptable for the MANUAL bench test only. The back-up feature must use the locked target.
- Idle baseline before the merge (2026-09-22 17:04, disarmed, `39044dc`): `det_latency_s` p10/p50/p90 0.174/0.197/0.219 s, `det_fps` 15.0, `vision_work_ms` 8.7.
- Bench test and latency with the latch on: done 2026-09-24, PASSED (section 2, "Bench result").

## 7. Session costs and tools (for the budget check)

- Grok CLI implementation jobs on 2026-09-20: approximately 12 USD of xAI credit in total. The documentation job was the most expensive (2.29 USD, 46 turns): give documentation jobs less to read next time.
- The 2026-09-20 session model was Fable 5.1. The 2026-09-22 session model was Opus 5.5.
- Grok on 2026-09-22: review 1 0.60 USD; second opinion 0.03 USD; implementation round 1 2.94 USD (42 turns); review 2 0.90 USD; implementation round 2 1.69 USD (39 turns); review 3 0.89 USD (ran out of turns, no verdict) and 0.22 USD (scoped retry). Total approximately 7.30 USD.
- A push to `main` needs Kevin's approval in the chat. The auto-mode classifier blocks an unapproved push as a production deploy. That is the correct behaviour.
