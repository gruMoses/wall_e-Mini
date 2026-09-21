# Next session: start here

Written 2026-09-20 (evening), at the end of the follow-me session.
This document uses Simplified Technical English where practical.

## 1. State of the robot

- The robot runs `main` at commit `8eb6cc5` plus documentation-only commits.
- Kevin validated this build in the field at 18:09 on 2026-09-20. His words: "that was gorgeous".
- All changes from 2026-09-20 are live: tracking fix, steering retune, speed-safety logic, honest person-range sampler, vision-latency fix.
- The full record of the day is `docs/follow_me_run_2026-09-20.md`.
- The end goal and the owner's constraints are in `CLAUDE.md` and in the auto-memory file `project-walle-north-star.md`.

## 2. Work that is NOT on `main`

### Branch `fm-armsup` (pushed to origin, NOT merged, NOT deployed)

Purpose: bench test for the "both arms up" gesture. When the robot sees both wrists above the shoulders for 0.4 s, it gives one small reverse pulse (22 bytes for 0.25 s, less than 7 cm). The cooldown is 3 s. This is the acknowledgement test that Kevin asked for. It is the first step toward the back-up feature (section 4, item 2).

WARNING: This branch commands motion from a camera gesture. Do not merge it to `main` while Kevin is absent. `main` deploys to the robot automatically.

Status at the end of the session: refer to section 6 of this document.

Procedure for the next session:

1. Read the diff of `fm-armsup` against `main`.
2. Run the full test suite on the branch.
3. Run a Grok safety review of the diff (`grok_review`, focus: unintended motion, twitch outside the safety chain, pose worker effect on vision latency).
4. Correct all findings.
5. Ask Kevin for approval. Merge and push only when Kevin is with the robot.
6. After the deploy, read `oak.det_latency_s` at idle. It must stay near 0.20 s. If it does not, move the pose worker to a lower rate or revert.
7. Bench test with Kevin: armed, MANUAL, garage. Both arms up. The robot must move back less than 3 inches, one time. One arm up must do nothing.

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

## 4. Backlog, in priority order

1. **`fm-armsup` bench test.** Refer to section 2.
2. **Back-up feature.** Both arms up while Kevin walks toward the robot: the robot reverses, keeps Kevin centred, and keeps its distance. Agreed: trigger is both arms raised (the camera cannot see palms or fingers beyond 0.5 m); reverse speed cap 0.4 m/s; the robot stops when the arms go down (approximately 0.3 s); a maximum distance for each reverse movement; the RC always overrides; there are no rear sensors and Kevin accepts that. Skid-steer yaw sign does not depend on the travel direction, thus the keep-centred steering law stays the same. Write a design first, then get a Grok second opinion and a Grok safety review.
3. **Video recording is off.** The H.265 encoder is skipped when gestures are configured (`pi_app/hardware/oak_depth.py`, the condition `hand_queues is None` near the `VideoEncoder` block; journal line `OAK recorder: no recording queues available`). Kevin wants recording back. The comment says "ISP limits", but the hand stream replaces the preview stream, thus the count of camera outputs is the same as in the gestures-off configuration. Plan: a config flag, `fps=` on the 1080p output, encoder preset at the camera frame rate, then an idle A/B of `det_latency_s`, `det_fps` and `depth_fps`. Revert if latency increases.
4. **Hand gestures.** Decision: fix them, do not remove them. The RC stays the authority for now. Fix order: (a) mode ownership between RC channel 4 and gestures (an RC-started follow-me must put the state machine in phase ACTIVE so that the stop palm is checked; a gesture start must survive or be refused with a logged reason); (b) count the hold on new hand results, not on 30 Hz control ticks; (c) range: a hand crop from the person box at a higher resolution, or the on-device hand models. Refer to `docs/gesture_review_2026-09-20.md`.
5. **Phantom obstacle on gravel.** In the 15:49 run the corridor read 0.5 m to 0.9 m for 3 s with 9 to 11 percent valid pixels and nothing in front. The 18:09 run had throttle on 3 percent of ticks only. Recording (item 3) gives the footage that this item needs.
6. **Steering gain.** Left at `pid_lateral_kp` 0.5 and cap 32 on purpose. The simulation shows margin, but at forward 110 the outside wheel is at the mixer limit, thus more steer cap is inside-wheel braking. Optional experiment with Kevin present: `POST /api/follow_me/params {"pid_lateral_kp": 0.6}`. It needs no deploy and reverts in seconds.
7. **Lidar.** Recommendation: Slamtec RPLIDAR S2L (IP65, 80 klux, approximately 299 USD). Refer to `docs/lidar_selection_2026-09-20.md`. Verify the prices before an order. Do not start the integration before Kevin has the unit. Kevin also plans a T-bar handle to ride on the back axle: a rider changes the mass, the stopping distance and the yaw plant, thus measure them again.
8. **Housekeeping.** Remove the merged worktrees and branches under `.claude/worktrees/` (`fm-tracking`, `fm-steer`, `fm-depth`, `fm-coast`, `fm-latency`, `fm-docs`, `fable-follow-fixes`, and the older ones that `git worktree list` shows as prunable). Keep `fm-armsup`. The untracked files `docs/code-review-wave3-wave4.md` and `docs/code-review-wave5-debug-tab.md` are from an earlier session: ask Kevin before you commit or delete them.

## 5. Known residual risks in the live build (from the second safety review)

All of them end in a stop, or they last 1 s or less.

1. A person at approximately 2.5 m with no stereo range and a box that is 0.85 of the frame height or more: the robot does not move closer until the range comes back.
2. A lone detection with an unknown range (for example a chicken) can continue the lock after the operator leaves the frame: the robot continues at a speed that does not increase for a maximum of 1 s, then it stops.
3. A close person whose box is short and whose range is unknown: the speed holds (it does not increase) for a maximum of 1 s.
4. The range status `ambiguous` can occur frequently beyond approximately 5 m, because one pixel of disparity is approximately 1 m at that distance.

## 6. Status of `fm-armsup` at the end of the session

- Branch commit: `d23c252` on `fm-armsup` (one commit on top of `main`).
- Full test suite on the branch: 1038 tests, OK (4 skipped). `main` has 1011 tests.
- Implemented: `pi_app/control/arms_up.py` (pure detector and debounce), `pi_app/hardware/pose_worker.py` (MediaPipe Pose lite on the person crop, own thread, armed only, maximum 10 Hz, no new camera output), the twitch in `pi_app/control/controller.py`, the per-tick `arms_up` log block, and the tests.
- Verified on the robot: MediaPipe Pose lite loads on the Pi 5 and takes approximately 40 ms for each 640x480 frame. The model file downloaded to the Pi one time.
- NOT done: Grok safety review. Field test. Latency check with the pose worker active.
- Known design points to examine in the review:
  1. `ArmsUpConfig.twitch_test_enabled` defaults to `True`. A merge makes the twitch live immediately.
  2. In MANUAL, the twitch replaces the RC stick command for 0.25 s.
  3. The twitch is injected before the obstacle scale, the disarm override, the charger inhibit, the pack-low override and the slew limiter. The forward obstacle layer does not gate reverse motion (this is the existing behaviour).
  4. The pose worker uses the largest confident person box, not the locked follow-me target. A second person with raised arms can trigger the twitch.
  5. The 640x480 frame and the 640x352 detection frame have different crops. The worker applies the normalised person box to the 640x480 frame, as the recorder does. Examine the vertical offset before you trust the wrist and shoulder positions near the box edges.

## 7. Session costs and tools (for the budget check)

- Grok CLI implementation jobs on 2026-09-20: approximately 12 USD of xAI credit in total. The documentation job was the most expensive (2.29 USD, 46 turns): give documentation jobs less to read next time.
- The session model was Fable 5.1. Kevin's budget for Fable was tight that week. The global rule is Opus 5 as the orchestrator: ask Kevin to change the model at the start of the next session.
- A push to `main` needs Kevin's approval in the chat. The auto-mode classifier blocks an unapproved push as a production deploy. That is the correct behaviour.
