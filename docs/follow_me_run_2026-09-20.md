# Follow-me field record (2026-09-20)

This document records the four follow-me runs of 2026-09-20. Reproduce
the plant and depth numbers with `python3 tools/analyze_follow_me_log.py <log>`.

## 1. 14:58 run (round 3) — plant and tracklet

Log: `logs/arm_20260920_145718.log` (31 s, git `854611f`).

### 1.1 Evidence

1. The 2026-09-19 retune uses 0.22 deg/s per L-R byte. That number is
   wrong. The regression uses `imu.yaw_rate_dps`. 64 percent of those
   samples are exactly 0.0 (1,186 of 1,833 ticks on 2026-09-19; 172 of
   270 on 2026-09-20). The derivative of `imu.heading_deg` against L-R
   gives 0.65-0.71 deg/s per byte, r = 0.89-0.93, lag about 0.45 s, on
   both days.
2. The zeros come from `OakImuReader._consume_producer_yaw`. The
   duplicate branch reports the live rate only when `age_s` is at most
   max(0.05, 3 * cadence). Under follow-me load the IMU snapshot age on
   duplicate reads is p50 0.090 s, p90 0.182 s, max 0.286 s, and the
   vision thread then drains the IMU at about 6-10 Hz. The logged rate
   is 0.0 while `gy_body_dps` is 18-20 deg/s. The heading stays correct
   because the producer integrates every packet.
3. The 2026-09-19 retune raises loop gain 3.2x (`pid_lateral_kp` 0.4 ->
   0.8, steer caps 25/18 -> 40). The robot weaves (person x: +0.8, -0.9,
   +1.0, -1.5, +1.7 m at about 4.5 s) and overshoots a 28 deg bearing
   into a 120 deg swing at 45-58 deg/s.
4. Offline sim (0.70 deg/s/byte plant, 0.15-0.30 s dead time, 0.30 s
   lag, 6-8 Hz detections): 09-19 gains diverge (peaks 15, -20, 21, -29,
   55 deg). kp 0.5 / max 32 / slew 0.25 is damped (peaks 15, -8).
5. Five hard stops in 31 s, one person, conf 0.9. Tracklet id churns
   266→272→271→273→275→276→277 (box 0.10 frame wide at 4 m + 0.12-0.35 s
   gaps + ego yaw 20-35 deg/s ⇒ IoU 0). The 09-19 anti-steal rule reads
   each new CONFIRMED id as a different person → 1.0 s hand-off → speed 0.

### 1.2 Changes (git `41f4fdf`)

1. IMU duplicate-rate freshness bound is now max(0.20, 3 * cadence).
2. Steer retune: `pid_lateral_kp` 0.8 -> 0.5, both steer caps 40 -> 32,
   `steer_slew_per_tick` 0.1 -> 0.25. `pid_lateral_kd`,
   `steer_edge_boost`, `steer_edge_knee`, and `direct_turn_speed_*` stay
   unchanged.
3. The analyzer derives yaw rate from the heading derivative (lag 0-6
   ticks) and labels the logged-rate fit unreliable when samples are 0.0.
4. Centre-distance fallback in `TrackletTracker`
   (`tracklet_center_gate_*`) and a single-candidate rebind in
   `TargetTracker._find_committed` (exactly one candidate, seen ≤ 0.5 s
   ago, base 0.6 m depth gate).

Never fit a plant from `yaw_rate_dps`. Use the heading derivative.
Round 3 is validated for the first 50 s of the 15:49 run: 100 percent
fresh, no hard stops, no weave, id flip bridged. Plant re-measured
0.67 deg/s/byte, lag 0.46 s.

## 2. 15:49 run (round 4) — near-run-over

Log: `logs/arm_20260920_154844.log` (77 s, git `41f4fdf`).

### 2.1 Evidence (t = 74.9-76.2 s)

Operator report: "it almost tried to run me over". The operator stands
about 1.2 m away at the right frame edge. The box is full height
(ymin 0.00, ymax 1.00), bbox x [0.82, 1.00]. Stereo reads 1.2, 1.2,
1.2, 1.1, 1.0, 1.0, 0.8 m. Those readings are TRUE.

| Time (s) | Operator range | Robot speed | Speed command (bytes) |
|---|---|---|---|
| 74.9 | 2.6 m (last accepted) | 0.88 m/s | 54 |
| 75.1–75.3 | 1.2 m | 0.8 m/s | 69 → 84 → 101 |
| 75.8 | 1.0 m | 0.4 m/s | 0 |
| 76.2 | 0.8 m | 0 | safety stop fires |

The depth-continuity gate reads the closer range as an occluder. The
target goes not-fresh at a held 3.0 m. Speed ROSE 54.1 -> 68.9 -> 84.1
-> 100.7 bytes in 0.4 s because `direct_turn_speed` scale applies only
on fresh ticks. Decay reaches 0 at 1.0 m. The 0.8 m stop tier fires
after that.

### 2.2 Misdiagnosis and principle

The first diagnosis treats the 0.8-1.2 m readings as a frame-edge
stereo artifact, judged from box WIDTH. A box clipped by the frame
edge does not give range from its width. A height veto specced from
that diagnosis would have hidden a TRUE close range. The operator
report corrects it.

WARNING: Do not build a rule that can discard a CLOSE range.

Identity gates decide whom to follow. They never hide a closer range
from the speed command.

### 2.4 Fixes (git `ef6cdbf`)

- **S1.** Speed uses min(locked `target.depth_m`, nearest raw in-range
  person). Config: `nearest_person_speed_limit_enabled` (default True),
  `nearest_person_margin_m` 0.3. Telemetry: `nearest_person_m`,
  `speed_depth_m`.
- **S2.** Blind ticks never accelerate. Not-fresh ticks cap at the last
  fresh post-turn-scale request, then decay.
- **`close_unknown`.** Unknown range + bbox height ≥
  `close_unknown_bbox_height` 0.85 forces speed to `follow_distance_m`
  (forward 0).
- **Depth-unknown coast.** When `depth_status` is not `"ok"` (`z_m` =
  0), the tracker may continue an existing lock on bbox x and hold the
  last known depth for at most `target_depth_coast_max_s` 1.0 s.
- **Sampler.** Torso band, shifted off a clipped edge, pixels 0.30–9 m,
  support floor 12 px / 2 percent. Statuses: `ok`, `no_support`,
  `ambiguous`, `far_veto`, `no_frame`, `height_veto` (off:
  `person_depth_height_veto_ratio` 0.0). Other statuses publish `z_m` 0.

### 2.5 Two independent safety reviews

Review 1 of the merged S1/S2/sampler diff finds four holes after a
local review and 980 green tests. Review 2 confirms those holes closed.

1. **H1.** Depth-unknown coast drives on a held far range while FRESH,
   so S2 never runs; an edge-clipped close person with no stereo is
   invisible to nearest-person. Fix: no-accelerate cap on coast ticks;
   `close_unknown` zeros forward speed.
2. **H2.** Sampler discards the person's own pixels below
   `min_distance_m` 0.5 m and publishes background (`H2a`); a
   full-height box with a far median is background past a close person
   (`H2b` `far_veto`); a bimodal ROI is not a measurement (`H2c`
   `ambiguous`).
3. **H3.** S1 compares against `raw_depth_m`, so after the tracker
   accepts 1.2 m, `DepthFilter` still holds 3.0 m for about 5 fresh
   frames and the limit switches off. Fix: compare against
   `target.depth_m`.
4. **H4.** S2 caps at the last fresh REQUEST; a `SafetyLayer` ramp in
   progress (54 of a requested 100) keeps climbing 54 → 69 → 84. Fix:
   not-fresh AND depth-unknown-coast ticks cap at min(last fresh
   request, previous tick's post-safety speed).

### 2.6 Known residuals (review 2; fail-safe or ≤ 1 s)

1. A person at about 2.5 m with unknown stereo and a ≥ 0.85-high box
   zeros speed until stereo returns.
2. Unknown-depth boxes skip the animal-height rule and can lone-rebind
   → up to 1 s at non-rising speed toward that box.
3. A short-box close person with unknown stereo coasts ≤ 1 s.
4. `ambiguous` may stutter beyond about 5 m (1-px disparity ≈ 1 m).
5. Corridor phantom at 57–59 s on the gravel drive (p5 0.5–0.9 m,
   9–11 percent valid). Not addressed.

## 3. 17:11 validation

Log: `logs/arm_20260920_171057.log` (105 s, git `ef6cdbf`). VALIDATED.
Operator: "way better ... almost Black Mirror ... I love it". Every
close encounter (person ≤ 1.7 m, including four side-steps to the
frame edge) has forward speed 0. `close_unknown` fires 4 times.
`tracker_reject` is only ever `no_candidates`. Speed cap hits 7 percent
(was 27 percent on 14:58). `depth_status`: ok 776 / ambiguous 39 /
no_support 35.

## 4. Vision latency

Person detections reach steering about 0.6 s old before this work
(more than half of the steering loop's about 1.1 s dead time).

### 4.1 Two independent measurements

1. Log field `oak.det_latency_s`: host age of the NN message
   (`time.monotonic()` minus DepthAI `getTimestamp()`). Works at IDLE:
   push a camera change, then read `logs/latest.log`.
2. Cross-correlate IMU yaw rate (`d heading/dt`) against bbox-centre
   motion. Lag is at best |r|. Slope should be about −1/70 frame/deg.
   On the three earlier 09-20 runs: 0.60-0.65 s (r −0.78 to −0.88).

Idle before the fix: `det_latency_s` 0.60 s, `det_fps` 12.5,
`depth_fps` 11.5, `vision_work_ms` 18. Under follow-me load: p50 0.80 s
at `det_fps` 8.

### 4.2 What does not fix it, and what does (git `8eb6cc5`)

`nn_input_queue_size = 1` alone changes nothing (0.60 → 0.60 s). Host
queues are already maxSize=1 nonblocking. The delay is on-device:
sensors free-run at about 30 fps while the Myriad sustains about 12
inferences/s, so frames queue in camera/ISP pools ahead of the NN.

What does: `OakDetectionConfig.camera_fps = 15.0` — pass `fps=` to
every `Camera.requestOutput` (colour NN, colour preview/hand, both
monos). Also shipped: `vision_deadline_sleep` (the loop slept 67 ms
unconditionally; work 60 ms + sleep 67 ms = 8 Hz),
`poll_detections_first`, MediaPipe Hands not run in RC-started
FOLLOW_ME (`hand_poll_in_follow_me` default False).

WARNING: Do not remove `camera_fps = 15.0` without a new measurement of
`oak.det_latency_s`. `nn_inference_threads` 2 → 1 is unused and is not
tried.

Idle before / after:

| Field | Before | After |
|---|---|---|
| `det_latency_s` | 0.60 s | 0.197 s |
| `det_fps` | 12.5 | 15.0 |
| `depth_fps` | 11.5 | 15.0 |
| `vision_work_ms` | 18 | 10 |

Depth health is unchanged (`corridor_support_px` about 8800,
`depth_valid_pct` 6.6).

### 4.3 Corrected motor-to-yaw lag

The analyzer's motor→yaw lag drops from 0.47 s to 0.12 s (slope
unchanged, 0.62 deg/s/byte, r 0.89 on the 18:09 run). The drivetrain
does not change. IMU heading is drained on the vision thread, so most
of the old "0.47 s actuator lag" is HEADING latency from a blocked
vision loop (IMU sample age 90-290 ms), not the motors. Real plant lag
is about 0.1-0.2 s. Re-fit before any gain change.

## 5. 18:09 validation

Log: `logs/arm_20260920_180915.log` (76 s, git `8eb6cc5`). VALIDATED.
Operator: "that was gorgeous ... really really good". Under follow-me
load: `det_latency_s` p10/p50/p90 0.17/0.20/0.22, `det_fps` 15,
`vision_work` 10.6 ms. Fresh 100 percent. `tracker_reject` none.
Track-id changes 0 (was 4-6). Same-track range jumps >1 m: 0 (was 13).
`depth_status` ok 641 / ambiguous 5. Time with |bearing| > 25 deg:
1.6 s (was 15.7 s on 17:11). Speed cap 5 percent. Obstacle throttle
3 percent of ticks (was 17 percent). 63 ticks with a person < 1.7 m;
max speed command among them 0.0.

## 6. Open items

1. Hand gestures do not work as deployed. See
   `docs/gesture_review_2026-09-20.md`.
2. Video recording is suppressed whenever gestures are configured
   (`CLAUDE.md` Known Issues).
3. Round-4 residuals in section 2.6, including the gravel-drive
   corridor phantom.
4. `detect_min_person_height_m` 1.20 may still be too aggressive after
   the 2026-07-26 VFOV correction (needs
   `tools/replay_follow_me_log.py`). OAK chip 75-79 °C on the 14:58 run
   is not re-checked as a limit.
## 7. Decisions deliberately NOT made

Steering gain stays at `pid_lateral_kp` 0.5 / steer cap 32 after a
second opinion: at forward 110 over neutral 126 the mixer clips at
245, so extra steer cap is inside-wheel braking, not yaw authority;
n = 1 good latency run (18:09); the man-gate goal wants lateral
precision, not turn authority. `nn_inference_threads` stays 2.
`person_depth_height_veto_ratio` stays 0.0. Unarmed gesture control is
later, behind its own safety review.
## Appendix: Technical Names
FOLLOW_ME, IMU, L-R differential, `pid_lateral_kp`, `max_steer_offset_byte`,
`nearest_person_speed_limit_enabled`, `close_unknown_bbox_height`,
`target_depth_coast_max_s`, `depth_status`, `camera_fps`, `oak.det_latency_s`.
