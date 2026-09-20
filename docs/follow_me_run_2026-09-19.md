# Follow-me run analysis (2026-09-19 18:50)

Log: `/home/pi/wall_e-Mini/logs/arm_20260919_184719.log` on the Pi (a copy
was analyzed on the Mac). FOLLOW_ME ran from 18:50:10 to 18:53:45 (215 s,
1,833 ticks, direct pursuit for more than 99 percent of the time). Kevin's
report: good but not great; jumpy toward the end; did not turn fast enough;
the speed felt limited.

Reproduce every number in this document with
`python3 tools/analyze_follow_me_log.py <log>` (section 4).

## 1. Findings

Four causes, in order of effect:

1. **The detection filter rejected the operator beyond 4.3 m.**
   `detect_min_bbox_width` was 0.09. A 0.45 m wide person is 0.086 of the
   70 deg frame at 4.4 m. YOLO still saw the person at 0.86-0.90 confidence.
   Each rejected frame set `fresh_detection` false, and one missed frame at
   full speed scaled the speed by 0.78 (the persistence decay window was
   0.3 s at full speed). The robot stopped, the operator walked on, the
   robot re-acquired and surged to full speed. This is the stop-surge cycle.
2. **The direct-pursuit steering authority was about 5 times too low.**
   A regression of the IMU yaw rate on the L-R byte differential over the
   run (1,468 ticks with wheel speed >= 0.3 m/s) gives 0.22 deg/s per byte
   (r = 0.41). The cap `direct_mode_max_steer_byte` = 18 allowed at most
   about 8 deg/s while a person crossing at 3 m sweeps about 25 deg/s. At
   t = 31-34 s the operator sat at a 20-26 deg bearing for 8 s with 5-10
   bytes of steer, then walked out of the right edge of the frame
   (`bbox xmin` 0.755 -> 0.882, rejected by the edge rule).
3. **The velocity PID wound up against external limits.** `SpeedLayer`
   targets `open_loop_byte` from the depth error, but the persistence decay,
   the accel ramp, the obstacle throttle and the slew limiter all cut the
   command after it. The PID then sat at its +21 byte clamp
   (`speed_loop.corr_byte` 21.2 while `speed_offset` was 2.5) and dumped it
   as a surge when the limit lifted.
4. **Phantom depth-corridor obstacles in low sun (last 30 s).**
   `obstacle.distance_m` flickered 2.3 -> 1.4 -> 2.3 -> 1.6 -> 0.4
   (`depth_p5_mm` 383) -> 0.8 while the operator was 2.8-3.8 m ahead and
   the full-frame valid depth fraction was 2-7 percent. `throttle_scale`
   went 0.9 -> 0.0 -> 0.4 -> 0.8 and motor L went 236 -> 127 -> 137 -> 217.
   The corridor distance was the 5th percentile of the valid corridor
   pixels with no minimum pixel support, so a handful of noisy near pixels
   set it.

**Speed limit.** The top speed was 1.2 m/s, set by `VescConfig.max_erpm` =
15000. `docs/gearing_memo.md` records that value as unverified. On this run
the wheels reached 13,000-14,600 eRPM at a VESC duty of only 0.58-0.65,
i.e. about 22,000 eRPM per unit duty. `max_follow_speed_byte` (110 of 128)
was at its cap for long stretches (t = 20-30 s, 54-58 s, 160-204 s), and a
walking operator still opened the distance from 2.2 m to 4.7 m in 10 s.

## 2. Changes (2026-09-19, second follow-me round)

Config (`config.py`):

| Field | Old | New | Why |
|---|---|---|---|
| `VescConfig.max_erpm` | 15000 | 20000 | 1.2 -> 1.6 m/s top speed in every mode; duty headroom measured above |
| `FollowMeConfig.speed_loop_mps_per_byte` | 0.009416 | 0.012555 | kinematic scale follows `max_erpm` |
| `FollowMeConfig.trail_speed_scale_mps_per_byte` | 0.0075 | 0.0100 | scaled 4/3; re-calibrate on gravel |
| `FollowMeConfig.slip_cmd_diff_per_byte` | 40.0 | 53.3 | scaled 4/3 (feature disabled) |
| `FollowMeConfig.detect_min_bbox_width` | 0.09 | 0.05 | finding 1; the edge rule keeps the sliver protection |
| `FollowMeConfig.pid_lateral_kp` | 0.4 | 0.8 | finding 2 |
| `FollowMeConfig.max_steer_offset_byte` | 25 | 40 | finding 2 (PID output scale) |
| `FollowMeConfig.direct_mode_max_steer_byte` | 18 | 40 | finding 2 |

Code:

- `SpeedLayer` (finding 3): the closed-loop target is now the forward byte
  that reached the motors on the previous tick minus the correction the
  layer itself applied, never more than the open-loop request. The
  controller passes that byte through `update_telemetry(emitted_forward_byte=...)`.
  New telemetry: `follow_me.speed_loop.target_byte`.
- Persistence decay (finding 1): new `steer_hold_grace_s` (0.30 s hold
  before any decay) and `steer_hold_decay_speed_floor` (0.5, was a literal
  0.3), so one missed frame no longer cuts the speed.
- Direct-pursuit turn slowdown (finding 2): forward speed scales down with
  the bearing of the fresh detection (`direct_turn_speed_knee_norm` 0.30,
  `direct_turn_speed_min_scale` 0.35), so the differential is not clipped
  by the mixer at full speed. New telemetry: `follow_me.turn_speed_scale`.
- Depth corridor (finding 4): the near estimate is the k-th smallest valid
  depth with `corridor_min_support_px` (400) of support, and a 2-poll
  persistence (`corridor_persistence_polls`) so a single-poll phantom
  cannot lower the distance. New telemetry: `obstacle.corridor_valid_pct`,
  `obstacle.corridor_support_px`.
- `tools/analyze_follow_me_log.py`: offline analyzer (timeline, jumpiness
  with cut attribution, detection-filter histograms, steering authority,
  yaw regression, speed-cap use, obstacle flicker).

## 3. Field test procedure

CAUTION: `max_erpm` 20000 makes the RC manual mode 33 percent faster too.
Drive the first manual segment gently.

1. Arm. Start Follow Me on the same course as the 18:50 run.
2. Walk at a normal pace for 30 s, then a brisk pace for 30 s. Expected:
   no stop between 4 and 5 m; the distance settles near 1.5-2.5 m.
3. Make two 90 deg turns at about 3 m range. Expected: the robot slows
   into the turn and re-centres within about 2 s; the operator stays in
   frame.
4. Walk away to 6 m and stop. Expected: the robot closes to 1.5 m and
   stops without a surge.
5. Disarm. Copy the log and run the analyzer. Compare the jumpiness
   section with this run (cuts caused by "dropout" should be near zero).

If the robot wobbles at close range, lower `pid_lateral_kp` to 0.6 first
(live through `POST /api/follow_me/params`). If the VESC saturates (eRPM
stays below the command at duty >= 0.95), set `max_erpm` to 18000.

## Appendix: Technical Names

FOLLOW_ME, YOLO, OAK-D Lite, VESC, eRPM, duty, RC, IMU, L-R differential,
`SpeedLayer`, `SafetyLayer`, `update_telemetry`, `fresh_detection`,
`speed_offset`, `open_loop_byte`, `corr_byte`, `throttle_scale`,
`depth_p5_mm`, `bbox`, `max_erpm`, `max_follow_speed_byte`,
`detect_min_bbox_width`, `pid_lateral_kp`, `max_steer_offset_byte`,
`direct_mode_max_steer_byte`, `steer_hold_grace_s`,
`direct_turn_speed_knee_norm`, `corridor_min_support_px`,
`tools/analyze_follow_me_log.py`, `docs/gearing_memo.md`.
