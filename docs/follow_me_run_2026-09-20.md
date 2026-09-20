# Follow-me retune (2026-09-20)

This document records the plant-gain correction after the 2026-09-19
18:50 run and the 2026-09-20 14:58 weave. Reproduce the plant number
with `python3 tools/analyze_follow_me_log.py <log>`.

## 1. Evidence

1. The 2026-09-19 retune uses 0.22 deg/s per L-R byte. That number is
   wrong. The regression uses `imu.yaw_rate_dps`. 64 percent of those
   samples are exactly 0.0 (1,186 of 1,833 ticks on 2026-09-19; 172 of
   270 on 2026-09-20). The derivative of `imu.heading_deg` against L-R
   gives 0.65-0.71 deg/s per byte. The best correlation is r = 0.89-0.93
   at a lag of about 0.45 s, on both days.
2. The zeros come from `OakImuReader._consume_producer_yaw`. The
   duplicate branch reports the live rate only when `age_s` is at most
   max(0.05, 3 * cadence). Under follow-me load the IMU snapshot age on
   duplicate reads is p50 0.090 s, p90 0.182 s, max 0.286 s. The vision
   thread drains the IMU at about 6-10 Hz, so the bound almost never
   passes. The logged rate is 0.0 while `gy_body_dps` is 18-20 deg/s on
   the same tick. The heading stays correct because the producer
   integrates every packet.
3. The 2026-09-19 retune raises the loop gain by 3.2x (`pid_lateral_kp`
   0.4 -> 0.8, `max_steer_offset_byte` 25 -> 40,
   `direct_mode_max_steer_byte` 18 -> 40). The robot weaves with growing
   amplitude (person x: +0.8, -0.9, +1.0, -1.5, +1.7 m at about 4.5 s
   period) and overshoots a 28 deg bearing into a 120 deg swing at
   45-58 deg/s.
4. An offline simulation (integrator plant 0.70 deg/s/byte, 0.15-0.30 s
   dead time, 0.30 s lag, 6-8 Hz detections with 0.15-0.25 s latency,
   real deadband/edge-boost/slew) shows the 2026-09-19 gains are
   unstable on the pessimistic plant (bearing peaks 15, -20, 21, -29,
   55 deg). kp 0.5 / max 32 / slew 0.25 is well damped on both plants
   (peaks 15, -8) and still has more turn authority than the pre-09-19
   tune (max bearing in a 90 deg operator turn 43 deg vs 55 deg).

## 2. Changes

1. IMU duplicate-rate freshness bound is now max(0.20, 3 * cadence).
2. Follow-me steer retune: `pid_lateral_kp` 0.8 -> 0.5,
   `max_steer_offset_byte` 40 -> 32, `direct_mode_max_steer_byte` 40 ->
   32, `steer_slew_per_tick` 0.1 -> 0.25. `pid_lateral_kd`,
   `steer_edge_boost`, `steer_edge_knee`, and `direct_turn_speed_*` stay
   unchanged.
3. The log analyzer derives yaw rate from the heading derivative and
   searches lag 0-6 ticks. It still prints the logged-rate fit and
   labels it unreliable when samples are 0.0. Direct-cap default is the
   session-header value, else 32.
4. This document and the correction at the top of
   `docs/follow_me_run_2026-09-19.md`.

The tracklet centre-distance fallback and single-candidate rebind ship
separately on the same day.

## 3. Field test

CAUTION: Do this test on a clear path. Stay at a range where the
operator can stop the robot.

1. Straight walk 30 m at normal pace. The robot must not weave. The
   analyzer must show bearing peaks that shrink, with no growth.
2. Make two 90 deg turns at 3 m.
3. Walk out to 6 m and walk back.
4. If the robot still weaves, POST `/api/follow_me/params` with
   `{"pid_lateral_kp": 0.4}`. If turns are too slow, POST
   `{"pid_lateral_kp": 0.6}`.

## Appendix: Technical Names

FOLLOW_ME, IMU, L-R differential, `pid_lateral_kp`,
`max_steer_offset_byte`, `direct_mode_max_steer_byte`,
`steer_slew_per_tick`, `tools/analyze_follow_me_log.py`.
