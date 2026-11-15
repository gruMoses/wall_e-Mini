Moved out of CLAUDE.md on 2026-11-14 (text kept as it was; it was the "OAK-D Lite Camera" section). Speeds in m/s here are in the old units before `wheel_ground_scale` 0.815 (refer to `docs/known-issues-history.md`, "Wheel ground scale").

### OAK-D Lite Camera
- Obstacle avoidance: depth corridor, valid-pixel % threshold, tiered speed reduction/stop.
- Follow Me: **YOLOv8n** blob via `NeuralNetwork` node + host-side NMS (depthai v3). `SpatialDetectionNetwork` / `YoloDetectionNetwork` / `DetectionParser` yield zero detections with the CURRENT blob because it comes from a plain ultralytics ONNX export through blobconverter (decode head kept, no NN Archive `heads` metadata). The supported path is the Luxonis conversion tool → NN Archive, which also unblocks on-device `ObjectTracker` and spatial coordinates. Refer to `docs/oak_d_lite_capability_audit.md` before you re-convert the model.
- Depth EMA filter on person position for smoothing.
- **Second follow-me round (2026-09-19, from the 18:50 run log; analysis in
  `docs/follow_me_run_2026-09-19.md`, reproduce with
  `tools/analyze_follow_me_log.py <log>`)**: direct-pursuit steering was
  retuned and direct pursuit now slows into turns
  (`direct_turn_speed_*`). `detect_min_bbox_width` 0.05 (0.09 rejected the
  operator beyond 4.3 m). The velocity PID targets the forward byte that
  actually reached the motors on the previous tick
  (`update_telemetry(emitted_forward_byte=...)`) so it cannot wind up against
  the persistence decay, accel ramp, obstacle throttle or slew limiter; a
  0.30 s grace hold precedes the persistence decay. The depth corridor needs
  400 px of support and 2-poll persistence (phantom obstacles in low sun).
  `VescConfig.max_erpm` 20000 (was 15000; 1.6 m/s top speed in every mode).
  Steering gains from this round were wrong (see round 3). Width filter,
  corridor, `max_erpm` and the emitted-byte speed loop ran on the
  2026-09-20 17:11 and 18:09 validations.
- **Third follow-me round (2026-09-20 14:58, `docs/follow_me_run_2026-09-20.md`)**:
  the 09-19 "0.22 deg/s per L-R byte" yaw gain was WRONG (64 % of logged
  `imu.yaw_rate_dps` samples were 0.0: the duplicate-read freshness bound in
  `oak_imu.py` was 50 ms while the sample age under follow-me load is
  90-290 ms; now 0.20 s). From the heading derivative the plant is
  **0.65-0.71 deg/s per byte**. The 09-19 gains made the robot weave, so:
  `pid_lateral_kp` 0.5, `max_steer_offset_byte` 32,
  `direct_mode_max_steer_byte` 32, `steer_slew_per_tick` 0.25. **Never fit a
  plant from `yaw_rate_dps`; use the heading derivative (the analyzer does).**
  The same run hard-stopped five times because the tracklet id on a lone
  operator churned (narrow box + 0.12-0.35 s detection gaps + ego yaw => IoU
  0) and the sticky lock read each new confirmed id as a different person.
  Fix: centre-distance fallback in `TrackletTracker` (`tracklet_center_gate_*`)
  and a single-candidate rebind in `TargetTracker._find_committed` (exactly
  one candidate, <= 0.5 s, base depth gate). **Validated** first 50 s of
  the 15:49 run (100 % fresh, no hard stops, no weave).
- **Fourth follow-me round (2026-09-20 15:49 near-run-over; validated 17:11)**:
  identity gates decide whom to follow; they never hide a closer range from
  the speed command. Config/telemetry:
  `nearest_person_speed_limit_enabled`, `nearest_person_margin_m`,
  `close_unknown_bbox_height`, `target_depth_coast_max_s`. Sampler
  `depth_status`: `ok` / `no_support` / `ambiguous` / `far_veto` /
  `no_frame` / `height_veto` (off). `person_depth_*` knobs in
  `FollowMeConfig`. Log: `arm_20260920_154844.log` / `arm_20260920_171057.log`.
- **Fifth follow-me round (vision latency, validated 18:09
  `arm_20260920_180915.log`)**: `OakDetectionConfig.camera_fps = 15.0` is
  **load-bearing — NEVER remove it without re-measuring `oak.det_latency_s`**.
  Also: `nn_input_queue_size = 1` (alone did not fix latency),
  `vision_deadline_sleep`, `poll_detections_first`, MediaPipe Hands is not
  run in RC-started FOLLOW_ME. Idle A/B: read `oak.det_latency_s` from
  `logs/latest.log` (no field walk). Under load p50 0.20 s at 15 fps
  (was 0.80 s at 8 fps). Motor→yaw lag 0.47 → 0.12 s was heading latency
  from a blocked vision thread, not the drivetrain. Steer gains left at
  kp 0.5 / cap 32.
- **Sixth follow-me round (2026-09-26, `arm_20260926_125554.log`,
  `arm_20260926_130713.log`)**: the robot sat at the old 110-byte cap
  (1.38 m/s) behind a 1.55 m/s walk and lost the operator on range alone
  at 6.1 m. Now `max_follow_speed_byte` 128 (byte 254 = `max_erpm` =
  1.61 m/s, the drivetrain limit; ~1.5 m/s on a half pack),
  `max_speed_error_m` 1.75 (keeps the 73 bytes/m approach slope),
  `max_distance_m` 8.0 (the 0.05 width floor binds first near 7-7.5 m),
  `follow_distance_m` 1.2 (Kevin's 4 ft stop; zero forward inside 1.4 m).
  False near matches on the iron gate put 0.7-0.8 m phantoms in the depth
  corridor (slow, jerky approach). Fix: a near pixel needs 16 near
  neighbours within one 100 mm depth bin in its 13x13 window
  (`corridor_speckle_*`); `person_mask_depth_margin_m` 0.5 masks only the
  person's own pixels (the old whole-box mask hid anything between robot
  and person). Tuned on 45 raw frames + dropout (150); a blob-size filter
  lost woven wire fence and was rejected. The recorder's colourized depth cannot tell a
  false match from a hole; capture raw frames with the local-only
  `POST /api/debug/depth_snapshot` (`logs/depth_snapshots/*.npz`). See
  `docs/NEXT_SESSION.md` section 4, item 5.
- **Arms-up bench twitch (2026-09-22, `docs/NEXT_SESSION.md` section 2)**:
  MediaPipe Pose on the person crop (`pi_app/hardware/pose_worker.py`,
  `pi_app/control/arms_up.py`) gives one 22-byte, 0.25 s reverse pulse.
  Only the volatile latch `POST /api/arms_up/twitch_test` enables it
  (local-only, armed + MANUAL, 3 pulses / 5 min). Pose runs only while
  that latch is on, so measure latency in that state. Do NOT reuse the
  latch for the continuous back-up feature: it needs its own latch and
  its own Grok safety review.
- The onboard IMU is a **BMI270**: raw/calibrated accel + gyro only. `ROTATION_VECTOR` / `GAME_ROTATION_VECTOR` / magnetometer are BNO08x-only and return nothing on this device. Do not try them. Refer to `docs/oak_d_lite_capability_audit.md`.

