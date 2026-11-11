# Wheel speed scale — 2026-11-10

Status: the correction is prepared in a worktree. It is NOT pushed. Kevin decides when it ships.

## 1. Summary

- The software converts VESC eRPM to m/s with three constants: 14 poles (7 pole pairs), a gear ratio of 34.2857:1 and a wheel radius of 0.18415 m. This conversion reads approximately 1.23 times the GPS ground speed.
- The error is a constant scale. It is not slip. It does not change with motor current, speed, drive, fix type or month.
- The data cannot show which of the three constants is wrong. The correction is one lumped factor, `VescConfig.wheel_ground_scale = 0.815` (uncertainty approximately ±0.03). The three geometry constants stay as labelled.
- Byte 254 (20000 eRPM) is approximately 1.31 m/s on the ground, not 1.61 m/s.
- The correction does NOT make follow-me faster. The robot drives the same eRPM for the same byte. Only the m/s numbers change, and two m/s clamps of the velocity loop become worth more bytes.

## 2. Evidence

Each line says whether the fact is MEASURED (from logs), READ (from code or a document) or INFERRED.

### 2.1 Data and method

- MEASURED. Source: the armed logs of 2026-11-04 to 2026-11-08 (reduced extracts) and three full raw logs (2026-11-07 12:17, 2026-11-08 12:36, 2026-11-08 14:25). The raw logs have motor current and duty.
- Method A (short windows): two GPS epochs 2 s apart, both eRPM above 1500, gyro heading change below 3 degrees, wheel speed change below 0.15 m/s. The ratio is wheel speed over GPS speed. The GPS speed is the receiver speed over ground (SOG) or the position change.
- Method B (long segments): consecutive GPS epochs for at least 6 s with both eRPM above 1500, the same fix quality, and the gyro heading within ±4 degrees of the start. The ratio is the integrated wheel distance over the GPS distance between the end points. Position noise has little effect on a long distance.
- Scripts: `ratio.py`, `segments.py`, `summarize.py` and `perdrive.py` in the session scratch directory. They are not in the repository.

### 2.2 Results

| Check | Result | Type |
|---|---|---|
| Measured eRPM / commanded eRPM (byte offset / 128 × 20000), straight windows | 1.000 (p25 0.997, p75 1.003), n 257 | MEASURED |
| Wheel / SOG, steady windows above 0.8 m/s, per drive | 1.24, 1.26, 1.18, 1.19, 1.20 (5 drives); all 1.21, n 67 | MEASURED |
| Wheel / GPS distance, long segments of 5 m or more | pooled 1.24 over 181 m, n 12; median 1.22 | MEASURED |
| Longest segments | 56.7 m in 71 s: 1.21 (DGPS, fix 2). 46.8 m in 70 s: 1.29 (RTK float, fix 5) | MEASURED |
| Motor-byte scale fit to the GPS track, 30 s windows (earlier study) | 0.0100–0.0105 m/s per byte against 0.012555 nominal: ratio 1.20–1.26 | MEASURED (earlier) |
| March 2026 calibration on gravel at max_erpm 15000 | 0.0075 against 0.00942 nominal: ratio 1.256 | READ (`config.py` comment) |
| Motor current per side on the windows | median 2.9 A, p90 4.2 A, maximum 5.8 A; duty median 0.42 | MEASURED |
| Ratio against motor current (linear fit, SOG ratio) | slope 0.010 ± 0.013 per A: no dependence | MEASURED |
| Ratio against speed | position ratio 1.11–1.22 in each bin; SOG ratio 1.41 below 0.6 m/s, 1.20–1.24 above 0.9 m/s | MEASURED |
| Ratio against acceleration | windows with acceleration above 0.05 m/s² read approximately 0.1 higher (n 10–23) | MEASURED |
| Left/right eRPM asymmetry on straight windows | median 0.6–1.1 percent | MEASURED |
| RTK float (fix 5) against DGPS (fix 2) | long segments 1.24 and 1.23; SOG windows 1.30 and 1.29 | MEASURED |

### 2.3 Scale or slip

- A slip grows with torque. These straights use 2–4 A of motor current per side. The ratio has no slope against current. INFERRED: a steady slip of 23 percent at this torque is not credible.
- The small increase during acceleration is a real slip term or GPS lag. It is a few percent and does not explain the 23 percent.
- Turning and track scrub do not explain it. The windows are straight (heading within 3–4 degrees) and the left/right eRPM differs by approximately 1 percent.
- GPS bias does not explain it. Two independent GPS measurements (Doppler SOG and the end-point distance over 50 m or more) agree, at two fix types and on five drives. The March calibration at RTK gave the same ratio.
- The receiver SOG under-reads below approximately 0.8 m/s. Use SOG only above 0.8 m/s.
- Not measured: reverse driving, turning segments, and a surface split (gravel against grass). Each drive gives approximately the same ratio, so a surface effect is small or the same on the surfaces driven.

### 2.4 Which constant

- READ. `pi_app/control/rpm_plausibility.py`, `erpm_to_wheel_mps`: speed = eRPM / (pole pairs × gear ratio) / 60 × 2π × radius. `controller.py` uses the signed mean of the two eRPM readings.
- READ. The radius 0.18415 m is the label diameter of 14.5 in divided by 2. Nobody measured a rolling radius. The gear ratio is recorded in one comment only (6:1 × 20/14 × 40/10). No document gives the sprocket counts or the motor pole count from a parts list.
- INFERRED. Candidates that give the measured ratio: a rolling radius of approximately 0.150 m (a smaller tyre or a soft tyre); a different sprocket (40/8 in place of 40/10 gives 1.25); a 7:1 gearbox (1.17). The data cannot separate them, because all three are one product.
- The VESC FW reports electrical RPM. The VESC holds the commanded eRPM (measured / commanded = 1.000). Thus the command path and the telemetry path have the same error.

## 3. Commanded speed

- READ. Byte 254 = offset 128 = 20000 eRPM (`VescCanDriver._byte_to_rpm`).
- MEASURED. 20000 eRPM is 1.607 m/s with the nominal constants and 1.310 m/s with the measured scale (range 1.25–1.35 m/s).
- READ. The 2026-09-26 follow-me tuning in `config.py` says that byte 110 was "1.38 m/s" and that the operator walked at "~1.55 m/s". 1.38 m/s is the nominal value for byte 110. On the ground it is approximately 1.13 m/s.
- INFERRED. The "1.55 m/s" walk was very probably the robot wheel speed plus the camera range rate, thus in the same over-reading units. On the ground it is approximately 1.26 m/s. The log of that drive is deleted, so this cannot be checked.
- A full pack does not change this. Below approximately 46 V the VESC reaches maximum duty before 20000 eRPM (READ, `config.py`), as before.

## 4. Consumers of the measured speed

| Consumer | Unit | Effect of the over-read today | Effect of the correction |
|---|---|---|---|
| Follow-me velocity loop (`SpeedLayer.compute`) | target = bytes × `speed_loop_mps_per_byte`, feedback = `actual_speed_mps` | None in bytes: target and feedback carry the same scale | Same bytes while the loop is inside its clamps (test `test_wheel_ground_scale.py`) |
| `speed_pid_max_correction_mps` 0.20 m/s | m/s | 15.9 bytes of authority | 19.5 bytes (+23 percent) |
| Integral clamp `speed_integral_limit` 1.0 × `speed_ki` 0.15 | m/s | 11.9 bytes | 14.7 bytes |
| Nearest-person speed limit | camera depth (m) | none | none |
| RPM plausibility gate, IMU motion witness | eRPM | none | none |
| Slip detection (`slip_cmd_diff_per_byte`, off) | eRPM | none | none |
| Dead-reckoning odometry (`trail_speed_scale_mps_per_byte` 0.0100) | motor bytes, GPS-fitted | none | none |
| Person stop hold (`person_stop_hold_plausible_speed_mps`) | the person's speed from depth | none | none |
| Logs: `vesc.speed_mps`, `follow_me.actual_speed_mps`, `speed_loop` fields, web telemetry, recorder | m/s | 23 percent high | ground speed |
| `tools/analyze_follow_me_log.py` | reads `speed_loop_mps_per_byte` from each session header | old logs stay correct | new logs correct |
| `tools/odometry_analysis/wheel_scale.py` | logged `speed_mps` | shows 1.2–1.3 | will show approximately 1.0 on new logs; old logs keep 1.23 |

## 5. The correction

- Constant: `VescConfig.wheel_ground_scale = 0.815` (new). Uncertainty: ±0.03 (the spread of the drive results, 1.18–1.29).
- `erpm_to_wheel_mps` and `wheel_mps_per_byte` take `ground_speed_scale` (default 1.0). `controller.py` passes the config value.
- `FollowMeConfig.speed_loop_mps_per_byte` changes from 0.012555 to 0.010232. A test pins it to the derivation.
- Why a separate factor and not a new radius: the data cannot show that the radius is wrong. A Grok review (section 7) recommended this.
- What changes for the driver: nothing in the motion while the follow-me velocity loop is inside its clamps. The speed numbers on the phone, in the logs and in the analysis tools become the ground speed. The velocity loop can add or remove up to 19.5 bytes in place of 15.9 bytes when the wheels cannot reach the command (soft ground, low pack).
- The push restarts `wall-e`, because `config.py` and `controller.py` change.

## 6. Decisions for Kevin

1. Ship the correction or not.
2. Keep `speed_pid_max_correction_mps` at 0.20 m/s (19.5 bytes after the correction), or set it to 0.163 m/s to keep 15.9 bytes.
3. Keep `speed_integral_limit` at 1.0, or set it to 0.815 to keep 11.9 bytes.
4. Top speed: follow-me tops out at approximately 1.31 m/s on the ground. A fast walk can be faster. A higher `max_erpm` needs a duty check (READ: 17,200 eRPM used duty 0.71 on a full pack).
5. Approach slope: 73 bytes per metre stays 73 bytes per metre. In ground units it is 0.75 m/s per metre of range error, not 0.92 m/s. The slope was tuned in bytes on the robot, so it is still the validated slope.
6. Stale comments that give "1.61 m/s": `pi_app/control/safety.py` (`arm_midband_disarm_s`: the distance on a dead arm switch is 0.65 m, not 0.8 m), `docs/gearing_memo.md`, `CLAUDE.md` (follow-me rounds 2 and 6). Correct them with the next change to each file.
7. Do the tape test (section 8). It sets the final value and shows which constant is wrong.

## 7. Second opinion

Grok session `01a1c83e-133d-7fb2-8b63-eb43035aa6f7` (effort low).

- Grok agrees with the scale conclusion. It says that slip at 2–4 A is a few percent, that a 4 degree arc changes the path by approximately 0.02 percent, and that a 23 percent GPS scale error is not credible when Doppler SOG and a 50 m position change agree.
- Grok cautions that the ratio is not one constant: the results spread by several percent. Dead reckoning with the lumped value keeps approximately 3 m of error in 100 m.
- Grok agrees that the loop is ratiometric. It cautions that a setpoint in ground metres per second does not scale. Action: the agent searched the code. The only m/s values that the loop compares with the measured speed are the two clamps (section 4). No other m/s setpoint feeds the loop.
- Grok recommended a separate ground-speed factor and not a new `wheel_radius_m`, because the radius, the gearing and the pole count cannot be separated. Action: the correction uses `wheel_ground_scale`. The first draft changed the radius.
- Grok noted that "measured = commanded eRPM" does not prove the pole count. Correct: section 2.4 says the same.

## 8. Tape test

No tape-referenced wheel data exists. The 7 m push of 2026-11-06 (`docs/depthai_upgrade.md`, section 5) was done with `wall-e` stopped, so no eRPM was logged.

Procedure:

1. Put a tape measure along a straight, flat line of 20 m on gravel. Put a marker at 0 m and at 20 m.
2. Put a chalk mark on the side wall of one tyre, at the ground.
3. Arm in MANUAL. Start 3 m before the 0 m marker.
4. Drive straight with the stick held steady at approximately three quarters. Do not steer.
5. Say "now" (or press a phone button) at the 0 m and at the 20 m markers. A helper with a stopwatch also records the time.
6. Stop after the 20 m marker. Disarm.
7. Do it two times more: once at half stick, once on grass.
8. Push the robot by hand (disarmed) along 5 m of the tape and count the tyre turns with the chalk mark. Measure the distance for exactly 4 turns.

What to read:

- From the run log: integrate `vesc.left_rpm` and `vesc.right_rpm` between the two times. Ground scale = 20 m / (nominal wheel distance). The expected value is 0.815 ± 0.03.
- The motor current (`l_motor_a`, `r_motor_a`) for each run. If half stick and three quarters give the same scale, the error is scale, not slip.
- Tyre turns: 4 turns × 2π × 0.18415 m = 4.63 m if the label radius is correct. Approximately 3.77 m if the radius causes the whole error. If the distance is near 4.63 m, the error is in the gearing or the pole count: count the sprockets.
- At RTK fixed, also read the GPS distance between the two times. It must agree with the tape within 1 percent.

## Appendix: Technical Names

eRPM, RPM, VESC, FW, GPS, RTK, RTK fixed, RTK float, DGPS, SOG, Doppler, fix quality, Grok, Kevin, MANUAL, FOLLOW_ME, follow-me, dead reckoning, odometry, eRPM loop, velocity loop, PID, duty, pole pair, sprocket, gearbox, tyre, side wall, stick, phone button, run log, worktree, session, scratch directory, `wall-e`, `config.py`, `controller.py`, `VescConfig`, `FollowMeConfig`, `wheel_ground_scale`, `wheel_radius_m`, `speed_loop_mps_per_byte`, `speed_pid_max_correction_mps`, `speed_integral_limit`, `speed_ki`, `trail_speed_scale_mps_per_byte`, `slip_cmd_diff_per_byte`, `person_stop_hold_plausible_speed_mps`, `max_erpm`, `arm_midband_disarm_s`, `erpm_to_wheel_mps`, `wheel_mps_per_byte`, `SpeedLayer`, `VescCanDriver._byte_to_rpm`, `l_motor_a`, `r_motor_a`, `vesc.left_rpm`, `vesc.right_rpm`, `vesc.speed_mps`, `follow_me.actual_speed_mps`, `ratio.py`, `segments.py`, `summarize.py`, `perdrive.py`, `test_wheel_ground_scale.py`, `analyze_follow_me_log.py`, `wheel_scale.py`, `CLAUDE.md`, `docs/gearing_memo.md`, `docs/depthai_upgrade.md`, `pi_app/control/safety.py`, `pi_app/control/rpm_plausibility.py`.

Technical Verbs: arm, disarm, integrate, log, push, ship.
