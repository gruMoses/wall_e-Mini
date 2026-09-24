# Design: "pump to back up"

Status: DRAFT, 2026-09-24. Not implemented. Kevin decides the open items in section 10.
This document uses Simplified Technical English (ASD-STE100) as far as is reasonably possible. The appendix lists the technical names.

## 1. Purpose

Kevin walks toward the robot and gives a hand signal. The robot then moves backward, keeps Kevin near the centre of the camera image, and keeps its distance. When Kevin stops the signal, the robot stops.

Use cases: the robot is in a gate or a doorway and Kevin must get past it; Kevin wants the robot to move away from him without the RC.

## 2. Agreed constraints (Kevin, 2026-09-20)

- The reverse speed cap is 0.4 m/s.
- The robot stops when the signal stops.
- Each reverse movement has a maximum distance.
- The RC always overrides.
- The robot has no rear sensors. Kevin accepts this.
- Skid-steer yaw does not depend on the travel direction. Thus the keep-centred steering law stays the same in reverse.

## 3. What the tests showed

### 3.1 Raised arms do not work (bench test, 2026-09-24)

- The camera is low. At 2.5 m to 3 m, Kevin's shoulders are at 0.09 to 0.13 of the preview height, and the top of the detection box is clipped.
- Raised hands leave the frame when Kevin is closer than approximately 3.5 m to 4.5 m.
- The back-up feature must work at 1.5 m to 3 m. Thus the "both arms up" trigger cannot be used.
- Finger and thumb shapes cannot be used. At 1.5 m, a hand is approximately 25 pixels wide in the 640x480 preview.
- MediaPipe Pose cannot be used at this range. Its detector needs the face, and the face leaves the frame inside approximately 2.5 m.

### 3.2 A low two-hand "pump" is visible (test, 2026-09-24, 18:47)

Kevin stood in front of the robot and pumped both hands outward below the waist. The log recorded the person box of the YOLO detector at approximately 8 Hz.

| Condition | Box width | Notes |
|---|---|---|
| Standing, 21 s, at 2.9 m | median 0.127 of the frame (approximately 0.52 m), p95 1.28 x median | Box centre standard deviation 0.014 |
| Pump, at 1.5 m to 2.2 m | up to 3.7 x the rest width | Both box edges moved outward |
| Pump, at 2.1 m to 3.1 m | up to 3.2 x the rest width; 3 peaks above 1.8 x in 3.3 s | Left edge moved 0.160, right edge 0.106; centre standard deviation 0.008 |

Conclusions:

- Hands at hip height stay in the image from approximately 1.5 m to 6 m, because the camera is at approximately hip height.
- The pump makes the box 2 to 4 times wider, with approximately 1 peak per second.
- Both edges move outward and the centre stays in position. A one-arm wave moves one edge only.
- The detector needs no face, no fingers and no new model. It uses the box of the follow-me target.

CAUTION: This is one session with one person. It is not a test of false starts. Section 9, step 1 is that test.

## 4. The signal

- **Start:** both hands low, pumped outward 2 to 3 times, at 2 m or more from the robot.
- **Continue:** keep pumping, or hold the hands out in a low "V".
- **Stop:** hands down.

## 5. Detector (new pure module, `pi_app/control/pump_gesture.py`)

Input on each new detection: the box of the locked follow-me target (the detection with the target's `track_id`) and the stereo range `z_m` from the SAME detection record.

1. Accept a detection only if `depth_status` is "ok", `z_m` is 1.0 m to 5.0 m, and the box does not touch a side edge of the frame.
2. Convert to metres with the same `z_m`: box width `w_m`, left edge `l_m`, right edge `r_m`. Get the frame geometry from `OakDepthReader.get_intrinsics()`, not from a field-of-view constant. A change of range then does not look like a pump.
3. Reject a body width `w_m` outside 0.3 m to 2.2 m.
4. Rest width and rest edges: the median over the last 3 s. Freeze them at the first rise through 1.4 x rest. Do not update them while the signal is active.
5. "Out" frame: `w_m >= 1.5 x rest`, both edges out by at least 25 % of the rest width, and a centre shift below 0.15 m.
6. Peak: at least 3 consecutive "out" frames, then at least 1 frame below 1.4 x rest. A one-frame box change is not a peak.
7. Start: 2 peaks within 3.0 s.
8. Continue: an "out" frame in the last 0.7 s. (At approximately 1 pump per second, the gap between pumps is approximately 0.5 s.)
9. Stop: no "out" frame for 0.7 s, or no accepted detection for 0.3 s.

Every threshold is a config value. The test data in section 3.2 sets the first values.

## 6. Behaviour

### 6.1 Where it runs

- In FOLLOW_ME only, on the locked target. The robot must already follow Kevin.
- Not in MANUAL and not in WAYPOINT_NAV (version 1).

### 6.2 Motion while the signal is active

- This state replaces the follow-me speed and distance loops for the whole movement. The follow-me command is not added.
- Reverse only. The robot never moves forward in this state.
- Keep distance: `d_keep = max(2.0 m, z at the start)`. At 2.0 m or more, the pump stays inside the frame.
- Speed: `v_back = clamp(0.8 x (d_keep - z), 0, 0.4)` m/s. When `z` is more than `d_keep`, the speed is 0, never forward.
- Steering: the follow-me keep-centred law, with the steer cap halved. When the robot turns in reverse, the rear swings through space that no sensor sees.
- The slew limiter and all existing overrides apply after this command.

NOTE: At 0.4 m/s the robot cannot move away from a normal walk (approximately 1.2 m/s). Kevin must walk slowly. If Kevin comes closer, the robot continues at the cap and does not go faster.

### 6.3 Limits for each reverse movement

- Maximum time 8 s. This is the hard limit.
- Maximum distance 3 m, from the wheel eRPM. On a slope or on grass this value is soft.
- After a stop, a new start needs 2 s, and the rest width seen again for 1 s (hands down), then a new pump.

## 7. Safety

The robot moves backward with no rear sensors. Each item below stops the reverse movement at once, and a stopped movement does not resume:

1. Hands down (section 5, item 9).
2. Target lost, not fresh, or its `track_id` changes to a different person.
3. Range unknown, or outside 1.0 m to 5.0 m, for 0.3 s.
4. A stick outside the neutral band, a web-teleop command, disarm, e-stop, mode change, charger, pack-low, calibration, or RC stale.
5. Stall: the measured wheel speed stays below 30 % of the commanded wheel speed for 0.5 s (the robot has hit something).
6. The limits in section 6.3.

More rules:

- The signal counts only for the locked target. A pump from another person does nothing.
- During development a volatile, local-only latch enables the feature, like the arms-up bench latch. The arms-up latch is not reused (refer to `docs/NEXT_SESSION.md`).
- A Grok safety review of the code diff is necessary before each push that can move the robot.
- WARNING: "Any stick" does not detect a receiver that holds the sticks at centre after a lost link. Before the first powered test, make sure that the receiver failsafe disarms the robot.

## 8. Second opinion (Grok, 2026-09-24)

Grok agreed with the box-width signal and rejected MediaPipe Pose for this range. It found these defects in the first draft. This version corrects all of them:

1. The start confirmation (a fall below 1.4 x) could last longer than the 0.4 s hold, so the gesture stopped itself. Now: continue uses a 0.7 s window after the last "out" frame.
2. A one-frame box flicker counted as a peak. Now: 3 consecutive frames, and the rest width freezes at the first rise.
3. The hold was easier than the start, and a false wide box could hold for 3 m. Now: continue needs symmetric "out" frames, a 0.15 m centre bound, and a restart needs the rest width again.
4. A closing range makes both pixel edges move out. Now: all edge tests are in metres, with the range from the same detection.
5. `min(0.4, 0.8 x (d_keep - z))` became negative (forward, toward Kevin) when `z` was more than `d_keep`. Now: clamped to 0 to 0.4.
6. The pump does not fit in the frame near 1.5 m. Now: `d_keep` is at least 2.0 m.
7. Keep-centred steering in reverse swings the blind rear. Now: steer cap halved. Stall uses commanded wheel speed, not a fixed eRPM.
8. The false-start test must include hard cases (section 9, step 1). A spotter stands behind the robot for the first powered test.

## 9. Validation plan

1. **Log only.** Deploy the detector with no motion. Kevin works for 10 minutes in front of the robot. Include these cases: hands on hips, a carried pipe or board, bending, turning around, walking toward the robot at normal speed, a second person, a dog or a chicken in the box. Then pump at approximately 2 m, 3 m and 4 m. Pass: no start during the work; at least 9 starts in 10 pumps. Record every single peak, box merge and near miss.
2. **Stand.** The robot is on the test stand, with the latch on and a cap of 0.2 m/s. Measure: hands down to wheels stopped (target 1.0 s or less), stick to stop (next tick), time limit.
3. **Ground, short.** Garage, cap 0.2 m/s, maximum 1 m, a spotter behind the robot.
4. **Ground, full.** Cap 0.4 m/s, maximum 3 m.

## 10. Open decisions for Kevin

1. Stop time: after the hands go down, the robot stops in approximately 0.7 s plus the slew ramp. At 0.4 m/s that is approximately 0.3 m more travel.
2. Speed cap 0.4 m/s: walk slowly toward the robot.
3. Limits for each movement: 8 s (hard) and 3 m.
4. FOLLOW_ME only in version 1: the robot must already follow you.
5. Keep distance of at least 2.0 m during the movement (the follow distance is 1.5 m).

## Appendix: technical names

**Technical Names:** Kevin, WALL-E, YOLO, OAK, OAK-D Lite, NN, RC, MediaPipe Pose, Grok, FOLLOW_ME, MANUAL, WAYPOINT_NAV, eRPM, e-stop, pack-low, failsafe, stall, spotter, `track_id`, `z_m`, `w_m`, `l_m`, `r_m`, `d_keep`, `v_back`, `depth_status`, `follow_distance_m`, `OakDepthReader.get_intrinsics()`, `pi_app/control/pump_gesture.py`, `docs/NEXT_SESSION.md`, box (detection box), frame, preview, pixel, stereo, range, latch, slew limiter, skid-steer, yaw, steer cap, p95, median, standard deviation, config, test stand, garage, web teleop, receiver.

**Technical Verbs:** clamp, deploy, freeze, log, pump, push, stall.
