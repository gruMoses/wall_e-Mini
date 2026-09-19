# RESULT — fable-follow-fixes (2026-09-19)

This document gives the before and after state of two follow-me changes.
The changes are on branch `fable-follow-fixes`. They are not pushed and not deployed.

## 1. Person track lock

### Before

- The brief said that the live YOLOv8 path does not set `track_id`. That is not correct. Commit 6ef3378 added the host-side `TrackletTracker` to the live path.
- The tracklet layer matched on IoU only. A closer person with a larger box that overlaps the operator can get the operator's `track_id`.
- `TargetTracker` did an immediate hand-off. When the committed id was missing for one frame, the closest candidate above the acquire floor got the lock.
- Result: a closer person who walks in front of the operator gets the lock on one frame.

### After

- Tracklet association also needs depth continuity: `tracklet_depth_gate_m` = 0.75 m, plus 0.10 m for each missed frame. The live YOLO path sends each detection depth to the tracker.
- The committed-target match (by id and by position) needs depth continuity: `target_depth_continuity_m` = 0.6 m, plus 1.5 m/s multiplied by the time since the target was last seen.
- The hand-off is sustained. A challenger must be the best candidate for `target_switch_min_s` = 1.0 s and for `target_acquire_min_frames`. During that time, follow-me holds the operator (grace hold). If the operator comes back, the challenger count goes back to zero.
- A new tentative tracklet at the operator position is accepted as the operator (id change after an occlusion). A different confirmed id at the same position is not accepted.
- To get the old behaviour, set `target_switch_min_s = 0` and set each depth gate to 0.

## 2. Velocity PID

### Before

- `speed_kp`, `speed_ki` and `speed_kd` were 0 from 2026-06-11. The cause was dead RPM readback, which caused a lunge and stall cycle.
- The loop used two different scales. The target came from the GPS ground-speed scale (0.0075 m/s per byte). The feedback was wheel speed from eRPM. This gives a permanent error of approximately 20 percent.

### After

- The new scale is `speed_loop_mps_per_byte` = 0.009416 m/s per byte. It is the wheel speed from the VESC values: 15000 eRPM divided by 128 bytes, 7 pole pairs, gear ratio 34.29:1, wheel radius 0.18415 m. The controller calculates `actual_speed_mps` with the same function (`rpm_plausibility.erpm_to_wheel_mps`). A test makes sure that the two values agree.
- Gains: kp = 0.6, ki = 0.15, kd = 0. The integral limit is 1.0. The correction limit is ±0.20 m/s (approximately ±21 bytes).
- New `RpmPlausibilityGate`. The gate trips when a motor has a command of 12 bytes or more from neutral and reports less than 150 eRPM for 0.5 s. When the gate trips, the controller sets the RPM values and `actual_speed_mps` to `None`. Follow-me then uses open-loop control. The gate stays tripped for 2 s minimum. It resets only when a real RPM value comes back. The dashboard telemetry shows `vesc_rpm_plausible` and `vesc_rpm_gate_trips`.
- With no telemetry, the open-loop output is the same as the output with the gains at zero.

## Tests

- New test files: `pi_app/tests/test_target_lock_steal.py` (19 tests) and `pi_app/tests/test_rpm_plausibility.py` (28 tests).
- Changed: `test_depth_filter_latch` (it expected the one-frame switch) and `test_tracklet_tracker` (it expected the gains to be 0).
- Full suite: `python3 -m unittest discover -s pi_app/tests -p "test_*.py"` gives 703 tests OK, with 4 skipped.

## Open items

1. Validate on the hardware. Watch `vesc_rpm_plausible` and `vesc_rpm_gate_trips` during the first follow-me drive. If the gate trips on normal drives, the window or the eRPM floor is too tight.
2. Decide finding 3 of the Fable review (refer to the section "Fable review").
3. Walk test with a second person: a person walks between the robot and the operator. The expected result: the robot coasts for less than 1 s and keeps the operator.

## Fable review

A Fable agent reviewed commits 71fce18 and 33635bc. It found no blockers.

- Finding 1 (fixed): `actual_speed_mps` used the mean of the absolute eRPM values. A turn then looked like overspeed, and the loop decreased the throttle in each low-speed arc. The controller now uses the signed mean of the two wheels. The steering difference cancels. The bench test of 2026-06-11 read positive eRPM on the two sides at +1500.
- Finding 2 (fixed): the PID integral stayed after a telemetry loss. The integral now goes to zero on each open-loop tick, when the target is too close, and on a full target-loss reset.
- Finding 3 (open, design decision): after an occlusion, the operator can come back with a new tentative id and more than 0.30 normalized-x away from the held position. The tracker then treats the operator as a challenger, and the robot coasts for up to 1.0 s more.
- Findings 4 to 6 (not changed): ROI depth noise can cause a short hesitation. Some tests prove less than their names say. The gate stays tripped while the robot is parked until the next real motion.

## Technical Names

IoU, eRPM, VESC, YOLOv8, NMS, PID, tracklet, grace hold, open-loop, pole pair, telemetry, `track_id`, `TrackletTracker`, `TargetTracker`, `RpmPlausibilityGate`.
