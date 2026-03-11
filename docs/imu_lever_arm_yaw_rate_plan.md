# IMU Lever-Arm Mitigation Plan (OAK IMU Yaw Path)

## Problem

The IMU is mounted far from the robot yaw pivot (~0.85 m). During turning, the accelerometer sees strong tangential and centripetal acceleration terms in addition to gravity. The current OAK IMU path uses accelerometer direction to project gyro into yaw-rate, which can bias yaw estimates under dynamic turns.

## Goal

Reduce yaw/heading distortion caused by lever-arm-induced linear acceleration without changing hardware placement.

## Strategy (Phase 1)

Switch the OAK yaw-rate path to use direct gyro yaw rate by default, while keeping a config toggle for the previous gravity-projected behavior.

### Why this first

- Minimal code risk
- Immediate reduction in acceleration-coupled yaw error
- Fully reversible via config

## Implementation Steps

1. Add config toggle in `ImuSteeringConfig`:
   - `oak_use_gravity_projected_yaw_rate: bool = False`
2. Thread the new config into `OakImuReader` construction in `main.py`.
3. Update `OakImuReader`:
   - Add constructor flag for projected/direct yaw mode.
   - Factor yaw-rate computation into helper method.
   - Default to direct gyro yaw (`gz`) when toggle is false.
4. Add tests for yaw-rate helper behavior:
   - Direct mode returns `gz`.
   - Projected mode computes projection when accel norm is valid.
   - Projected mode falls back to `gz` when accel norm is too small.
5. Run targeted test suite for new behavior and ensure no regressions in related modules.

## Validation Plan

1. Bench spin test:
   - Command constant turn left/right and compare `yaw_rate_dps` smoothness and symmetry.
2. Field check:
   - Compare heading drift and Follow Me trail stability during turns.
3. If needed:
   - Re-enable projected mode for A/B testing via config toggle.

## Follow-up (Phase 2, optional)

If additional improvement is needed, add lever-arm compensation on accelerometer before tilt/yaw derivation using:

`a_ref = a_imu - alpha x r - omega x (omega x r)`

with measured `r` in robot body frame.
