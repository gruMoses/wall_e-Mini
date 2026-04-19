# Heading Tuning (OAK-D IMU)

This note captures the known-good heading configuration and field test flow used to restore reliable heading behavior.

## Known-Good Settings

In `config.py` (`ImuSteeringConfig`):

- `oak_yaw_rate_source: "auto"`
- `oak_yaw_rate_scale: 0.46`
- `oak_use_gravity_projected_yaw_rate: False`
- `oak_nmni_enabled: True`
- `oak_nmni_threshold_dps: 0.3`
- `oak_bias_adapt_enabled: False`

These settings are tuned for the current OAK-D mounting/orientation and should be treated as the baseline.

## Why This Was Needed

The heading pipeline was initially under-reporting or over-reporting turn magnitude (for example, a physical ~180 degree turn not matching UI heading change). The fix required:

1. selecting the correct yaw-rate source behavior for the OAK frame (`auto` axis lock), and
2. applying an empirical yaw-rate scale (`0.46`) from field turn tests.

## Field Validation Procedure

Use this quick test anytime heading behavior is suspect:

1. Put robot in a safe stationary area.
2. Open web UI and note heading.
3. Rotate robot in place by about 180 degrees clockwise.
4. Note heading again.
5. Repeat counter-clockwise.

Expected result:

- Heading should move by approximately 180 degrees equivalent each turn (allowing some noise and wraparound).
- No long freezes or tiny changes during obvious rotation.

Wraparound reminder:

- A change like `50 -> 35` is a wrapped representation; compute signed delta with wrap handling before judging correctness.

## If Regression Happens

1. Confirm service is using OAK IMU source (startup logs should show `source: oak_d`).
2. Check recent `arm_*.log`:
   - `imu.heading_deg` should vary significantly during rotation.
   - `imu.yaw_rate_dps` should show clear non-zero peaks while turning.
3. If magnitude is consistently off, re-fit `oak_yaw_rate_scale` from a clean in-place turn:
   - `new_scale = old_scale * (expected_turn_deg / measured_unwrapped_turn_deg)`.

