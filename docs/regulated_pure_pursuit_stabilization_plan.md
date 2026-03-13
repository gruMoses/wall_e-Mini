# Regulated Pure Pursuit Stabilization Plan

## Context

Current Follow Me trail behavior is unstable in edge cases:

- Under-steer phase: robot failed to turn enough and felt like it did not follow.
- Over-steer phase: robot kept turning hard and nearly circled.
- Detection dropouts can hold stale trail steering too long.

This is a solved pattern in production robotics: use regulated pure pursuit (adaptive lookahead + curvature-regulated speed + guarded lost-target behavior), then tune with telemetry.

## Goal

Make trail following predictable, non-circling, and robust to brief detection loss while preserving corner-following capability.

## Baseline Signals to Use

Use existing log/MCAP fields:

- `pursuit_mode`, `speed_offset`, `steer_offset`
- `trail_length`, `trail_distance_m`
- `trail_rejected_jump_count`, `trail_rejected_speed_count`
- `target_world_x/y`, `trail_lookahead_x/y`
- `odom_x/y/theta_deg`, `heading_deg`, `num_persons`

## Phase 1 - Control Law Corrections (highest priority)

1. Add curvature-regulated speed in trail mode.
   - Reduce forward speed as curvature increases.
   - Keep a configured minimum speed floor.
   - Prevent high speed + high steer combinations that cause overshoot.
2. Add lost-target trail steer decay.
   - When `num_persons == 0`, decay steer magnitude over time.
   - Keep short blind pursuit for continuity, but prevent sustained hard arcs.
3. Add forward-progress guard in pure pursuit.
   - Reject lookahead candidates effectively behind the robot.
   - Require monotonic trail progress (no backtracking carrot selection).

## Phase 2 - Parameter Restructure

Add explicit, named parameters (instead of tuning by wheelbase alone):

- `trail_curvature_speed_reg_enabled`
- `trail_curvature_speed_min_byte`
- `trail_curvature_speed_gain`
- `trail_lost_steer_decay_tau_s`
- `trail_lost_steer_max_byte`
- `trail_forward_lookahead_min_m`

Keep existing fields for compatibility; prefer new parameters in controller logic.

## Phase 3 - Heading and Odometry Validation

Before tuning, verify heading is not stale:

1. In a manual spin test, confirm `heading_deg` and `odom_theta_deg` both change with turn.
2. Add a runtime warning if heading variance stays near zero while steering commands are non-zero.
3. If stale heading is detected, fail safe to direct pursuit or reduced-steer mode.

## Phase 4 - Safety and Regression Guards

1. Add unit tests for:
   - curvature speed regulation behavior
   - lost-target steer decay
   - no-lookahead-behind guard
2. Add log assertions in analysis scripts:
   - `% frames |steer_offset| >= 10` must stay below threshold
   - no long windows of constant high steer with `num_persons == 0`
3. Keep breadcrumb outlier filtering active; monitor reject counters each run.

## Test Protocol

Run in order, short sessions first:

1. Straight walk, no turns.
2. Gentle turn (~15-25 deg).
3. Medium corner with brief occlusion.
4. Sharp corner, then reacquisition.

For each run, report:

- trail/direct frame counts
- `avg|steer|`, `max|steer|`, saturation count (`|steer| >= 10`)
- longest `num_persons == 0` window during FOLLOW_ME
- heading span and odom heading span
- breadcrumb reject counts

## Acceptance Criteria

Ship candidate only if all pass:

1. No circling behavior in sharp corner test.
2. No sustained high-steer arc during target loss.
3. Robot follows turns without major corner cutting.
4. Reacquires after brief occlusion without full stop in normal cases.
5. Heading telemetry is confirmed live and consistent with motion.

## Rollout Strategy

1. Implement Phase 1 with conservative defaults.
2. Validate heading (Phase 3) before aggressive tuning.
3. Tune one parameter family at a time using the test protocol.
4. Keep a known-good config checkpoint after each successful pass.

## Notes for Next Implementer

- Prefer small, reversible parameter moves and compare paired runs.
- Do not change multiple steering mechanisms at once.
- Preserve telemetry field names to keep analysis scripts stable.
