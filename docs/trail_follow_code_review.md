# Trail Follow / Pure Pursuit — Code Review Findings

**Branch:** `feature/trail-follow-pure-pursuit`
**Date:** 2026-03-10
**Reviewer:** Claude (automated deep-dive, 6 parallel analysis agents)

---

## HIGH Severity (Should fix before merging)

### 1. `imu_status` used before assignment — `main.py:448`
**Type:** Bug (introduced by this branch)

`imu_status` is referenced in the `RecordingTelemetry` construction at line 448 but isn't assigned until line 482. On the first loop iteration, this causes a `NameError` silently caught by the outer `try/except` at line 468. On subsequent iterations, it uses the stale value from the previous cycle.

**Fix:** Initialize `imu_status = None` (or a sentinel) before the telemetry block, or move the `imu_status` assignment above its first use.

---

### 2. Lost-target zero-speed bug — `follow_me.py:_handle_lost_target()`
**Type:** Bug (behavioral)

When `_last_speed_offset` is 0 at the moment the person is lost, `fwd = 0 * 0.5 = 0` but the trail pursuit still produces a nonzero `steer_byte`. This results in `left = NEUTRAL + 0 + steer` and `right = NEUTRAL + 0 - steer` — one motor forward, one reverse. The robot spins in place instead of following the trail.

**Fix:** Ensure a minimum forward speed during lost-target trail following, e.g. `fwd = max(half_speed, MIN_LOST_TARGET_SPEED)`.

---

### 3. No dt clamp in odometry — `odometry.py:update()`
**Type:** Bug (robustness)

If the OAK-D pipeline stalls for a few seconds (thermal throttle, USB hiccup), the large `dt` multiplied by speed causes the pose to teleport. There's no upper bound on `dt`.

**Fix:** Add `dt = min(dt, MAX_DT)` (e.g. 0.2s). Discard the update entirely if `dt` exceeds a threshold.

---

### 4. Simulator uses open-loop odometry — `trail_replay.py`
**Type:** Design flaw

The simulator feeds the *logged* motor bytes to odometry rather than the *simulated* ones. After the first steering divergence, the odometry drifts from what the simulated controller would actually produce, making comparisons unreliable.

**Fix:** Feed the simulator's own output motor bytes back into odometry for a proper closed-loop replay. Optionally keep open-loop as a separate comparison mode.

---

### 5. No lost-target trail-following test
**Type:** Coverage gap (safety-critical path has zero tests)

The `_handle_lost_target()` → trail pursuit path is never tested. This is the most safety-sensitive code path (robot moving autonomously without a visible target).

**Fix:** Add tests for: person lost with trail available, person lost with empty trail, person lost with `_last_speed_offset = 0` (catches bug #2).

---

### 6. No end-to-end integration test
**Type:** Coverage gap

No test exercises the full pipeline: detection → `update_pose()` → `compute()` → trail drop → person lost → trail pursuit → motor bytes. Each module is tested in isolation but the integration is untested.

**Fix:** Add a multi-step test that simulates a person walking, disappearing, and the robot following the trail back.

---

## MEDIUM Severity (Should address before field testing)

### 7. Spin-in-place overestimates speed — `odometry.py`

Speed is computed as `max(0, left-NEUTRAL) + max(0, right-NEUTRAL)` averaged. When one motor goes forward and the other reverse (turning in place), this still reports a forward speed, causing phantom odometry drift.

**Fix:** Use `max(0, min(left, right) - NEUTRAL)` or detect counter-rotating motors and set speed to 0.

---

### 8. No NaN/inf guard on heading — `odometry.py`

If the IMU returns NaN or inf (glitch, sensor fault), `math.cos(NaN)` = NaN, and every subsequent pose update is poisoned forever.

**Fix:** Guard with `if not math.isfinite(heading_deg): return` at the top of `update()`.

---

### 9. No hysteresis on trail/direct mode switching — `follow_me.py`

The decision between trail pursuit and direct pursuit uses fixed thresholds (`direct_pursuit_distance_m`, `direct_pursuit_lateral_m`). When the person oscillates around these thresholds, the controller flips modes every cycle, causing jerky steering.

**Fix:** Add a small hysteresis band (e.g. switch to direct at 2.0m, switch back to trail at 2.5m).

---

### 10. Global closest-point search in pure pursuit — `pure_pursuit.py:45-48`

The closest trail point is found globally via `min(range(len(trail)), ...)`. On a trail that crosses itself, this can snap the lookahead backward to a previously-visited segment.

**Fix:** Maintain a running index and only search forward from the last known closest point.

---

### 11. Lookahead distance from trail point, not robot — `pure_pursuit.py:49-67`

The lookahead walk starts from the closest trail point and accumulates segment lengths. It should measure from the robot's position along the trail to correctly handle cases where the robot is between trail points.

**Fix:** Start accumulation from the robot-to-closest-point distance, not from zero.

---

### 12. Simulator single-detection limitation — `trail_replay.py`

Only the first person detection per frame is used. Multi-person scenarios (e.g. someone walking through frame) aren't replayed correctly.

**Fix:** Pass the full detection list and let `FollowMeController.compute()` handle selection (as it does in production).

---

### 13. `PursuitConfig.max_speed_byte` is dead config

Defined in the dataclass but never referenced anywhere. Misleading for future maintainers.

**Fix:** Remove it, or use it to clamp `speed_byte` in `compute()`.

---

### 14. `speed_hint` stored in TrailPoint but never consumed

`trail.py` stores `speed_hint` on every point, `follow_me.py` passes `_last_speed_offset` as the hint, but `pure_pursuit.py` never reads it.

**Fix:** Either wire it into the pursuit speed (use trail speed hints to modulate following speed) or remove it to avoid dead data.

---

### 15. One-cycle-delayed motor values in `update_pose()` — `controller.py`

`update_pose()` receives `self._slew_last_left/right` which are the *previous* cycle's post-slew values, because the current cycle's slew is computed after `compute()` returns. At 10 Hz this is ~100ms lag in the odometry motor input.

**Fix:** Either reorder to slew → update_pose → compute (requires restructuring), or accept as known limitation and document it. At 10 Hz this is likely acceptable but worth noting.

---

### 16. Simulator default config mismatch — `trail_replay.py`

Always uses `trail_follow_enabled=True` regardless of the original run's config. This means simulated output won't match actual behavior if the original run had trail following disabled.

**Fix:** Read config from the log metadata or accept it as a "what-if" tool (but document this).

---

## LOW Severity (Cleanup / nice-to-have)

| # | Finding | File |
|---|---------|------|
| 17 | `NEUTRAL = 126` duplicated (also in `mapping.py`) | `odometry.py` |
| 18 | `RobotPose` should be `frozen=True` dataclass | `odometry.py` |
| 19 | Misleading test name `test_turns_toward_left_point` (point is actually right in this coord system) | `test_trail_follow.py` |
| 20 | Non-atomic deque rebuild in `prune()` (clear + re-add vs reassignment) | `trail.py` |
| 21 | Many defensive `getattr()` calls on frozen dataclass fields | `follow_me.py` |
| 22 | `heading=0` used as sentinel in simulator but is a valid compass reading (true north) | `trail_replay.py` |
| 23 | O(N*M) heading merge in simulator could be slow on large logs | `trail_replay.py` |

---

## What's Correct (Verified)

These areas were specifically verified and are **solid**:

- **Camera-to-world transform** — sign conventions correct, verified independently by 2 agents
- **Pure pursuit curvature math** — `kappa = 2*local_y/L^2` is textbook correct
- **Full sign chain** — target right → positive local_y → positive kappa → positive steer_byte → left motor faster → robot turns right. Consistent with direct pursuit.
- **Trail ordering** — deque preserves insertion order through all operations
- **Prune logic** — correctly removes behind+close points and aged-out points
- **Min-spacing enforcement** — prevents trail clutter
- **All 21 existing unit tests pass**

---

## Missing: No Real Follow-Me Logs

No actual Follow Me run logs exist in the repo. The simulator cannot be cross-referenced against real-world data yet.
