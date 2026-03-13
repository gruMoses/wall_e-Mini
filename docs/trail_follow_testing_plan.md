# Trail-Following Pure Pursuit — Testing & Calibration Plan

**Branch:** `feature/trail-follow-pure-pursuit`
**Created:** 2026-03-13
**Robot:** WALL-E Mini — rear-wheel-drive skid-steer, front casters, OAK-D Lite camera (sideways mount), IMU heading via gravity-projected gyro

---

## Table of Contents

1. [System Overview (Read This First)](#1-system-overview)
2. [Known State Going In](#2-known-state-going-in)
3. [Equipment & Safety Checklist](#3-equipment--safety-checklist)
4. [Testing Order (Why This Sequence Matters)](#4-testing-order)
5. [Phase 0: Pre-Flight Checks](#5-phase-0-pre-flight-checks)
6. [Phase 1: Speed Scale Calibration](#6-phase-1-speed-scale-calibration)
7. [Phase 2: Direct Follow Me Steering Gains](#7-phase-2-direct-follow-me-steering-gains)
8. [Phase 3: Straight-Line Trail Following](#8-phase-3-straight-line-trail-following)
9. [Phase 4: Adaptive Lookahead Tuning](#9-phase-4-adaptive-lookahead-tuning)
10. [Phase 5: Curvature Velocity Scaling](#10-phase-5-curvature-velocity-scaling)
11. [Phase 6: Corner & Occlusion Scenarios](#11-phase-6-corner--occlusion-scenarios)
12. [Phase 7: Full Integration Stress Test](#12-phase-7-full-integration-stress-test)
13. [Parameter Quick Reference](#13-parameter-quick-reference)
14. [Diagnostic Playbook](#14-diagnostic-playbook)
15. [Log Analysis Guide](#15-log-analysis-guide)

---

## 1. System Overview

Read this section before testing so you understand what each component does. The trail-following system has four layers:

```
Person Detection (OAK-D Lite)
    │
    ▼
Follow Me Controller (follow_me.py)
    ├── Direct Pursuit: PD controller on lateral offset (close/centered person)
    └── Trail Pursuit: Pure Pursuit on breadcrumb path (far/off-center person)
            │
            ├── Odometry (odometry.py): dead-reckoning pose from IMU heading + motor bytes
            ├── Trail Manager (trail.py): breadcrumb deque + Savitzky-Golay smoothing
            └── Pure Pursuit (pure_pursuit.py): path-following with:
                    ├── Forward-only closest-point search (no backward snap)
                    ├── Adaptive lookahead = clamp(speed_mps × time_s, min, max)
                    └── Curvature-based velocity scaling (slow before turns)
```

**Motor output assembly** (differential drive):
```
left  = 126 + speed_offset + steer_offset
right = 126 + speed_offset - steer_offset
```
Where 126 is motor neutral (stopped). Range 0-254.

**Mode selection** (hysteresis to prevent flapping):
- **Direct pursuit** when person is close (< 3.5m) AND centered (< 1.0m lateral)
- **Trail pursuit** when person is far (> 4.55m) OR off-center (> 1.5m lateral)
- Switches require crossing the opposite threshold (not just the entry threshold)

**Lost target behavior** (3 stages):
1. **Trail blind pursuit** (up to 8s): keeps following breadcrumb trail at 50% last speed
2. **Search turn** (up to 3.5s): turns toward last known person direction
3. **Full stop**: clears trail, resets odometry, returns to neutral

---

## 2. Known State Going In

### What's working well (March 12 field test)
- IMU heading-hold PID is stable: `kp=0.7, ki=0.08, kd=0.5, max_correction=25`
- Speed-dependent gain scheduling prevents oscillation at high speed
- Direct Follow Me baseline (trail disabled) tracks straight + gentle turns well
- Detection stream is reliable in daylight at 1.5-6m range

### Known issues to address
1. **Follow Me steering_gain was 2.02** (March 8 session) — caused violent oscillation. Config.py now shows `steering_gain=0.35` and `steering_derivative_gain=0.06`, but these have NOT been field tested yet. The 2.02 value came from a broken auto-calibration run (IMU was stuck at 0° during calibration).
2. **Corner-cutting**: one tight corner clipped a building edge. This is the primary motivation for trail following — follow the path the person walked, not a straight line.
3. **Detection dropout stops**: person going around corners causes detection loss > timeout, triggering premature full stop. Extended timeout to 3.5s short / 8.0s trail pursuit, but untested with new pure pursuit upgrades.
4. **Speed scale calibration**: `trail_speed_scale_mps_per_byte=0.0016` was calibrated from VESC runs. Needs field verification with odometry logging.

### New features (not yet field tested)
- Savitzky-Golay path smoothing (5-point window, 2nd-order polynomial)
- Adaptive lookahead: `lookahead = clamp(speed_mps × 0.8s, 0.5m, 2.5m)`
- Forward-only closest-point search (window of 20 points, no backward snap)
- Curvature-based velocity scaling: `speed = base / (1 + 5.0 × max_curvature)`
- Pre-deceleration: looks 5 points ahead on curvature profile
- Acceleration-limited speed transitions: max 50 bytes/s change rate

---

## 3. Equipment & Safety Checklist

### Required
- [ ] Charged battery (full charge recommended — testing takes 30+ min)
- [ ] RC transmitter powered on and bound (manual override always available)
- [ ] Laptop connected to Pi via SSH for log access
- [ ] Phone/tablet open to web viewer: `http://<pi-ip>:8080`
- [ ] Tape measure (at least 10m)
- [ ] 4-5 traffic cones or markers for waypoint/distance references
- [ ] Open flat area, minimum 15m × 10m, no pedestrians (Phase 1-5)
- [ ] Area with corners/obstacles for Phase 6 testing

### Safety rules
- **Always have RC transmitter in hand** — manual override is immediate
- **Spotter required** for all corner/obstacle tests (Phase 6+)
- **Start each phase at LOW speed** — increase only after confirming stable behavior
- **Never test near drop-offs, roads, or fragile objects** until Phase 7
- **If robot oscillates violently, immediately disarm via RC** — do not wait to see if it corrects
- The robot weighs ~15 kg and can move at ~1.5 m/s. Respect it.

### Before first power-on
- [ ] Verify `feature/trail-follow-pure-pursuit` branch is deployed: `git log --oneline -1`
- [ ] Verify service starts clean: `sudo systemctl restart wall-e.service && journalctl -u wall-e.service -f`
- [ ] Confirm web viewer loads at `http://<pi-ip>:8080`
- [ ] Confirm RC arm/disarm works (CH5 switch)
- [ ] Confirm manual RC driving works (drive forward 1m, turn, stop)

---

## 4. Testing Order (Why This Sequence Matters)

The standard calibration order for pure pursuit on differential-drive robots is well-established in the ROS Nav2 community and robotics literature. You calibrate from the bottom up: physical measurements first, then low-level control, then path following, then high-level features. Each phase depends on the one before it.

```
Phase 0: Pre-flight checks          ← does the system boot and respond?
Phase 1: Speed scale calibration    ← odometry accuracy (everything depends on this)
Phase 2: Direct steering gains      ← basic Follow Me must work before trail mode
Phase 3: Straight-line trail follow ← isolate trail tracking from curvature effects
Phase 4: Adaptive lookahead tuning  ← primary oscillation/corner-cutting control
Phase 5: Curvature velocity scaling ← speed regulation in turns
Phase 6: Corner & occlusion tests   ← the actual goal: follow around obstacles
Phase 7: Full integration stress    ← long runs, mixed terrain, edge cases
```

**Do not skip phases.** If Phase 1 (speed scale) is wrong, the adaptive lookahead in Phase 4 will compute wrong distances, curvature scaling in Phase 5 will trigger at wrong times, and everything downstream fails.

**Do not tune multiple things at once.** Change one parameter, run one test, observe. This is the #1 mistake in pure pursuit tuning (per Nav2 tuning guide and ROS Discourse threads). If you change lookahead AND curvature alpha AND speed simultaneously, you cannot diagnose what helped or hurt.

---

## 5. Phase 0: Pre-Flight Checks

**Goal:** Confirm the system boots, sensors work, and basic RC control is functional.

### Steps

1. **SSH into Pi, check service status:**
   ```bash
   sudo systemctl status wall-e.service
   ```
   Expected: `active (running)`, no crash loops.

2. **Check OAK-D Lite camera:**
   - Open web viewer → confirm RGB stream shows live video
   - Confirm depth stream shows colorized depth
   - Stand 2m in front → confirm person detection box appears on RGB stream

3. **Check IMU:**
   - In SSH, watch recent log: `tail -f logs/latest.log | python3 -m json.tool`
   - Look for `"heading_deg"` field changing as you rotate robot by hand
   - Verify heading is NOT stuck at 0° (this was a previous bug — see imu_tuning_session_20260308.md)

4. **Check RC control:**
   - Arm robot (CH5 switch)
   - Drive forward ~2m, turn left, turn right, stop
   - Disarm

5. **Check Follow Me activation:**
   - Arm robot, activate Follow Me (web button or gesture: 3-4-3 finger sequence)
   - Stand 2m in front — robot should inch forward
   - Deactivate Follow Me (open palm "FIVE" gesture or web button)
   - Disarm

### Pass criteria
- [ ] All 5 checks above pass
- [ ] No error messages in `journalctl` output
- [ ] IMU heading responds to physical rotation
- [ ] At least one person detection visible in web viewer

### If something fails
- Camera not connecting: check USB-C cable, `dmesg | tail -20` for OAK errors
- IMU stuck at 0°: see `docs/imu_tuning_session_20260308.md` Problem 1
- Follow Me won't activate: check `config.py` → `follow_me.enabled = True`

---

## 6. Phase 1: Speed Scale Calibration

**Goal:** Verify that `trail_speed_scale_mps_per_byte = 0.0016` is accurate. This constant converts motor byte offset from neutral into meters/second, and the entire odometry system depends on it.

**Why this matters:** If speed_scale is wrong:
- Odometry position drifts → trail points placed in wrong locations → pursuit tracks wrong path
- Adaptive lookahead computes wrong distance → too short (oscillation) or too long (corner cutting)
- Curvature velocity scaling triggers at wrong speeds

### Test 1A: Measured straight-line drive

**Setup:**
1. Place two cones 5.0m apart on flat ground (use tape measure)
2. Position robot at cone 1, facing cone 2
3. Mark start position with tape

**Procedure:**
1. SSH into Pi. Start watching logs:
   ```bash
   tail -f logs/latest.log | grep -o '"motor":{[^}]*}' | head -20
   ```
2. Arm robot. Drive straight forward using RC at moderate constant throttle.
   - Target: a steady motor byte around 170-180 (offset of ~45-55 from neutral 126)
   - Hold it as steady as possible for the full 5m
3. Stop when front of robot reaches cone 2. Disarm.
4. Note the elapsed time (count "one-mississippi" or use phone timer)
5. Read the average motor byte from logs

**Calculation:**
```
actual_speed_mps = 5.0m / elapsed_seconds
motor_offset = avg_motor_byte - 126
measured_scale = actual_speed_mps / motor_offset
```

**Expected:** ~0.0016 m/s per byte (current calibration)

**Acceptance:**
- If measured is within 20% of 0.0016 (i.e., 0.0013 to 0.0019): **current value is fine**, move on
- If measured differs by more than 20%: update `trail_speed_scale_mps_per_byte` in `config.py`

### Test 1B: Odometry drift check

**Setup:** Same cones at 5m.

**Procedure:**
1. Enable Follow Me mode, stand at cone 2 (robot at cone 1)
2. Walk slowly toward robot, letting it follow you back to cone 1 position
3. After robot reaches cone 2's original position, stop Follow Me
4. Check logged `odom_x`, `odom_y` from telemetry (web viewer or logs)
5. The reported forward distance should be approximately 5.0m

**Acceptance:**
- Odometry reports 4.0-6.0m of forward travel for a 5m physical drive: **acceptable**
- Reports < 3m or > 7m: speed_scale is significantly wrong, recalibrate

### Test 1C: Repeat on target surface

If your testing surface differs from pavement (grass, dirt, gravel), repeat Test 1A on that surface. Skid-steer robots have significant surface-dependent slip. The speed_scale may need to be different for different surfaces.

---

## 7. Phase 2: Direct Follow Me Steering Gains

**Goal:** Confirm direct pursuit (PD controller) is stable before enabling trail mode. The config currently has `steering_gain=0.35` and `steering_derivative_gain=0.06` — these need field validation.

**Why before trail mode:** Trail pursuit uses direct pursuit as a fallback for close/centered targets. If direct pursuit oscillates, trail mode will oscillate too when it hands off.

**Important:** For this entire phase, temporarily disable trail following:
```python
# In config.py, FollowMeConfig:
trail_follow_enabled: bool = False    # TEMPORARILY set to False for Phase 2
```
Restart the service after changing.

### Test 2A: Straight-line follow (stability check)

**Procedure:**
1. Stand 3m in front of robot in open area
2. Activate Follow Me
3. Walk slowly forward in a straight line for ~10m
4. Stop and let robot settle at follow distance

**Observe (on web viewer):**
- Robot should track you smoothly with minimal side-to-side wandering
- `follow_me_steer_offset` should stay within ±5 bytes during straight walk
- No oscillation (rapid alternation between positive/negative steer values)

**If oscillation occurs:**
- Reduce `steering_gain` by 50% (e.g., 0.35 → 0.18)
- Restart service, repeat test
- Continue halving until stable, then increase by 25% to find sweet spot

### Test 2B: Gentle turn tracking

**Procedure:**
1. Activate Follow Me, start walking straight
2. After 5m, gradually turn ~30° to the left over 2-3 steps
3. Continue straight on new heading for 5m
4. Repeat with a right turn

**Observe:**
- Robot should smoothly curve to follow your new heading
- No overshooting (robot swings past your new line then corrects back)
- `follow_me_steer_offset` peaks during turn then settles to ~0

**If overshoot occurs:**
- Increase `steering_derivative_gain` by 50% (adds damping)
- If that causes jitter, increase `steering_ema_alpha` toward 0.5 (more smoothing)

### Test 2C: Lateral offset tracking

**Procedure:**
1. Activate Follow Me, walk straight
2. Sidestep 1m to the right while continuing forward
3. Hold the new offset for 3-4 steps, then sidestep back to center

**Observe:**
- Robot adjusts heading to close the lateral gap over 2-4 seconds
- No violent snap-to-center
- Robot returns to centered tracking after you return to center

### Test 2D: Speed variation

**Procedure:**
1. Walk slowly (shuffling pace) for 5m → robot should follow at low speed
2. Walk briskly for 5m → robot should increase speed
3. Stop abruptly → robot should slow and stop at follow distance (~1.5m)

**Observe:**
- `follow_me_speed_offset` correlates with your walking speed
- No lurching on speed transitions
- Robot stops within 0.5-1.0m of the desired 1.5m follow distance

### Pass criteria for Phase 2
- [ ] Straight-line: steer_offset stays within ±5 bytes
- [ ] Gentle turn: smooth curve, no overshoot
- [ ] Lateral step: converges within 3s, no oscillation
- [ ] Speed variation: smooth speed transitions, clean stop
- [ ] No incidents of violent oscillation or target loss during testing

### After Phase 2 passes
Re-enable trail following:
```python
trail_follow_enabled: bool = True    # Restore to True
```
Restart service.

---

## 8. Phase 3: Straight-Line Trail Following

**Goal:** Verify trail pursuit works on the simplest possible case — a straight line. This isolates path-following accuracy from curvature effects.

**Why straight lines first:** If the robot oscillates on a straight trail, the problem is in the core pursuit math (lookahead, closest-point search, steering gain), not in curvature handling. Diagnosing is much easier on a straight line.

### Test 3A: Trail mode activation

**Procedure:**
1. Stand 5m in front of robot (beyond `direct_pursuit_distance_m = 3.5m`)
2. Activate Follow Me
3. Walk forward slowly in a straight line

**Observe (in web viewer telemetry or logs):**
- `follow_me_pursuit_mode` should show `"trail"` when you're > 4.5m away
- `trail_length` should grow as you walk (new breadcrumbs added every 0.3m)
- `odom_x`, `odom_y` should advance forward
- Robot tracks you smoothly

**Acceptance:**
- [ ] Trail mode activates (pursuit_mode = "trail")
- [ ] Trail length grows to 5+ points over 5m of walking
- [ ] Robot follows at consistent distance without oscillation

### Test 3B: Direct/trail mode handoff

**Procedure:**
1. Start 5m away, activate Follow Me (trail mode activates)
2. Walk toward robot until you're at ~2m (direct pursuit should take over)
3. Walk away again to 5m (trail mode should re-engage)

**Observe:**
- `follow_me_pursuit_mode` transitions smoothly between `"direct"` and `"trail"`
- No jerky steering during transitions
- No stop or hesitation during mode switch

**Acceptance:**
- [ ] Clean handoff with no visible jerk or pause
- [ ] Mode switches logged correctly in telemetry

### Test 3C: Straight-line cross-track error

**Setup:**
1. Place cones every 2m along a 10m straight line
2. Position robot at one end, stand at the other end

**Procedure:**
1. Activate Follow Me
2. Walk the straight line (staying on the cone line)
3. Observe robot's physical deviation from the cone line

**Acceptance:**
- Robot stays within ±0.3m of the cone line: **excellent**
- Within ±0.5m: **acceptable, may improve with lookahead tuning in Phase 4**
- Outside ±0.5m: **problem — likely speed_scale or lookahead issue**

---

## 9. Phase 4: Adaptive Lookahead Tuning

**Goal:** Tune the adaptive lookahead distance for the right balance between smooth tracking (no oscillation) and tight path following (no corner cutting).

**Background:** The lookahead distance is the single most important pure pursuit parameter. It determines how far ahead on the trail the robot aims:
- **Too short** → robot reacts to every small trail deviation → oscillation/wiggling
- **Too long** → robot cuts corners and ignores trail curvature → defeats the purpose

The formula is: `lookahead = clamp(speed_mps × lookahead_time_s, min_m, max_m)`

Current values:
```
pursuit_lookahead_time_s = 0.8     # 0.8 seconds of look-ahead at current speed
pursuit_lookahead_min_m = 0.5      # Never look closer than 0.5m
pursuit_lookahead_max_m = 2.5      # Never look farther than 2.5m
```

### Test 4A: Low-speed lookahead (oscillation check)

**Procedure:**
1. Stand 4m away, activate Follow Me
2. Walk forward very slowly (~0.5 m/s human walking pace)
3. Walk a gentle S-curve (2m amplitude, 8m wavelength)

**At low speed, the lookahead = max(0.5m, speed_mps × 0.8s)**
- At 20 byte offset × 0.0016 = 0.032 m/s → lookahead = 0.5m (minimum)

**Observe:**
- Does the robot oscillate (wiggle side-to-side) on the straight portions?
- Does it track the S-curve reasonably?

**If oscillating at low speed:**
- Increase `pursuit_lookahead_min_m` from 0.5 to 0.7 or 0.8
- Longer minimum lookahead = more damping but slightly more corner cutting

### Test 4B: Medium-speed lookahead

**Procedure:**
1. Stand 5m away, walk at normal pace (~1.0 m/s)
2. Walk a straight line, then a 45° turn, then straight

**At medium speed:**
- At 50 byte offset × 0.0016 = 0.08 m/s → lookahead = max(0.5, 0.08 × 0.8) = 0.5m
- (The robot's max follow speed of 115 bytes × 0.0016 = 0.184 m/s → lookahead = max(0.5, 0.147) = 0.5m)

**Note:** With the current speed_scale of 0.0016, the robot's max speed of 0.184 m/s × 0.8s = 0.147m, which means the lookahead will almost always be at the 0.5m minimum. This is fine for now but worth noting — the adaptive range mainly matters if you increase speed_scale or max_follow_speed_byte later.

**Observe:**
- Smooth tracking through the 45° turn?
- Robot follows the path you walked, not cutting the corner?

### Test 4C: Lookahead time sensitivity

If the robot is oscillating and increasing `min_m` doesn't help, or if it's cutting corners:

**To reduce oscillation:** Increase `pursuit_lookahead_time_s` (e.g., 0.8 → 1.2 → 1.5)
**To reduce corner cutting:** Decrease `pursuit_lookahead_time_s` (e.g., 0.8 → 0.5)

Only change this if min_m adjustment alone doesn't solve the problem.

### Pass criteria for Phase 4
- [ ] No oscillation on straight-line segments at any speed
- [ ] Robot visibly follows the curved path (not cutting straight to person)
- [ ] No jerky steering transitions

---

## 10. Phase 5: Curvature Velocity Scaling

**Goal:** Verify the robot slows down before entering turns and maintains stable tracking through curves.

**Background:** The curvature velocity scaling system:
1. Computes curvature at each trail point using the three-point circle method
2. Looks 5 points ahead for pre-deceleration (starts slowing before the turn)
3. Scales speed: `speed = base_speed / (1 + alpha × max_curvature)`
4. Limits acceleration to 50 bytes/s for smooth transitions

Current values:
```
pursuit_curvature_scaling_enabled = True
pursuit_curvature_alpha = 5.0          # higher = more slowdown in turns
pursuit_min_speed_byte = 15.0          # floor speed (never fully stop in turns)
pursuit_lookahead_curvature_points = 5 # look 5 points ahead for pre-decel
pursuit_max_accel_byte_per_s = 50.0    # smooth speed changes
```

### Test 5A: Curvature scaling activation

**Procedure:**
1. Walk a large circle (~3m radius) with Follow Me active
2. Watch telemetry: `trail_curvature_at_lookahead` and `trail_speed_limited`

**Observe:**
- `trail_curvature_at_lookahead` should be non-zero during the curve (~0.33 for 3m radius)
- `trail_speed_limited` should be `true` during the curve
- Robot should visibly slow down in the curve compared to straight sections

**If curvature is always 0:** Trail points may be too sparse or too noisy. Check `trail_length` — need at least 5+ points to compute meaningful curvature.

### Test 5B: Speed reduction magnitude

**Procedure:**
1. Walk straight at brisk pace for 5m (establish speed baseline)
2. Make a 90° turn over ~2m
3. Continue straight for 5m

**Expected behavior:**
- At the 90° turn with curvature ≈ 1.0/m: `speed = base / (1 + 5.0 × 1.0) = base / 6.0` — that's 83% reduction
- The robot should noticeably slow through the turn

**If robot slows TOO much:**
- Decrease `pursuit_curvature_alpha` (e.g., 5.0 → 3.0 → 2.0)
- Lower alpha = less speed reduction in turns

**If robot doesn't slow enough (overshoots the turn):**
- Increase `pursuit_curvature_alpha` (e.g., 5.0 → 7.0 → 10.0)
- Also consider increasing `pursuit_lookahead_curvature_points` from 5 to 7 for earlier pre-deceleration

### Test 5C: Pre-deceleration timing

**Procedure:**
1. Walk straight for 5m, then make a sharp 90° turn
2. Watch when the robot starts slowing relative to the turn entry point

**Observe:**
- Robot should begin decelerating ~1-2m before the turn (the 5-point lookahead on curvature)
- If it only slows after entering the turn, increase `pursuit_lookahead_curvature_points` from 5 to 7-10
- If it slows way too early (3m+ before the turn), decrease to 3

### Test 5D: Acceleration limiting smoothness

**Procedure:**
1. Walk through a series of turns: straight → left turn → straight → right turn
2. Focus on speed transitions (when robot accelerates out of turns)

**Observe:**
- Speed changes should be gradual, not jerky
- `pursuit_max_accel_byte_per_s = 50.0` means full speed recovery takes ~2s from minimum

**If too jerky:** Decrease max_accel_byte_per_s (e.g., 50 → 30)
**If too sluggish** (robot is slow to recover speed after turns): Increase to 70-100

### Pass criteria for Phase 5
- [ ] Robot visibly slows before tight turns
- [ ] `trail_speed_limited` = true during curves, false on straights
- [ ] Smooth speed transitions (no jerky acceleration/deceleration)
- [ ] Robot maintains tracking through curves (no overshoot or cutting)

---

## 11. Phase 6: Corner & Occlusion Scenarios

**Goal:** Test the primary use case — following a person around obstacles without cutting corners or losing track.

**SAFETY:** Have a spotter positioned near any obstacle the robot might collide with. Start with wide clearances and gradually tighten.

### Test 6A: Wide corner (building corner, 2m clearance)

**Setup:** Find a building corner. Place a cone 2m away from the corner edge.

**Procedure:**
1. Start robot 5m from corner, you between robot and corner
2. Activate Follow Me
3. Walk around the corner, staying outside the cone (2m from wall)
4. Continue straight on the other side for 5m

**Observe:**
- Does the robot follow your path around the corner? Or does it cut toward the wall?
- Does it lose detection when you disappear around the corner?
- Does trail blind pursuit carry it through the corner?

**Key log fields to check:**
- `follow_me_tracking` — does it go false when you round the corner?
- `follow_me_pursuit_mode` — should stay "trail" during the corner
- `trail_length` — should have enough points to navigate the corner
- Duration of `num_detections=0` spans — how long is the robot blind?

**Acceptance:**
- [ ] Robot follows path around corner (stays > 1m from wall)
- [ ] No collision or near-miss with wall
- [ ] If detection is lost, trail pursuit continues for at least 3-5 seconds
- [ ] Robot reacquires person on the other side of the corner

### Test 6B: Brief occlusion (walk behind a pole/tree)

**Procedure:**
1. Activate Follow Me, walk toward a narrow obstacle (pole, tree, mailbox)
2. Walk behind it so you're briefly occluded (1-2 seconds)
3. Emerge on the other side, continue walking

**Observe:**
- Robot should NOT stop during the 1-2s occlusion
- Trail pursuit should maintain motion
- Robot should smoothly resume tracking after reacquisition

### Test 6C: Extended occlusion (walk behind a building, 4-5s out of view)

**Procedure:**
1. Walk around a building corner where you'll be out of view for 4-5 seconds
2. The 8-second `lost_target_trail_pursuit_max_s` should keep the robot moving

**Observe:**
- Robot continues trail-following during the blind period
- Speed decays to 50% of last speed (lost target behavior)
- Robot emerges on the other side of the corner and reacquires

**If robot stops prematurely:**
- Check if `lost_target_trail_pursuit_max_s` is actually 8.0 in config
- Check if trail had enough points when detection was lost
- May need to increase trail_pursuit_max_s to 10-12s for longer corners

### Test 6D: Tight corner (1m from wall)

**Only attempt after 6A passes.** This is the hard case.

**Procedure:**
1. Walk around a corner staying ~1m from the wall
2. Robot must follow your path without getting closer than 0.5m to the wall

**If robot cuts the corner:**
- This means the lookahead is too long (aiming past the turn) or trail points are too sparse
- Try reducing `pursuit_lookahead_min_m` from 0.5 to 0.3
- Try reducing `trail_min_spacing_m` from 0.3 to 0.2 (denser breadcrumbs in turns)
- Verify `trail_smoothing_window` isn't over-smoothing the corner (reduce from 5 to 3 if needed)

### Pass criteria for Phase 6
- [ ] Wide corner (2m): follows path, no wall contact
- [ ] Brief occlusion: no stop, smooth reacquisition
- [ ] Extended occlusion: maintains trail pursuit for at least 5s
- [ ] Tight corner (1m): follows path within 0.3m of your walked path (stretch goal)

---

## 12. Phase 7: Full Integration Stress Test

**Goal:** Verify the complete system works reliably over extended runs with varied conditions.

### Test 7A: Long continuous follow (3-5 minutes)

**Procedure:**
1. Walk a varied path: straights, gentle turns, speed changes, stops
2. Maintain Follow Me for 3-5 continuous minutes
3. Do NOT walk around tight obstacles yet — just open-field variety

**Observe:**
- Robot maintains follow distance throughout
- No random stops or loss of tracking
- Smooth speed and steering throughout
- Trail mode engages during turns, direct mode for straight close follow

### Test 7B: Stop-and-go

**Procedure:**
1. Walk 5m, stop for 3s, walk again
2. Repeat 5 times
3. Robot should settle at follow distance during each stop, then resume

### Test 7C: Approach and retreat

**Procedure:**
1. Walk toward robot (getting closer than follow distance)
2. Robot should stop or back up gently
3. Walk away — robot should resume following

### Test 7D: Mixed terrain (if applicable)

**Procedure:**
1. Walk from pavement to grass or dirt
2. Observe whether tracking degrades on softer surface
3. If robot consistently overshoots or undershoots on new surface, the speed_scale may need surface-specific adjustment

### Test 7E: Full obstacle course

**Procedure:**
1. Set up a course with: straight → wide corner → straight → brief occlusion → straight → sharp turn
2. Walk the course at normal pace
3. Repeat 3 times

**Acceptance for Phase 7:**
- [ ] 3+ minute continuous follow without unexpected stops
- [ ] Clean stop-and-go behavior
- [ ] No crashes or near-misses on obstacle course
- [ ] Reliable detection reacquisition after occlusions

---

## 13. Parameter Quick Reference

All parameters are in `config.py` → `FollowMeConfig`. Restart the service after any change:
```bash
sudo systemctl restart wall-e.service
```

### Speed & Distance
| Parameter | Default | What it does | Tune when... |
|-----------|---------|--------------|--------------|
| `follow_distance_m` | 1.5 | Desired distance from person | Robot follows too close/far |
| `max_follow_speed_byte` | 115 | Maximum forward speed in byte units | Robot too fast/slow |
| `max_speed_error_m` | 2.5 | Distance error at which max speed is reached | Speed ramps too fast/slow |
| `trail_speed_scale_mps_per_byte` | 0.0016 | Motor byte → m/s conversion | Odometry drifts significantly |

### Direct Pursuit Steering
| Parameter | Default | What it does | Tune when... |
|-----------|---------|--------------|--------------|
| `steering_gain` | 0.35 | Proportional gain on lateral offset | Oscillation (↓) or sluggish (↑) |
| `steering_derivative_gain` | 0.06 | Derivative gain (damping) | Overshoot (↑) or jitter (↓) |
| `steering_ema_alpha` | 0.3 | Lateral offset smoothing (0=heavy, 1=none) | Jittery steering (↓) or laggy (↑) |
| `max_steer_offset_byte` | 15.0 | Maximum steering differential | Turns too sharp (↓) or too wide (↑) |

### Trail Management
| Parameter | Default | What it does | Tune when... |
|-----------|---------|--------------|--------------|
| `trail_min_spacing_m` | 0.3 | Min distance between breadcrumbs | Corners cut (↓ to 0.2) or too many points (↑) |
| `trail_max_age_s` | 30.0 | Breadcrumb expiry time | Stale trail after stops (↓) |
| `trail_consume_radius_m` | 0.4 | How close robot gets to "eat" a waypoint | Robot circles points (↓) or skips them (↑) |
| `trail_max_points` | 100 | Maximum breadcrumb buffer size | Usually don't change |

### Path Smoothing
| Parameter | Default | What it does | Tune when... |
|-----------|---------|--------------|--------------|
| `trail_smoothing_enabled` | True | Enable Savitzky-Golay smoothing | Disable to isolate smoothing issues |
| `trail_smoothing_window` | 5 | SG window size (must be odd, ≥ 3) | Over-smoothing corners (↓ to 3) |
| `trail_smoothing_poly_order` | 2 | SG polynomial order (< window) | Rarely needs changing |

### Pure Pursuit
| Parameter | Default | What it does | Tune when... |
|-----------|---------|--------------|--------------|
| `pursuit_lookahead_time_s` | 0.8 | Lookahead = speed × this | Oscillation (↑) or corner cutting (↓) |
| `pursuit_lookahead_min_m` | 0.5 | Floor on lookahead distance | Low-speed oscillation (↑) |
| `pursuit_lookahead_max_m` | 2.5 | Ceiling on lookahead distance | High-speed corner cutting (↓) |
| `pursuit_wheelbase_m` | 0.28 | Effective track width for steering | Steering too aggressive (↑) or weak (↓) |

### Curvature Velocity Scaling
| Parameter | Default | What it does | Tune when... |
|-----------|---------|--------------|--------------|
| `pursuit_curvature_scaling_enabled` | True | Slow down in turns | Disable to isolate speed issues |
| `pursuit_curvature_alpha` | 5.0 | Slowdown aggressiveness (higher = more) | Too slow in turns (↓) or overshooting (↑) |
| `pursuit_min_speed_byte` | 15.0 | Minimum speed in turns | Stalling in turns (↑) |
| `pursuit_lookahead_curvature_points` | 5 | How far ahead to look for pre-deceleration | Late braking (↑) or too early (↓) |
| `pursuit_max_accel_byte_per_s` | 50.0 | Speed change rate limit | Jerky transitions (↓) or sluggish recovery (↑) |

### Mode Selection & Lost Target
| Parameter | Default | What it does | Tune when... |
|-----------|---------|--------------|--------------|
| `direct_pursuit_distance_m` | 3.5 | Below this = direct pursuit | Trail mode too aggressive close up (↑) |
| `direct_pursuit_lateral_m` | 1.0 | Below this lateral = direct pursuit | Mode flapping (↑) |
| `lost_target_timeout_s` | 3.5 | Search-turn timeout | Premature stops (↑) or too long blind (↓) |
| `lost_target_trail_pursuit_max_s` | 8.0 | Trail blind-pursuit timeout | Stops before rounding corners (↑) |

---

## 14. Diagnostic Playbook

### Symptom: Robot oscillates side-to-side on straights

**In direct mode (`pursuit_mode = "direct"`):**
1. Check `steering_gain` — if > 1.0, it's too high (was 2.02, caused violent oscillation)
2. Reduce `steering_gain` by 50%, test again
3. If still oscillating, increase `steering_ema_alpha` (more smoothing)
4. If oscillation is fast (2+ Hz), increase `steering_derivative_gain` for damping

**In trail mode (`pursuit_mode = "trail"`):**
1. Check `pursuit_lookahead_min_m` — increase from 0.5 to 0.7-1.0
2. Check trail smoothing — if disabled, enable it
3. Check `trail_min_spacing_m` — if very small (< 0.2), noisy trail points cause wiggle

### Symptom: Robot cuts corners

1. Decrease `pursuit_lookahead_max_m` (e.g., 2.5 → 1.5)
2. Decrease `pursuit_lookahead_time_s` (e.g., 0.8 → 0.5)
3. Increase trail density: decrease `trail_min_spacing_m` (0.3 → 0.2)
4. Check smoothing: `trail_smoothing_window=5` might over-smooth tight corners → try 3
5. Verify curvature scaling is enabled and `pursuit_curvature_alpha` ≥ 3.0

### Symptom: Robot stops unexpectedly during follow

1. Check `follow_me_tracking` in logs — did it go false?
2. If yes, check `follow_me_num_detections` — when did detections drop to 0?
3. How long was the detection dropout? Compare to `lost_target_timeout_s` (3.5s)
4. If dropout < 3.5s and robot still stopped: check `trail_length` — may have been 0 (no trail to follow)
5. If dropout > 8s: increase `lost_target_trail_pursuit_max_s`

### Symptom: Robot overshoots at turns (swings wide)

1. Robot going too fast into the turn: increase `pursuit_curvature_alpha` (e.g., 5 → 8)
2. Not enough pre-deceleration: increase `pursuit_lookahead_curvature_points` (5 → 8)
3. Check `pursuit_wheelbase_m` — if too small (< physical track width), steering corrections are weak. Try increasing from 0.28 to 0.35-0.40

### Symptom: Trail mode never activates

1. Check `trail_follow_enabled = True`
2. Check if person is far enough: `direct_pursuit_distance_m = 3.5` means person must be > 4.5m for trail mode (due to 1.3× hysteresis)
3. Check `trail_length` in telemetry — if always 0, trail points aren't being added
4. Check odometry: if `odom_x`, `odom_y` are stuck at 0, odometry isn't being updated (check `update_pose()` is being called)

### Symptom: Robot spins in place when target is lost

1. This is the "zero-speed spin" bug documented in `trail_follow_code_review.md`
2. Check `_MIN_LOST_TARGET_SPEED = 5` in follow_me.py — this should prevent zero forward speed
3. If `last_speed_offset` was 0 when target was lost, the minimum speed (5) kicks in
4. If robot is still spinning: check motor bytes in log — both should be > NEUTRAL (126) during forward motion

### Symptom: Odometry drifts badly (trail placed in wrong locations)

1. Verify speed_scale: re-run Phase 1 calibration
2. Check IMU heading: is it updating? (heading_deg should change with robot rotation)
3. Check for NaN in heading: `grep "NaN" logs/latest.log`
4. On dirt/grass: speed_scale calibrated on pavement may be 30-50% too high (wheel slip)
5. If all else fails: disable trail mode and use direct pursuit only — it doesn't need odometry

---

## 15. Log Analysis Guide

### Accessing logs

```bash
# On the Pi via SSH:

# Live tail with pretty-print:
tail -f logs/latest.log | python3 -m json.tool

# Filter to Follow Me fields only:
tail -f logs/latest.log | python3 -c "
import sys, json
for line in sys.stdin:
    try:
        d = json.loads(line)
        fm = {k:v for k,v in d.items() if 'follow' in k or 'trail' in k or 'odom' in k or 'pursuit' in k}
        if fm: print(json.dumps(fm))
    except: pass
"

# Post-run analysis:
python3 pi_app/tools/perf/follow_me_analyze.py --log logs/latest.log --follow-only --last-seconds 120
```

### Key fields to watch in real-time

| Field | What to look for | Problem indicator |
|-------|-----------------|-------------------|
| `follow_me_pursuit_mode` | "direct" vs "trail" | Stuck on one mode = threshold issue |
| `follow_me_steer_offset` | Should be smooth, ±15 max | Rapid sign changes = oscillation |
| `follow_me_speed_offset` | 0 when close, ramps up with distance | Always 0 = not detecting person |
| `trail_length` | Grows during walk, consumed behind | Always 0 = trail not populating |
| `trail_speed_limited` | True during curves | Always false = curvature scaling not working |
| `trail_curvature_at_lookahead` | ~0 straight, > 0 in turns | Always 0 = trail too short for curvature |
| `follow_me_tracking` | True when person in view | Frequent toggling = detection instability |
| `odom_x`, `odom_y` | Advances with robot motion | Stuck at 0 = odometry not updating |

### Post-run analysis checklist

After each test run, pull the log and check:

1. **Tracking uptime**: % of frames where `follow_me_tracking = true`
   - Target: > 80% during active follow
2. **Max detection dropout**: longest continuous span of `num_detections = 0`
   - Target: < 3s during line-of-sight follow, < 8s during occlusion
3. **Steering smoothness**: standard deviation of `steer_offset` during straight segments
   - Target: < 3.0 bytes
4. **Mode distribution**: % time in "direct" vs "trail"
   - Expected: mostly "direct" for close follow, "trail" when far or turning
5. **Speed limited events**: count of frames where `trail_speed_limited = true`
   - Should only occur during curves, not on straights

### Saving calibration results

After each tuning change, record what you changed and the result:

```
Date: ___________
Phase: ___________
Parameter changed: ___________
Old value: ___________
New value: ___________
Test performed: ___________
Result: PASS / FAIL / PARTIAL
Observations: ___________
```

Keep a running log of changes in a text file on the Pi (`~/tuning_log.txt`) or in this repo under `docs/tuning_sessions/`. This prevents re-trying combinations that already failed.

---

## Appendix A: Skid-Steer Specific Considerations

This robot is a rear-wheel-drive skid-steer with front casters. Several tuning considerations differ from typical Ackermann or symmetric differential-drive robots:

1. **Effective track width > physical track width**: Skid-steer turning involves lateral tire scrub, which makes the turning radius larger than pure differential-drive kinematic models predict. The `pursuit_wheelbase_m = 0.28` (physical track width) may need to be 0.30-0.40 for accurate steering commands. Increase this if the robot consistently under-steers (turns too wide).

2. **Asymmetric turning**: Rear-wheel drive with front casters means the rear axle is the pivot point. Left and right turns may not be symmetric due to caster swivel friction. If the robot turns more easily in one direction, this isn't a tuning issue — it's a hardware characteristic.

3. **Speed-dependent turning behavior**: At low speed, skid-steer turns have high friction (scrubbing rubber). At high speed, momentum helps. Pure pursuit steering commands may work well at one speed but not another. The existing speed-dependent gain scheduling in the IMU heading controller helps with this, but the Follow Me steering is NOT gain-scheduled. If high-speed turns overshoot, consider reducing `max_steer_offset_byte` for Follow Me.

4. **Surface dependency**: Skid-steer turning behavior changes dramatically between pavement, grass, and dirt. Parameters tuned on pavement may cause oscillation on grass (less friction → more overshoot) or sluggish turns on dirt (more friction → undershoot). If you'll operate on multiple surfaces, tune on the worst-case surface and accept slightly conservative behavior on others.

5. **Odometry accuracy**: Dead-reckoning on a skid-steer robot is inherently less accurate than on a differential-drive robot because wheel slip during turns is unmodeled. The IMU heading partially compensates (no yaw drift from slip), but forward speed estimation assumes no longitudinal slip. This is acceptable for short-lived trails (< 30s) but would be problematic for long-term mapping.

---

## Appendix B: Relevant Documentation

- `docs/imu_tuning_session_20260308.md` — IMU heading calibration, PID tuning, Follow Me steering gain issue
- `docs/trail_follow_code_review.md` — Code review findings and hardening roadmap
- `docs/trail_follow_pure_pursuit_plan.md` — Original architecture and implementation plan
- `docs/research_trail_following_strategies.md` — Research findings driving the upgrades
- `docs/testing.md` — General testing procedures and CLI tools
- `docs/performance_baseline_report_20260301.md` — Control loop and CPU performance baseline

---

## Appendix C: Emergency Procedures

**Robot won't stop:**
1. Flip the RC arm switch to DISARM (CH5)
2. If that fails, turn off the RC transmitter (failsafe should trigger stop)
3. If that fails, kill power at the robot (battery disconnect)

**Robot behaves erratically:**
1. DISARM immediately via RC
2. Do not re-arm until you've checked logs
3. Check `journalctl -u wall-e.service -n 100` for errors

**Robot heads toward obstacle/person:**
1. DISARM via RC
2. Physically push robot away from obstacle if needed
3. Review whether obstacle avoidance is enabled (`obstacle_avoidance.enabled = True`)
