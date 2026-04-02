# Follow-Me Fix Plan
*Generated 2026-04-01 — based on arm_20260401_190138.log + live CAN diagnostics*

---

## Executive Summary

Six real issues identified from full code audit + live Pi diagnostics + log data.
Two user-reported "issues" are actually logging artifacts, not motor-control bugs.
Priority ordering below leads with correctness before observability.

---

## Confirmed Issues (with Root Cause)

### Issue 1 — CRITICAL: Steer forced to 0 during detection persistence window
**Status:** Confirmed from log data
**Evidence:**
```
t=23.1s  track=True  det=0  z=5.8  x=-4.7  str=0.0  motor_L=206  motor_R=206
t=24.8s  track=True  det=0  z=5.8  x=-4.7  str=0.0  motor_L=206  motor_R=206
```
The target was 39° off-axis (x=−4.7 m at z=5.8 m) for 1.7 seconds of persistence, but steer_offset was **exactly 0.0** the whole time, so both motors ran equal (206, 206) — the robot drove straight while the person was hard left.

**Root Cause:**
`follow_me.py:684–685`:
```python
if not fresh_detection:
    steer = 0.0   # ← kills ALL steering the moment vision drops one frame
```
The comment says "coasts straight instead of holding the last turn command and overshooting into empty space." But the opposite is worse: at 39° off-axis the robot needs to turn to keep the target in FOV, not drive straight into empty space.

**Proposed Fix:**
Replace the hard zero with a hold-then-decay of the last valid steer:
```python
# In FollowMeController.__init__:
self._last_fresh_steer: float = 0.0      # steer from last frame with fresh detection
self._last_fresh_steer_time: float = 0.0 # monotonic() of that frame

# In compute(), replace the steer=0.0 block:
STEER_HOLD_DECAY_S = 1.5  # seconds to decay from last steer to 0
if not fresh_detection:
    elapsed_since_fresh = now - self._last_fresh_steer_time
    decay = max(0.0, 1.0 - elapsed_since_fresh / STEER_HOLD_DECAY_S)
    steer = self._last_fresh_steer * decay
else:
    # ... compute steer normally ...
    self._last_fresh_steer = steer
    self._last_fresh_steer_time = now
```
This keeps the last steering command alive and fades it to zero over ~1.5 s, giving the robot time to continue turning around a corner during a brief occlusion.

**Add to FollowMeConfig:**
```python
steer_hold_decay_s: float = 1.5   # how long to hold steer during detection dropout
```

**Expected behavior after fix:** Robot maintains turn toward person during brief detection gaps (corner turns, tall grass). Steer fades gracefully to neutral only on prolonged loss.

---

### Issue 2 — HIGH: VESC telemetry not included in JSON log
**Status:** Confirmed from log inspection
**Evidence:** The log_obj in `main.py` never writes `vesc_left_rpm`, `vesc_right_rpm`, or `vesc_actual_speed_mps`. The controller adds them to the `telem` dict but they are never serialized.
Live CAN diagnostics confirm CAN IS working:
```
candump:  00000901  [8] 00 00 00 00 00 00 00 00   ← STATUS from VESC id=1 (RPM=0, stopped)
          00000902  [8] 00 00 00 00 00 00 00 00   ← STATUS from VESC id=2
          00001B01  [8] 00 02 4D 7E 02 26 00 00   ← STATUS5 from VESC id=1: voltage = 0x0226/10 = 55.4V
```
Frame IDs decode correctly against the `vesc.py` parser (`packet_id = (arb_id >> 8) & 0xFF`, `vesc_id = arb_id & 0xFF`). RX parsing logic is correct.

**Root Cause:**
`main.py` `log_obj` missing a `"vesc"` section. The velocity PID in `SpeedLayer.compute()` uses `actual_speed_mps` from VESC, but this value (and whether it was None) is invisible in the logs. This makes diagnosing closed-loop behavior impossible post-hoc.

**Proposed Fix:**
Add a `"vesc"` section to `log_obj` in `main.py`:
```python
"vesc": {
    "left_rpm":   telem.get("vesc_left_rpm"),
    "right_rpm":  telem.get("vesc_right_rpm"),
    "speed_mps":  round(telem.get("vesc_actual_speed_mps"), 3)
                  if telem.get("vesc_actual_speed_mps") is not None else None,
},
```
Also add `"actual_speed_mps"` to the `"follow_me"` log section:
```python
"actual_speed_mps": telem.get("follow_me_actual_speed_mps"),
```

**Expected behavior after fix:** Log analysis can confirm whether velocity PID was active, diagnose slip events, and verify RPM feedback is alive.

---

### Issue 3 — HIGH: BMS discharge FET off for first 14.1 seconds
**Status:** Confirmed from log data
**Evidence:**
```
BMS FET state over FM session:
  t=0.0s  discharge_fet_on=False  charge_fet_on=False
  t=14.1s discharge_fet_on=True   charge_fet_on=False
```
During the first 14.1 s of Follow Me, the BMS reported `discharge_fet_on=False`. Motor commands were being issued (small, near-neutral during close range), but the pack was potentially not delivering current.

**Root Cause:**
The BMS requires a successful BLE poll to report FET state. On first startup, `BmsState.discharge_fet_on=None` (default). The service connects and makes its first poll ~14 s after arm. In the meantime, `is_charging()` returns False (correct), so `charger_inhibit=False` and motor commands flow through. But `discharge_fet_on` being False/None means the pack may not supply current regardless.

There is no existing safety check for `discharge_fet_on`. The code only gates on `is_charging()` (charge FET + positive current = charging).

**Proposed Fix:**

Option A (safest — Kevin's call): **Block Follow Me mode until BMS confirms discharge FET on.**
In `controller.py`, add a check before entering FOLLOW_ME:
```python
if bms_state is not None and bms_state.discharge_fet_on is False:
    # BMS actively reporting discharge FET off — don't drive
    _logger.warning("BMS discharge FET OFF — Follow Me blocked")
    left = right = CENTER_OUTPUT_VALUE
    self._motor.stop()
```

Option B (lighter): **Log a prominent warning when motors commanded but discharge_fet_on=False.**
Acceptable if this startup pattern is considered normal BMS behavior (FET comes on after first poll confirms no fault condition). Log only, no block.

**Design question for Kevin:**
Is the 14-second window normal for this BMS? If the BMS always takes ~14 s to confirm FET state after arm, Option B is appropriate and Option A would prevent driving for 14 s after every arm. If FET=False indicates a real fault, Option A is required.

---

### Issue 4 — MEDIUM: IMU pid_correction logged in FM mode but not applied (log confusion)
**Status:** Confirmed artifact, not a real motor interference
**Evidence:**
```
t=24.8s  imu.pid_correction=11.13   imu_steering.correction_raw=None  motor_L=206  motor_R=206
```
Both motors stayed equal (206, 206) while `imu.pid_correction` jumped to 11.13 bytes. This confirms the PID computes non-zero corrections in FM mode but they are never sent to motors (`correction_raw=None`).

The IMU "fights FM" observation is a **false positive** in log analysis — `imu.pid_correction` is always computed and logged regardless of mode, but it is never applied when `self._mode == "FOLLOW_ME"` (confirmed at `controller.py:490–495`).

**Root Cause (two parts):**
1. In `controller.py:445`, `self._imu_compensator.update(0.0, dt)` is called in FM mode to get heading for dead-reckoning. This runs the full PID (including `_compute_heading_hold_correction()`) and updates `self._last_pid` with non-zero values.
2. In `controller.py:500–508`, `get_status()` is called unconditionally (all modes), logging those `_last_pid` values as `pid.correction`, `pid.error_deg`, etc.

The actual motor path is correct: `imu_correction=None` in FM mode → correction block at `controller.py:571–600` skipped.

**Proposed Fix:**
In `controller.py`, in the FOLLOW_ME block, call a heading-only method instead of the full `update()`:
```python
# Instead of: self._imu_compensator.update(0.0, max(dt, 0.001))
heading = self._imu_compensator.get_heading_deg()  # read heading, no PID
```
Add to `ImuSteeringCompensator`:
```python
def get_heading_deg(self) -> float:
    """Read IMU and return current heading without running PID. For FM mode."""
    data = self.imu_reader.read()
    with self.lock:
        self.state.heading_deg = data['heading_deg']
        self.state.yaw_rate_dps = data['gz_dps']
        self.state.last_update_time = time.monotonic()
    return self.state.heading_deg
```
This eliminates the spurious PID state being logged in FM mode, while still providing accurate heading for dead-reckoning odometry.

Also: clear `_last_pid` to zeros when entering FM mode so the logged `pid.*` fields don't show stale FM-mode computations.

**Expected behavior after fix:** `pid.correction` and `pid.error_deg` will be 0 throughout FM mode, eliminating the false "IMU fights FM" signal.

---

### Issue 5 — MEDIUM: Depth valid% very low (8–14%)
**Status:** Confirmed from log data
**Evidence:**
```
depth_valid_pct:          11.6%
depth_quality_reject_count: 1934 / 4331 frames = 44.7% rejected
```
Only ~12% of pixels in the obstacle ROI have valid stereo depth. This means obstacle avoidance is working from very sparse data.

**Root Cause (likely):**
Outdoor grass/open terrain has poor stereo texture for the OAK-D Lite disparity algorithm. Additionally:
- `roi_height_pct: 0.5` with `roi_vertical_offset_pct: -0.20` shifts the ROI upward — the upper portion of the ROI may see sky or distant terrain with poor stereo matching.
- `min_depth_mm: 600` rejects all readings below 0.6 m (appropriate for safety but removes a lot of valid near-field depth).

**Proposed Fixes:**
1. **Lower ROI vertical center** — change `roi_vertical_offset_pct` from `-0.20` to `0.0` or even `+0.10` so the ROI looks at the ground plane rather than horizon/sky.
2. **Enable subpixel disparity** in OAK-D pipeline config (if not already) — improves depth on low-texture surfaces.
3. **Reduce `roi_height_pct`** slightly (e.g. 0.35) to concentrate on forward ground rather than a wide vertical band.

**Design question for Kevin:**
The ROI is intentionally shifted up (`-0.20`) — was this to avoid ground-plane false positives at close range? If so, the tradeoff is losing a lot of valid depth data. Worth A/B testing with `0.0` offset on the next run.

---

### Issue 6 — LOW: `steer_offset` telemetry not updated in lost-target path
**Status:** Confirmed from log data
**Evidence:**
After detection dropout at t=24.9s, motor_L dropped from 206 → 155 while motor_R held ~194–203 (the trail/search steer is working), but `follow_me.steer_offset` remained 0.0 throughout because `_last_steer_offset` is only set in the target-present path.

**Root Cause:**
In `follow_me.py`, `_handle_lost_target()` calls `_mix_commands(fwd, steer)` and returns (left, right) but does NOT update `self._last_steer_offset`. The `get_status()` telemetry therefore always shows 0.0 steer during lost-target operation, even when trail pursuit is actively steering.

**Proposed Fix:**
In `_handle_lost_target()`, after computing the trail/search steer, update `_last_steer_offset`:
```python
# At the trail-pursuit return:
self._last_steer_offset = steer  # or cmd.steer_byte
self._last_speed_offset = fwd
return _mix_commands(fwd, steer)

# At the search-rotation return:
self._last_steer_offset = steer
return _mix_commands(fwd, steer)
```
This makes the log show the actual trail steer being applied, not 0.

---

## Non-Issues (Previously Reported — Explained)

### "bt_L and bt_R are null = no RPM feedback"
**Finding:** False correlation. `bt.L` and `bt.R` in the log are the Bluetooth joystick input bytes. They are `null` throughout Follow Me because the robot is under autonomous control — there is no BT joystick active. This has no relationship to VESC telemetry.

Live CAN diagnostics confirm VESCs are broadcasting STATUS frames at ~60 Hz:
```
00000901  [8] 00 00 00 00 00 00 00 00   ← VESC 1 RPM=0, current=0, duty=0
00000902  [8] 00 00 00 00 00 00 00 00   ← VESC 2 RPM=0
00001B01  [8] 00 02 4D 7E 02 26 00 00   ← VESC 1 voltage=55.4V
```
The telemetry RX thread in `vesc.py` IS receiving and parsing these correctly. The reason VESC RPMs appear "null" in the log analysis is **Issue 2 above** — they are never written to the JSON log file.

### "IMU heading PID fights follow-me steering (±7–11° corrections)"
**Finding:** Log artifact. In FM mode, `imu_compensator.update(0.0, dt)` runs the full PID to extract heading for dead-reckoning, and the computed correction is stored in `_last_pid`. `get_status()` is called unconditionally and logs these values as `imu.pid_correction`. The value of 11.13 bytes at t=24.8s looks alarming but the motor bytes were exactly equal (206, 206) at that moment — the correction was **never applied**. Confirmed by `imu_steering.correction_raw=null` throughout the entire FM session.

---

## Priority Ordering

| Priority | Issue | File(s) | Complexity |
|----------|-------|---------|------------|
| 1 | Steer=0 during persistence dropout | `follow_me.py` | Medium |
| 2 | VESC RPM not logged | `main.py` | Trivial |
| 3 | BMS discharge FET startup delay | `controller.py`, `main.py` | Low |
| 4 | IMU PID logged in FM mode | `imu_steering.py`, `controller.py` | Low |
| 5 | Depth valid% ROI tuning | `config.py` | Config change |
| 6 | `steer_offset` missing in lost-target path | `follow_me.py` | Trivial |

---

## Outstanding Design Questions for Kevin

1. **Steer hold decay rate:** Should the held steer decay over 1.5 s (proposed) or should it hold at full value for the persistence window (2.0 s) then instantly zero? Full hold risks overshoot if the person was detected at edge of frame; gradual decay is safer.

2. **BMS startup block:** Is the 14-second discharge_FET-off window always normal for this BMS? If yes, log a warning only (Option B). If it can indicate a real fault, add a block (Option A).

3. **IMU mode in Follow Me:** The `imu_compensator.update()` call in FM mode serves double duty: reads IMU for odometry heading AND runs (discarded) PID. Should we add a `get_heading_deg()` method that reads IMU without touching PID state? This cleanly separates the two concerns and eliminates log noise. The only downside is a small refactor to `imu_steering.py`.

4. **Depth ROI offset:** The `-0.20` vertical offset was presumably set to avoid false ground-plane stops at close range. Changing it to `0.0` may cause the robot to stop when looking at its own feet (if OAK is mounted low). What is the camera mount height and forward tilt angle? This determines the safe ROI window.

5. **Velocity PID at max speed:** When `SpeedLayer` saturates to `max_follow_speed_byte=80`, the velocity PID is still active and computing corrections. At 5.8m with RPM=0 (motors just commanded), the PID sees `velocity_error = target_speed - 0 = large positive` and may integrate aggressively. The `speed_integral_limit=50.0` is in m/s accumulated, which is very large. Should the velocity PID be disabled when already at `max_follow_speed_byte`? This prevents integrator windup that will cause speed overshoot once the robot does reach the target.

---

## CAN Bus Health Summary (Live, 2026-04-01)

```
Interface:     can0  UP  LOWER_UP  (MCP2515 on SPI0, kernel driver: mcp251x)
python-can:    4.6.1
Kernel mods:   can_raw, can, can_dev, mcp251x

VESC 1 (right, id=1):  Voltage=55.4V  FET temp=27.9°C  RPM=0 (stopped)
VESC 2 (left,  id=2):  Voltage=53.9V  FET temp=28.5°C  RPM=0 (stopped)
TX (Pi→VESC):  00000301 [4] 00 00 00 00  (SET_RPM id=1, 0 RPM)
               00000302 [4] 00 00 00 00  (SET_RPM id=2, 0 RPM)
Status:        All STATUS (9), STATUS_4 (16), STATUS_5 (27) frames arriving at ~60 Hz
```
**CAN is healthy. VESC telemetry is working. The "telemetry dead" diagnosis was a logging gap.**

---

## Files Changed by This Plan

| File | Change |
|------|--------|
| `pi_app/control/follow_me.py` | Issues 1, 6: Steer hold/decay during dropout; update `_last_steer_offset` in lost-target path |
| `pi_app/app/main.py` | Issue 2: Add `vesc` section to `log_obj`; add `actual_speed_mps` to `follow_me` section |
| `pi_app/control/controller.py` | Issues 3, 4: BMS discharge check; use `get_heading_deg()` in FM mode |
| `pi_app/control/imu_steering.py` | Issue 4: Add `get_heading_deg()` method |
| `config.py` | Issues 1, 5: Add `steer_hold_decay_s`; tune `roi_vertical_offset_pct` |

---

## Grok Safety Review — Addressed (2026-04-01, commit 34bb4a0)

All five Grok review items implemented in follow-up commit on master:

| # | Item | Resolution |
|---|------|-----------|
| 1 | BMS FET warning spams every tick | Rate-limited to once/5s; `BmsConfig.bms_fet_grace_period_s=20` + `bms_fet_safety_timeout_s=2` added. After grace period, FET-off >2s triggers `charger_inhibit` → force neutral. Arm/disarm events reset all tracking. |
| 2 | Steer hold decay fixed at 1.5s | Default lowered to 1.0s; made speed-aware: `effective_decay = decay_s * max(0.3, 1.0 - speed_factor)`. At max speed → 0.3s; at zero speed → 1.0s. |
| 2b | No turn-rate clamp during blind decay | Held steer clamped to 70% of `max_steer_offset_byte` during decay window. |
| 3 | No decay/detection observability in logs | `steer_decay_factor`, `fresh_detection`, `steer_hold_active` added to `get_status()` and `log_obj.follow_me`. |
| 4 | `get_heading_deg()` may skip roll/pitch/error state | Now updates all state fields (roll, pitch, error_count reset on success) so AHRS read cadence is preserved identically to `update()`. |
