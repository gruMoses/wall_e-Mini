# Trail Follow / Pure Pursuit - Consolidated Review and Hardening Plan

**Branch:** `feature/trail-follow-pure-pursuit`  
**Date:** 2026-03-10  
**Updated:** 2026-03-10 (consolidated findings + execution roadmap)

---

## Context

- Steer output rate limiting was intentionally removed after field testing; do not reintroduce it.
- Lost-target behavior moved from direct blind driving to trail-aware recovery.
- Differential torque concerns still matter, but should be handled through safer mode logic and speed policies, not steer slew.

---

## Priority Findings

## Critical

1. **Unauthenticated BT command acceptance** (`pi_app/io/bt_proto.py`): accepts any well-formed command; no HMAC/nonce verification.
2. **Follow Me mode-state desync risk** (`pi_app/control/controller.py`, `pi_app/control/safety.py`): web activation sets `_mode` directly without safety-state authority.

## High

3. **`imu_status` used before assignment** in recorder telemetry path (`pi_app/app/main.py`).
4. **Lost-target zero-speed spin case** in `_handle_lost_target()` (`pi_app/control/follow_me.py`).
5. **No odometry `dt` clamp** causing pose teleport after stalls (`pi_app/control/odometry.py`).
6. **No guaranteed motor neutral/stop on all shutdown/fault paths** (`pi_app/app/main.py`).
7. **Timestamp precision loss (`ts` int rounding)** harms replay/calibration (`pi_app/app/main.py`).
8. **Calibration parser reads wrong key paths** (`pi_app/cli/follow_me_calibration.py`).
9. **No E2E/lost-target integration tests** for safety-critical trail-follow behavior.

## Medium

10. Spin-in-place overestimates forward speed in odometry.
11. No NaN/inf heading guard in odometry.
12. No hysteresis for direct/trail mode switching.
13. Pure pursuit closest-point search can snap backward on crossing trails.
14. Replay uses first detection only; no multi-person replay fidelity.
15. Telemetry key mismatch around detection count naming.
16. Waypoint speed interpolation can divide by zero on bad config.
17. Lockfile unlink behavior in launcher is race-prone.

## Low

- Cleanup items: dead config, unused `speed_hint`, naming/duplication nits, performance nits in replay merge.

---

## Claim Validation Status (Deep Pass)

Validated against current code as of this update:

### Confirmed bugs / defects (direct code evidence)

- Unauthenticated BT command acceptance (`pi_app/io/bt_proto.py`).
- `imu_status` used before assignment in recorder telemetry build (`pi_app/app/main.py`).
- Lost-target zero-speed trail-pursuit can command spin-in-place (`pi_app/control/follow_me.py`).
- Odometry lacks `dt` clamp and finite heading guards (`pi_app/control/odometry.py`).
- Timestamp precision loss from `ts` integer rounding (`pi_app/app/main.py`).
- Calibration parser reads follow-me fields from wrong object path (`pi_app/cli/follow_me_calibration.py`).
- Detection-count telemetry key mismatch (`follow_me_num_persons` writer vs `follow_me_num_detections` readers).
- Waypoint interpolation divide-by-zero risk when radii are equal (`pi_app/control/waypoint_nav.py`).

### Confirmed coverage gaps

- No tests hit lost-target recovery paths (`_handle_lost_target`) in Follow Me.
- No tests cover `update_pose()`/trail-mode integration in controller-level behavior.
- No replay-tool test coverage exists for parser/compatibility behavior.

### Confirmed architectural/safety risks (real, but require policy decision)

- Follow Me mode authority split (`_mode` direct writes vs safety state); risk accepted only if explicitly intended.
- No explicit final motor neutral command in global `finally` path; behavior depends on lower-level stop semantics.
- Lockfile removal in launcher can undermine strict singleton guarantees under race conditions.

### Plausible optimizations / scenario-dependent recommendations

- Pure pursuit backward-snap on crossing trails (depends on path geometry).
- Direct/trail mode hysteresis to reduce mode flapping (depends on field behavior).
- Replay multi-detection fidelity and closed-loop replay mode (analysis-quality improvements).

---

## What Is Verified Good

- Camera-to-world transform sign conventions.
- Pure pursuit curvature sign chain and directionality.
- Trail ordering/min-spacing/prune behavior.
- Existing module-level unit tests passing.

---

## Execution Roadmap (Detailed)

## Phase 0: Baseline and Guardrails

- Freeze baseline run artifacts and collect short reproducible traces.
- Add fixture corpus for parser/replay regression.
- Define success metrics for each phase.

## Phase 1: Security Hardening (Must-do First)

1. Enforce CMD2 authentication:
   - HMAC-SHA256 verification
   - nonce binding + timestamp freshness
   - strict sequence replay protection
2. Introduce fail-closed default mode; dev bypass must be explicit.
3. Add auth-reject telemetry counters.

**Acceptance:** unauthenticated motion command path closed by default.

## Phase 2: Safety and Mode Integrity

1. Make safety the single source of truth for Follow Me state transitions.
2. Guard `FOLLOW_ME_ENTERED` when Follow Me controller unavailable.
3. Force MANUAL on every disarm path (all modes).
4. Add explicit transition reason telemetry.

**Acceptance:** no stale FOLLOW_ME mode after disarm/rearm cycles.

## Phase 3: Runtime Fail-safe Reliability

1. Guarantee motor neutral/stop in global shutdown/finally.
2. Wrap control-loop `process()` with fail-safe neutral/disarm behavior.
3. Replace broad silent catches in critical paths with bounded reporting.

**Acceptance:** injected exceptions still produce deterministic neutral output.

## Phase 4: Telemetry Contract Unification

1. Publish telemetry schema v2 in docs.
2. Fix producer issues:
   - move `imu_status` assignment before recorder telemetry construction
   - standardize detection-count key naming
   - preserve timestamp/GPS precision
3. Ensure MCAP/JSON parity for follow-tracking/targets and odometry fields.
4. Fix consumers:
   - calibration parser key paths
   - replay parser compatibility and version handling

**Acceptance:** replay and calibration consume the same schema without ad hoc fallback.

## Phase 5: Control Robustness and Numerical Hardening

1. Lost-target minimum forward speed in trail pursuit.
2. Odometry guards:
   - `dt` clamp / discard on extreme gaps
   - NaN/inf heading reject
   - spin-in-place forward-speed correction
3. Add mode-switch hysteresis (direct/trail decision only).
4. Add waypoint config validation for denominator safety.

**Acceptance:** no pose teleport, no NaN poisoning, no threshold chatter.

## Phase 6: Test and CI Expansion

1. Safety-critical integration tests:
   - detection -> trail -> loss -> trail recovery
   - disarm/emergency mode exits
2. Telemetry schema contract tests (JSON + MCAP producers and parsers).
3. BT auth rejection matrix tests.
4. Coverage gates for critical modules and changed lines.

**Acceptance:** CI blocks regressions on safety/security/schema behavior.

## Phase 7: Staged Rollout

1. Bench safety tests (auth rejects, disarm neutral guarantees).
2. Indoor low-speed autonomy tests.
3. Outdoor supervised validation with logs and replay calibration checks.

**Acceptance:** no unexpected mode transitions, safe recovery behavior confirmed.

---

## Implementation Checklist

- [ ] BT auth verification and replay protection
- [ ] Follow Me mode-state unification
- [ ] Shutdown fail-safe neutral command
- [ ] Recorder telemetry ordering fix (`imu_status`)
- [ ] Timestamp/GPS precision fixes
- [ ] Calibration/replay parser alignment
- [ ] Lost-target minimum forward speed
- [ ] Odometry dt/NaN/spin guards
- [ ] Direct/trail hysteresis
- [ ] Waypoint config validation
- [ ] Safety/E2E integration tests
- [ ] Telemetry contract tests
- [ ] CI coverage gates

---

## Notes

- This document is the single source for suggestions and remediation planning for this branch.
- Keep new findings appended here under a dated "Addendum" section to avoid suggestion-file sprawl.
