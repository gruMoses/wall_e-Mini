# WALL-E Mini Deep Hardening Plan

## Scope

This plan addresses the highest-risk holes found in deep review:

1. Command/auth security gaps (Bluetooth control path)
2. Safety and mode-state integrity (Follow Me activation/deactivation)
3. Runtime fault tolerance and shutdown safety (fail-safe motor stop)
4. Telemetry/data contract correctness (JSON + MCAP + tooling consumers)
5. Numerical/config hardening
6. Test architecture and regression coverage

This is intentionally aggressive and detailed ("spare no expense") while still
sequenced to reduce operational risk.

---

## Goals and Non-Goals

### Goals

- No unauthenticated remote motion command path in production.
- Single source of truth for autonomy mode transitions.
- Deterministic stop behavior on disarm, crash, and shutdown.
- Consistent telemetry schema across producers and consumers.
- High-confidence replay/calibration pipelines from logs.
- CI catches regressions in safety-critical behavior.

### Non-Goals

- Full autonomy redesign (planner/mapping stack overhaul).
- Hardware redesign.
- Rewriting all legacy modules at once.

---

## Risk Ranking (What we fix first)

### Critical

- Unauthenticated BT command acceptance.
- Follow Me mode desync between controller `_mode` and safety `follow_me_active`.

### High

- Entering FOLLOW_ME without FollowMe controller available.
- Silent recorder exceptions and undefined variable ordering (`imu_status` usage).
- Missing explicit motor neutral/stop in global shutdown path.
- Timestamp precision loss (`ts` rounded to int) affecting analytics/replay.
- Calibration parser reading incorrect field locations.

### Medium

- Telemetry key mismatch (`follow_me_num_persons` vs `follow_me_num_detections`).
- Waypoint speed interpolation divide-by-zero on bad config.
- Lockfile unlink behavior in launcher.

---

## Workstream Structure

Parallel workstreams with strict sequencing gates:

- **WS-A Security** (BT protocol, ingress validation)
- **WS-B Safety/Control Integrity** (mode state, disarm guarantees)
- **WS-C Telemetry Contract** (JSON/MCAP schema + tooling)
- **WS-D Reliability/Operations** (shutdown, locking, watchdog behavior)
- **WS-E Verification** (tests, replay fixtures, CI gates)

Release gating: WS-A + WS-B + WS-D must pass before any field testing.

---

## Phase 0 - Baseline Freeze and Test Harness (Day 0)

### Deliverables

- Tag current branch state.
- Capture baseline logs and behavior on a short scripted run:
  - manual drive
  - arm/disarm
  - Follow Me enter/exit
  - BT command injection attempt (current behavior baseline)
- Add a "golden corpus" folder for logs used in parser regression tests.

### Files

- `pi_app/tests/fixtures/logs/` (new)
- `pi_app/tests/fixtures/mcap/` (new, small redacted samples)
- `docs/` update with baseline commit hash and run metadata

### Acceptance Criteria

- Baseline artifacts reproducible in CI/local.
- Known failing behavior documented and linked to fixes below.

---

## Phase 1 - Security Hardening (WS-A, Critical)

## 1.1 Enforce authentication for CMD2

### Current hole

`pi_app/io/bt_proto.py` accepts any well-formed CMD2 packet.

### Implementation Tasks

1. Add cryptographic verification utility:
   - HMAC-SHA256 over canonical payload: `<left_i>;<right_i>;<seq>;<ts_ms>;<sn_hex>`
   - Constant-time comparison.
2. Add anti-replay checks:
   - strict `seq` monotonicity per client session
   - timestamp freshness window (for example: +/- 5 s)
   - nonce/session binding validation
3. Add config-backed secret management:
   - load secret from env/file
   - fail closed in production mode when missing
4. Add protocol-level error codes:
   - `bad_hmac`, `stale_ts`, `bad_nonce`, `old_seq`, `auth_required`
5. Keep optional dev mode only if explicitly enabled in config (default off).

### Files

- `pi_app/io/bt_proto.py`
- `pi_app/cli/spp_server.py`
- `config.py` (BT auth settings)
- `docs/` protocol spec update

### Tests

- valid HMAC accepted
- invalid HMAC rejected
- stale timestamp rejected
- old sequence rejected
- nonce mismatch rejected
- dev mode override behavior

### Acceptance Criteria

- No command accepted without valid auth in default config.
- Backward compatibility documented and explicitly opt-in only.

---

## Phase 2 - Safety and Mode-State Integrity (WS-B, Critical/High)

## 2.1 Single-source Follow Me activation state

### Current hole

Web activation sets `_mode` directly, bypassing safety `follow_me_active`.

### Implementation Tasks

1. Introduce explicit controller APIs that route through safety state transitions:
   - `request_follow_me_activate(source=web|gesture|rc)`
   - `request_follow_me_deactivate(source=...)`
2. Refactor event processing so `_mode` is derived from validated state/events.
3. On disarm:
   - always force `_mode = MANUAL`
   - clear follow-me active state
   - clear gesture state as needed
4. Ensure transitions are idempotent and logged once.

### Files

- `pi_app/control/controller.py`
- `pi_app/control/safety.py`
- `pi_app/control/gesture_control.py` (if transition hooks needed)

### Tests

- web activate + disarm always exits Follow Me
- disarm/rearm does not auto-resume Follow Me
- rc + web + gesture transitions do not diverge

### Acceptance Criteria

- No stale FOLLOW_ME mode after any disarm path.

## 2.2 Guard FOLLOW_ME entry when FollowMe controller unavailable

### Implementation Tasks

1. On `FOLLOW_ME_ENTERED` event:
   - if `self._follow_me is None`, reject transition and emit telemetry flag
2. Add user-visible reason code (`follow_me_unavailable_reason`).

### Acceptance Criteria

- Mode/telemetry never report FOLLOW_ME when unavailable.

---

## Phase 3 - Runtime Safety and Reliability (WS-D, High)

## 3.1 Guaranteed neutral/stop in shutdown and fatal exception paths

### Implementation Tasks

1. In `main.run()` global `finally`, call explicit motor stop/neutral write before teardown.
2. Wrap `controller.process(...)` with fail-safe:
   - on exception: emergency neutral command + disarm relay + bounded retry policy
3. Add one-time error reporting (no silent swallow in critical paths).

### Files

- `pi_app/app/main.py`
- `pi_app/control/controller.py` (optional helper)

### Tests

- simulated exception in process loop causes neutral output and safe disarm
- shutdown path always writes neutral at least once

### Acceptance Criteria

- Verified stop command emitted on all controlled exits and injected failures.

## 3.2 Remove lockfile unlink race from launcher

### Implementation Tasks

1. Remove lockfile deletion from `run_main.sh`.
2. Rely on `flock` in `main.py`.
3. If stale lock diagnostics needed, inspect PID while lock held (do not unlink path pre-lock).

### Files

- `run_main.sh`
- optional helper script for diagnostics

### Acceptance Criteria

- No duplicate control process can be launched via stale-file race.

---

## Phase 4 - Telemetry Contract Unification (WS-C, High/Medium)

## 4.1 Define explicit schema v2 for runtime logs

### Implementation Tasks

1. Create schema doc:
   - `docs/telemetry_schema_v2.md`
2. Standardize field names:
   - use `follow_me_num_detections` everywhere
   - remove/alias deprecated names with sunset date
3. Precision policy:
   - `ts` must remain float epoch seconds (no int quantization)
   - `ts_iso` retained
   - lat/lon precision preserved (no `round1` on GPS coordinates)

### Files

- `pi_app/app/main.py`
- `pi_app/hardware/oak_recorder.py`
- `docs/telemetry_schema_v2.md`

### Acceptance Criteria

- Producers and consumers agree on identical key paths.
- No precision regression in timestamps or GPS.

## 4.2 Fix producer/consumer mismatches

### Implementation Tasks

1. Move `imu_status` fetch ahead of `RecordingTelemetry(...)`.
2. Include explicit follow tracking/target fields in MCAP payload:
   - `follow_tracking`
   - `follow_target_x_m`, `follow_target_z_m`
   - `follow_target_confidence` if available
3. Ensure JSON and MCAP carry consistent core fields for replay/calibration.
4. Replace broad `except: pass` around recorder/logging with narrowed handling plus sampled warnings.

### Files

- `pi_app/app/main.py`
- `pi_app/hardware/oak_recorder.py`

### Acceptance Criteria

- No undefined-var recorder exceptions.
- Replay no longer needs heading backfill for new recordings.

## 4.3 Fix downstream tools

### Implementation Tasks

1. `follow_me_calibration.py` parse from `follow_me` object, not top-level.
2. `trail_replay.py` use schema-v2 fields first, then legacy fallback.
3. Add parser version detection and compatibility warnings.

### Files

- `pi_app/cli/follow_me_calibration.py`
- `pi_app/cli/trail_replay.py`

### Acceptance Criteria

- Calibration and replay outputs match expected values on fixture logs.

---

## Phase 5 - Config and Numerical Hardening (WS-B/WS-C, Medium)

## 5.1 Guard invalid config relationships

### Tasks

1. Validate waypoint config:
   - enforce `slow_radius_m > arrival_radius_m`
2. Validate obstacle config:
   - enforce `slow_distance_m > stop_distance_m`
3. Validate follow me tuning:
   - non-negative gains/limits where required
4. Fail fast on startup with explicit error messages.

### Files

- `pi_app/control/waypoint_nav.py`
- relevant config validation layer (new if absent)

### Acceptance Criteria

- Invalid configs produce deterministic startup errors, not runtime crashes.

---

## Phase 6 - Test Expansion and Quality Gates (WS-E)

## 6.1 Safety-critical integration tests

Add tests for:

- mode transitions across RC/web/gesture sources
- disarm forcing MANUAL from all autonomy modes
- Follow Me unavailable guard behavior
- emergency path and motor neutral writes

### Files

- `pi_app/tests/test_controller.py` (or split by concern)
- new `pi_app/tests/test_safety_integration.py`

## 6.2 Telemetry contract tests

Add tests for:

- JSON log schema v2 required fields
- MCAP telemetry required fields
- producer/consumer compatibility (round-trip tests)

### Files

- `pi_app/tests/test_telemetry_schema.py`
- `pi_app/tests/test_replay_parsers.py`

## 6.3 Security tests

Add tests for:

- BT HMAC happy path and rejection matrix
- replay resistance (seq/timestamp)

### Files

- `pi_app/tests/test_bt_proto_auth.py`
- `pi_app/tests/test_spp_server_auth.py`

## 6.4 CI enforcement

1. Add branch coverage thresholds for:
   - `pi_app/control/controller.py`
   - `pi_app/control/safety.py`
   - `pi_app/io/bt_proto.py`
   - telemetry parsers
2. Add changed-lines coverage gate.

### Acceptance Criteria

- Safety/security test suites required to pass before merge.

---

## Phase 7 - Rollout and Operational Validation

## 7.1 Staged rollout

1. Lab bench:
   - command injection tests
   - disarm/emergency tests
2. Indoor low-speed:
   - Follow Me transitions and lost-target behavior
3. Outdoor supervised:
   - replay and calibration data quality validation

## 7.2 Observability and incident hooks

1. Add explicit log lines for rejected auth attempts and mode transition reasons.
2. Add counters:
   - `bt_auth_reject_count`
   - `follow_me_transition_reject_count`
   - `loop_exception_count`

### Acceptance Criteria

- No unexplained mode transitions.
- No command acceptance without auth.

---

## Implementation Sequence (Suggested)

1. Phase 1 (Security)
2. Phase 2 (Mode integrity)
3. Phase 3 (Fail-safe shutdown/runtime)
4. Phase 4.1/4.2 (Telemetry producer correctness)
5. Phase 4.3 (Tooling consumers)
6. Phase 5 (Config guards)
7. Phase 6 (Coverage and CI gates)
8. Phase 7 (Field rollout)

This sequence minimizes time spent with known critical exposures.

---

## Detailed Task Checklist

## A. Security

- [ ] Add HMAC verification to CMD2
- [ ] Add nonce/timestamp freshness checks
- [ ] Add strict replay prevention
- [ ] Add config-driven secrets and fail-closed defaults
- [ ] Add auth rejection telemetry/logging
- [ ] Add protocol doc update

## B. Safety/Mode

- [ ] Unify Follow Me state authority
- [ ] Ensure disarm always forces MANUAL
- [ ] Block FOLLOW_ME when controller unavailable
- [ ] Add explicit transition reason codes

## C. Runtime Reliability

- [ ] Add guaranteed neutral in main `finally`
- [ ] Add fail-safe wrapper around `controller.process`
- [ ] Replace blanket silent exceptions in critical loop sections
- [ ] Remove lockfile unlink race

## D. Telemetry Contract

- [ ] Write telemetry schema v2 doc
- [ ] Stop integer-quantizing `ts`
- [ ] Preserve GPS precision
- [ ] Standardize detection count key
- [ ] Include follow tracking/target in MCAP
- [ ] Fix `imu_status` ordering bug

## E. Tooling

- [ ] Fix calibration parser key paths
- [ ] Add parser compatibility layer for legacy logs
- [ ] Add replay/parser regression fixtures

## F. Tests/CI

- [ ] Add safety integration tests
- [ ] Add BT auth tests
- [ ] Add telemetry schema tests
- [ ] Enforce coverage thresholds for critical modules

---

## Rollback Plan

For each merged phase:

1. Keep feature flags for new auth strictness and schema consumers.
2. If field issue detected:
   - roll back consumer strictness first, keep producers additive.
3. If control/safety issue detected:
   - immediate fallback to MANUAL-only mode + disable Follow Me web activation.

---

## Definition of Done

All of the below must be true:

- No unauthenticated BT motion command path.
- Follow Me mode transitions are deterministic and safety-state aligned.
- Disarm/emergency always produces neutral command and MANUAL mode.
- JSON/MCAP telemetry matches schema-v2 and parsers consume it correctly.
- Calibration/replay tools pass fixture-based regression tests.
- CI blocks merge on safety/security/telemetry regressions.

---

## Optional Stretch Improvements

- Formal state machine library for mode transitions.
- Signed telemetry snapshots for forensic integrity.
- Runtime self-check endpoint exposing safety/health assertions.
- On-device "safe mode" boot profile for degraded operation.

