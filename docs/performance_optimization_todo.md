# WALL-E Mini Performance Optimization TODO

Last updated: 2026-03-01
Scope: Reduce Pi CPU load and maximize OAK-D offload while preserving safety/control behavior.

## Status Snapshot

- [x] Research complete (code audit + OAK-D docs + live runtime checks)
- [x] OAK-connected live evidence captured from current logs/processes
- [ ] Phase 0 baseline matrix captured under controlled scenarios
- [ ] Phase 1 quick wins implemented
- [ ] Phase 2 OAK offload/dataflow improvements implemented
- [ ] Full regression + soak validation complete

---

## Priority Backlog

### P0: Measurement + Guardrails (do first)

- [ ] Define KPI thresholds:
  - `loop_dt_ms` p95/p99/max
  - `% samples >=30ms`
  - app CPU and memory
  - depth/person data staleness
  - emergency/disarm response timing
- [ ] Capture reproducible baseline matrix (3x repeats each):
  - idle
  - control-loop only
  - OAK pipeline only
  - full stack nominal
  - full stack under CPU stress
  - thermal soak (20 min)
- [ ] Save baseline report with before/after template.

Acceptance criteria:
- Baseline report committed to `logs/` + summary in `docs/`.
- Thresholds agreed before optimization code changes.

---

### P1: CPU Quick Wins (low risk, no behavior change)

- [ ] In `pi_app/app/main.py`, tighten logging gates:
  - avoid expensive per-tick debug work unless explicitly enabled
  - reduce unnecessary dict/JSON/CSV construction in hot loop
- [ ] Reduce per-tick BT file I/O overhead (`/tmp/wall_e_bt_latest.json`) by decoupling from 50 Hz control loop path.
- [ ] In `pi_app/hardware/oak_recorder.py`, fully skip preview/MCAP image work when not needed.
- [ ] In `pi_app/web/oak_viewer.py`, apply conservative default stream rates under load.

Acceptance criteria:
- >=15% reduction in app CPU (full-feature profile) without control output changes.
- No regression in safety events or motor command behavior.

---

### P2: Core OAK Offload/Dataflow Improvements (highest impact)

- [ ] `pi_app/hardware/oak_depth.py`: replace host ROI percentile depth math with on-device ROI spatial calculation path (DepthAI `SpatialLocationCalculator`), preserving fallback path.
- [ ] Separate control-critical data path from preview/recording data path.
- [ ] Ensure recorder uses true on-device encoded outputs end-to-end:
  - verify/fix recording queue plumbing (`get_recording_queues()` currently not returning queues)
  - keep H.265 bitstream path offloaded to OAK where possible
- [ ] Tune queue policies:
  - control queues: `maxSize=1`, `blocking=False`
  - recording/archival queues: larger buffered async writer path

Acceptance criteria:
- p95 `loop_dt_ms` improvement in full stack mode.
- lower `% >=30ms` vs baseline.
- no increase in stale depth/person events.

---

### P3: Follow-Me Stability + Optional Offload

- [ ] Evaluate OAK `ObjectTracker` integration for person continuity and less host-side target churn.
- [ ] Optionally move lightweight target filtering/selection to DepthAI `Script` node (business logic only; no heavy CV).
- [ ] Align Follow-Me activation semantics across docs/code/tests.

Acceptance criteria:
- reduced target switching/jitter during Follow-Me.
- no unsafe mode transition regressions.

---

### P4: Validate + Ship

- [ ] Re-run full baseline matrix after each phase.
- [ ] Produce before/after summary table:
  - loop timing, CPU, memory, frame freshness, thermal state
- [ ] Run 30-60 min soak for full-feature profile.
- [ ] Keep fallback profile documented:
  - disable web viewer first
  - then reduce recording load
  - autonomous features remain gated by safety

Acceptance criteria:
- Meets KPI thresholds and passes soak without throttling/instability.

---

## Immediate Next Actions (Start Now)

1. [ ] Add/run scenario harness for controlled baseline runs.
2. [ ] Capture and summarize P0 baseline data in a single markdown report.
3. [ ] Implement first P1 quick win in main loop logging/file I/O path.
4. [ ] Re-measure and compare against baseline before moving to next item.

---

## Notes From Current Live Evidence

- Current `latest.log` shows active OAK depth data and mostly stable loop timing with occasional long-tail spikes.
- Historical logs show heavier jitter under loaded OAK/full-feature scenarios.
- Biggest remaining Pi-side cost centers are host frame math, annotation/JPEG/MCAP work, and high-frequency hot-loop overhead.
