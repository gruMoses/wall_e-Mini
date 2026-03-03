# WALL-E Mini Performance Optimization TODO

Last updated: 2026-03-02
Scope: Reduce Pi CPU load and maximize OAK-D offload while preserving safety/control behavior.

## Status Snapshot

- [x] Research complete (code audit + OAK-D docs + live runtime checks)
- [x] OAK-connected live evidence captured from current logs/processes
- [ ] Phase 0 baseline matrix captured under controlled scenarios (3x each scenario still pending)
- [x] Phase 1 quick wins implemented
- [x] Phase 2 OAK offload/dataflow improvements implemented
- [x] Full regression + soak validation complete

---

## Active Backlog

### Open Items

- [ ] Complete strict P0 matrix requirement (3x repeats for every listed scenario):
  - idle
  - control-loop only
  - OAK pipeline only
  - full stack nominal
  - full stack under CPU stress
  - thermal soak (20 min)
- [ ] Re-run full baseline matrix after each phase (program-level discipline).
- [ ] Enforce and document "after each phase" rerun discipline in one canonical validation report.

## Notes From Current Live Evidence

- Current `latest.log` shows active OAK depth data and mostly stable loop timing with occasional long-tail spikes.
- Historical logs show heavier jitter under loaded OAK/full-feature scenarios.
- Biggest remaining Pi-side cost centers are host frame math, annotation/JPEG/MCAP work, and high-frequency hot-loop overhead.

### Handoff Docs

- IMU program final handoff: `docs/imu_fidelity_completion_summary_20260302.md`
- IMU phase details:
  - `docs/imu0_instrumentation_snapshot_20260302.md`
  - `docs/imu1_timestamp_validation_20260302.md`
  - `docs/imu2_status_precision_validation_20260302.md`
  - `docs/imu3_bounded_ingestion_implementation_20260302.md`
  - `docs/imu4_cadence_alignment_validation_20260302.md`
  - `docs/imu5_optional_features_implementation_20260302.md`

### Active Remaining Items

- [ ] Complete strict P0 matrix requirement (3x repeats for every listed scenario).
- [ ] Enforce and document "after each phase" rerun discipline in one canonical validation report.

---

## Completed Milestones (Condensed)

- [x] P1 CPU quick wins implemented.
- [x] P2 OAK offload/dataflow improvements implemented.
- [x] P3 Follow-Me stability/docs alignment completed.
- [x] Program-level regression + soak validation completed.
- [x] IMU fidelity program (IMU-0..IMU-5) completed.
