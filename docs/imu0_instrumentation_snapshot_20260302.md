# IMU Instrumentation Snapshot

Date: 2026-03-02
Branch/commit: local working tree (uncommitted)
Operator: Codex (assisted run)
Run profile: nominal + stress + soak
Duration: 2x nominal 60s, 2x stress 60s, 1x soak 30m

## Baseline Context

- Baseline source report: `docs/performance_baseline_report_20260301.md`
- Comparison window: latest IMU-0 matrix runs on 2026-03-02
- Notes: IMU-0 instrumentation-only validation run set completed.

## Completed Run Logs

- `logs/run_20260302_182226.log` (nominal r1)
- `logs/run_20260302_182337.log` (nominal r2)
- `logs/run_20260302_182451.log` (stress r1)
- `logs/run_20260302_182626.log` (stress r2)
- `logs/run_20260302_182759.log` (soak 30m)

## Per-Run Loop + IMU Summary

| Run | mean/p95/p99/max loop_dt_ms | %>=30ms | IMU cadence avg | IMU coalesced % | IMU errors |
| --- | --- | --- | --- | --- | --- |
| nominal r1 | 20.05 / 21 / 21 / 25 | 0.00% | 0.1s (~10Hz) | 85.20% | 0 |
| nominal r2 | 20.06 / 21 / 21 / 21 | 0.00% | 0.1s (~10Hz) | 85.27% | 0 |
| stress r1 | 20.79 / 24 / 26 / 27 | 0.00% | 0.1s (~10Hz) | 85.57% | 0 |
| stress r2 | 20.63 / 23 / 25 / 29 | 0.00% | 0.1s (~10Hz) | 85.28% | 0 |
| soak 30m | 20.09 / 21 / 21 / 26 | 0.00% | 0.1s (~10Hz) | 85.25% | 0 |

## IMU-0 Instrumentation Metrics

| Metric | Value | Notes |
| --- | --- | --- |
| IMU packets received | ~4.9k per 60s run; 180k over 30m soak | From `tools/imu_pipeline_analyzer.py` |
| IMU packets consumed | ~0.71-0.74k per 60s run; 26.6k over soak | Latest-only consumption policy |
| IMU packets coalesced/dropped | ~4.15k-4.26k per 60s run; 153.7k over soak | Expected under latest-only mode |
| IMU queue drains | ~678-723 per 60s run; 26k over soak | Confirms queue coalescing activity |
| IMU sample age p50/p95/max (ms) | ~100 / ~100 / ~100 | Based on `last_sample_age_s` and cadence snapshot |
| Effective IMU cadence avg/p95 (Hz) | ~10 Hz avg | Current poll coupling limits effective cadence |
| First error observed | none | `error_delta=0`, `warning_delta=0` |
| Throttled periodic summary count | 0 | No warnings emitted |

## Guardrail Snapshot (completed runs only)

| Guardrail | Result | Delta vs baseline |
| --- | --- | --- |
| `loop_dt_ms` p95 | pass | Nominal: 21ms; Stress: 23-24ms |
| `loop_dt_ms` p99 | pass | Nominal: 21ms; Stress: 25-26ms |
| `% >=30ms` | pass | `0.0%` on all four completed runs |
| Max `loop_dt_ms` | pass | Nominal: 21-25ms; Stress: 27-29ms |
| App CPU mean | pass | nominal ~10.1%, stress ~10.2-10.3%, soak 6.9% |
| Thermal throttling | pass | `throttled=0x0` throughout all runs |

## Perf Bundle Resource Summary

| Scenario | Mean CPU % | p95 CPU % | Mean RSS MB | Max RSS MB | Mean temp C | Max temp C | Throttle flags |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| nominal r1 | 10.29 | 32.00 | 304.8 | 322.6 | 49.67 | 51.0 | none |
| nominal r2 | 10.06 | 32.00 | 304.1 | 321.4 | 49.85 | 51.0 | none |
| stress r1 | 10.29 | 26.00 | 303.1 | 323.2 | 65.20 | 69.7 | none |
| stress r2 | 10.21 | 30.00 | 302.5 | 322.1 | 67.26 | 71.4 | none |
| soak 30m | 6.88 | 8.00 | 321.5 | 322.7 | 51.31 | 55.4 | none |

## Analyzer Output References

Use:

```bash
python3 tools/imu_pipeline_analyzer.py --log logs/run_20260302_182226.log
python3 tools/imu_pipeline_analyzer.py --log logs/run_20260302_182337.log
python3 tools/imu_pipeline_analyzer.py --log logs/run_20260302_182451.log
python3 tools/imu_pipeline_analyzer.py --log logs/run_20260302_182626.log
python3 tools/imu_pipeline_analyzer.py --log logs/run_20260302_182759.log
```

## Conclusion

- Instrumentation overhead assessment: negligible across nominal/stress/soak runs.
- IMU-0 status: complete (instrumentation-only phase guardrails passed).
- Follow-ups:
  - finalize TODO checkboxes for IMU-0 and begin IMU-1 (timestamp propagation).
