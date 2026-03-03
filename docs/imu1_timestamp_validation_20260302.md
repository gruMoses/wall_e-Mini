# IMU-1 Timestamp Validation (Device dt)

Date: 2026-03-02
Scope: Validate IMU-1 changes (device timestamp propagation + dt integration source change), with packet policy unchanged.

## Runs Used

- `logs/run_20260302_190602.log` (nominal r1)
- `logs/run_20260302_190706.log` (nominal r2)
- `logs/run_20260302_190810.log` (stress r1)
- `logs/run_20260302_190916.log` (stress r2)

Perf bundles:

- `logs/perf/20260302_190549_imu1_nominal_60s_r1`
- `logs/perf/20260302_190653_imu1_nominal_60s_r2`
- `logs/perf/20260302_190757_imu1_stress_60s_r1`
- `logs/perf/20260302_190901_imu1_stress_60s_r2`

## Loop Guardrails (from `tools/imu_pipeline_analyzer.py`)

| Run | mean/p95/p99/max loop_dt_ms | %>=30ms |
| --- | --- | --- |
| nominal r1 | 20.06 / 21 / 21 / 23 | 0.00% |
| nominal r2 | 20.03 / 21 / 21 / 21 | 0.00% |
| stress r1 | 20.68 / 24 / 25 / 27 | 0.00% |
| stress r2 | 20.71 / 24 / 25 / 30 | 0.22% (1/454) |

Result: within configured guardrails; no meaningful loop-tail regression vs IMU-0 baseline set.

## IMU Pipeline dt/Cadence Observations

| Run | cadence avg_s | cadence min_s | cadence max_s | last_sample_age_s | errors |
| --- | ---: | ---: | ---: | ---: | ---: |
| nominal r1 | 0.0674 | 0.0668 | 0.1343 | 0.0280 | 0 |
| nominal r2 | 0.0672 | 0.0668 | 0.1347 | 0.0525 | 0 |
| stress r1 | 0.0681 | 0.0668 | 0.1391 | 0.0100 | 0 |
| stress r2 | 0.0678 | 0.0668 | 0.1365 | 0.0145 | 0 |

Notes:

- Device timestamp propagation is active and consumed successfully.
- Packet policy remains latest-only in this phase; coalescing remains expected/high.
- No IMU poll errors or warning emissions observed.

## Resource/Thermal Snapshot (from perf bundles)

| Run | Mean CPU % | p95 CPU % | Mean temp C | Max temp C | throttled nonzero |
| --- | ---: | ---: | ---: | ---: | ---: |
| nominal r1 | 10.27 | 34.00 | 52.27 | 53.8 | 0 |
| nominal r2 | 10.32 | 28.00 | 51.99 | 53.8 | 0 |
| stress r1 | 10.28 | 28.00 | 67.22 | 71.4 | 0 |
| stress r2 | 10.05 | 39.00 | 71.01 | 73.6 | 0 |

## Conclusion

- IMU-1 implementation validated.
- Guardrails pass for nominal/stress verification runs.
- Next phase: IMU-2 acceptance confirmation in a longer nominal/stress sample, then proceed to IMU-3 bounded packet consumption.
