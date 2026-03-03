# IMU-4 Validation (Controller Cadence Alignment)

Date: 2026-03-02
Scope: Align controller-side IMU update cap with OAK IMU ingestion cadence, without changing main-loop cadence.

## Implementation

- `config.py`:
  - `imu_steering.update_rate_hz` set to `60.0` (aligned with default OAK IMU poll)
- `controller.py`:
  - controller now uses `min(update_rate_hz, oak_imu_poll_hz)` when computing IMU update interval
  - maintains positive-rate validation and existing fallback behavior

## Validation runs

- `logs/run_20260302_192432.log` (nominal 30s)
- `logs/run_20260302_192507.log` (stress 30s)

## Loop and IMU metrics (`tools/imu_pipeline_analyzer.py`)

| Run | mean/p95/p99/max loop_dt_ms | %>=30ms | cadence avg_s | errors |
| --- | --- | --- | --- | --- |
| nominal | 20.01 / 21 / 21 / 25 | 0.00% | 0.0667 | 0 |
| stress | 20.72 / 23 / 25 / 26 | 0.00% | 0.0674 | 0 |

## Resource/thermal snapshot (perf bundles)

| Run | Mean CPU % | p95 CPU % | Mean temp C | Max temp C | throttled nonzero |
| --- | ---: | ---: | ---: | ---: | ---: |
| nominal | 13.73 | 55.00 | 53.57 | 55.4 | 0 |
| stress | 13.39 | 47.00 | 66.15 | 71.4 | 0 |

## Conclusion

- IMU-4 alignment changes validated.
- Guardrails pass in quick nominal/stress checks.
- Main loop cadence remains unchanged.
