# IMU-2 Validation (Status Precision)

Date: 2026-03-02
Scope: Validate IMU-2 changes that restore float telemetry precision (remove integer coercion in IMU status/log paths).

## Runs Used

- `logs/run_20260302_191311.log` (nominal r1)
- `logs/run_20260302_191414.log` (nominal r2)
- `logs/run_20260302_191517.log` (stress r1)
- `logs/run_20260302_191623.log` (stress r2)

Perf bundles:

- `logs/perf/20260302_191259_imu2_nominal_60s_r1`
- `logs/perf/20260302_191402_imu2_nominal_60s_r2`
- `logs/perf/20260302_191504_imu2_stress_60s_r1`
- `logs/perf/20260302_191608_imu2_stress_60s_r2`

## Loop Guardrails (from `tools/imu_pipeline_analyzer.py`)

| Run | mean/p95/p99/max loop_dt_ms | %>=30ms |
| --- | --- | --- |
| nominal r1 | 20.06 / 21 / 21 / 26 | 0.00% |
| nominal r2 | 20.02 / 21 / 21 / 21 | 0.00% |
| stress r1 | 20.64 / 23 / 25 / 28 | 0.00% |
| stress r2 | 20.80 / 24 / 25 / 26 | 0.00% |

Result: no loop-tail regression from IMU-2 precision changes.

## IMU Pipeline Health

All runs reported:

- `imu_pipeline.metrics_available_pct = 100%`
- `imu_errors.error_delta = 0`
- `imu_errors.warning_delta = 0`

Cadence snapshots remained stable near 0.067-0.068s average.

## Resource/Thermal Snapshot (from perf bundles)

| Run | Mean CPU % | p95 CPU % | Mean temp C | Max temp C | throttled nonzero |
| --- | ---: | ---: | ---: | ---: | ---: |
| nominal r1 | 10.51 | 29.00 | 52.81 | 54.9 | 0 |
| nominal r2 | 10.52 | 32.00 | 52.74 | 53.8 | 0 |
| stress r1 | 10.22 | 32.00 | 67.62 | 71.9 | 0 |
| stress r2 | 10.10 | 35.00 | 71.40 | 73.0 | 0 |

## Precision Verification

- `Controller.get_imu_status()` now returns raw float values (no blanket int rounding).
- Structured log `imu` and `imu_steering` paths preserve those values without `to_int()` coercion.

## Conclusion

- IMU-2 implementation validated.
- Guardrails pass.
- Next phase: IMU-3 bounded packet consumption with configurable fallback.
