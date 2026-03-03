# IMU-5 Optional Features (Flag-Gated) - Implementation Note

Date: 2026-03-02
Scope: Add optional drift/noise mitigation hooks and validate conservative default enablement.

## Implemented

In `config.py` under `ImuSteeringConfig`:

- `oak_nmni_enabled` (default `True` after guardrail validation)
- `oak_nmni_threshold_dps` (default `0.3`)
- `oak_bias_adapt_enabled` (default `False`)
- `oak_bias_adapt_alpha` (default `0.001`)
- `dterm_ema_alpha` (default `0.0`, disabled)

Runtime wiring:

- `app.main` now passes OAK NMNI/bias flags into `OakImuReader`.
- `OakImuReader` applies:
  - optional NMNI zeroing for low yaw rates
  - optional stationary gyro bias adaptation
- `ImuSteeringCompensator` applies optional D-term EMA (`dterm_ema_alpha`).

## Smoke check

- Scenario: `imu5_flags_on_smoke_20s` (runtime override enabling all optional features)
- Log: `logs/run_20260302_193129.log`
- Analyzer:
  - loop_dt mean/p95/p99/max: `19.83 / 20 / 21 / 21 ms`
  - `%>=30ms`: `0.0%`
  - IMU errors/warnings: `0`

## Status

- Implementation complete.
- Conservative default now enabled: NMNI on, bias-adapt off, D-term EMA off.

## One-at-a-time A/B smoke checks

Scenarios:

- `imu5_nmni_only_smoke_20s` -> `logs/run_20260302_193243.log`
- `imu5_bias_only_smoke_20s` -> `logs/run_20260302_193307.log`
- `imu5_dterm_only_smoke_20s` -> `logs/run_20260302_193330.log`

Observed in all three runs:

- loop_dt mean ~`19.85-19.90ms`
- `%>=30ms`: `0.0%`
- IMU errors/warnings: `0`

Status update:

- One-at-a-time feature A/B smoke checks completed.
- Proceeded to default-enable guardrail matrix (below).

## Stationary drift benchmark (default vs NMNI+bias)

Runs:

- Default: `logs/run_20260302_193435.log`
- NMNI+bias enabled: `logs/run_20260302_193539.log`

Extracted comparison (from structured `imu` telemetry):

| Mode | samples | heading delta (deg) | heading span (deg) | abs yaw mean (dps) | abs yaw p95 (dps) |
| --- | ---: | ---: | ---: | ---: | ---: |
| default | 486 | -1.825 | 1.957 | 0.0445 | 0.0843 |
| NMNI+bias | 487 | 0.0 | 0.0 | 0.0 | 0.0 |

Interpretation:

- In this stationary benchmark, optional NMNI+bias shows materially lower observed drift/noise.

## Enabled-default guardrail matrix (NMNI default ON)

Scenarios and logs:

- nominal r1: `logs/run_20260302_194758.log`
- nominal r2: `logs/run_20260302_194901.log`
- stress r1: `logs/run_20260302_195007.log`
- stress r2: `logs/run_20260302_195113.log`
- soak 30m: `logs/run_20260302_195215.log`

Loop and IMU health (`tools/imu_pipeline_analyzer.py`):

| Run | mean/p95/p99/max loop_dt_ms | %>=30ms | imu errors |
| --- | --- | --- | --- |
| nominal r1 | 20.04 / 21 / 21 / 25 | 0.00% | 0 |
| nominal r2 | 20.05 / 21 / 21 / 22 | 0.00% | 0 |
| stress r1 | 20.83 / 24 / 26 / 28 | 0.00% | 0 |
| stress r2 | 20.68 / 24 / 25 / 27 | 0.00% | 0 |
| soak 30m | 20.09 / 21 / 21 / 26 | 0.00% | 0 |

Resource/thermal snapshot (perf bundles):

| Scenario | Mean CPU % | Mean RSS MB | Mean temp C | Max temp C | throttle flags |
| --- | ---: | ---: | ---: | ---: | --- |
| nominal r1 | 10.23 | 304.5 | 52.83 | 55.4 | none |
| nominal r2 | 10.55 | 304.4 | 52.46 | 54.3 | none |
| stress r1 | 10.31 | 303.1 | 67.72 | 71.9 | none |
| stress r2 | 10.30 | 296.9 | 71.51 | 73.6 | none |
| soak 30m | 6.89 | 321.0 | 52.66 | 73.6 | none |

Decision:

- Guardrails pass with NMNI enabled as default.
- Keep bias adaptation and D-term EMA disabled by default pending broader driving A/B.
