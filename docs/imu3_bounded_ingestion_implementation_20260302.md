# IMU-3 Implementation Note (Bounded Ingestion + Poll Decoupling)

Date: 2026-03-02
Scope: Implement IMU-3 mechanics with safe defaults and fallback controls.

## What was implemented

- Added OAK IMU ingestion config controls in `config.py`:
  - `imu_steering.oak_imu_packet_mode` (`latest` or `bounded`, default `latest`)
  - `imu_steering.oak_imu_max_packets_per_poll` (default `4`)
  - `imu_steering.oak_imu_poll_hz` (default `60.0`)

- Wired these settings from `app.main` into `OakDepthReader` construction.

- Updated `OakDepthReader`:
  - decoupled IMU polling cadence from depth poll cadence using a separate IMU poll interval
  - added bounded catch-up cap per outer loop (`max_catchup=4`) to prevent runaway CPU
  - implemented packet selection behavior:
    - `latest` mode: consume newest packet only
    - `bounded` mode: consume up to `N` latest packets per poll
  - maintained metrics accounting for consumed vs coalesced packets

## Safety/Fallback posture

- Default runtime behavior remains conservative (`latest` mode).
- Bounded mode is feature-configurable and can be reverted by setting:
  - `oak_imu_packet_mode = "latest"`
  - `oak_imu_max_packets_per_poll = 1` (optional hard fallback)

## Smoke validation

- Run: `imu3_default_mode_smoke_20s`
- Structured log: `logs/run_20260302_191959.log`
- Analyzer summary:
  - loop_dt mean/p95/p99/max: `19.85 / 21 / 21 / 21 ms`
  - `%>=30ms`: `0.0%`
  - no IMU poll errors/warnings

## Remaining validation before closing IMU-3

## Candidate-N stress validation (runtime override)

Bounded mode was exercised without persistent config flips using runtime overrides:

- `imu3_bounded_n2_stress_30s`
  - log: `logs/run_20260302_192132.log`
  - loop_dt mean/p95/p99/max: `20.60 / 23 / 25 / 25 ms`
  - cadence avg: `0.0344s` (~29 Hz)
  - packet coalesced %: `70.71%`
  - mean CPU: `13.06%`
- `imu3_bounded_n4_stress_30s`
  - log: `logs/run_20260302_192208.log`
  - loop_dt mean/p95/p99/max: `20.41 / 24 / 26 / 26 ms`
  - cadence avg: `0.0184s` (~54 Hz)
  - packet coalesced %: `45.22%`
  - mean CPU: `12.83%`

Both runs showed:

- `%>=30ms`: `0.0%`
- IMU error/warning deltas: `0`
- thermal throttling: none

Fallback verification:

- Latest-mode stress remained available and healthy in prior IMU-1/IMU-2 validation runs.

## IMU-3 closeout status

- Guardrails pass for tested bounded `N` candidates.
- Effective IMU cadence improves materially vs baseline (`~10 Hz` -> `~29-54 Hz`).
- Safe fallback behavior verified (`latest` mode remains default and validated).
