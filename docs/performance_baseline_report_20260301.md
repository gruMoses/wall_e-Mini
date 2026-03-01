# Performance Baseline Report (2026-03-01)

## Scope

This report summarizes the baseline artifacts captured under `logs/perf` and the active app log `logs/run_20260301_120511.log`.

## Data quality notes

- Valid artifacts:
  - `S0_idle_30s` (thermal/system idle baseline)
  - `S1_oak_probe_720p30.csv` (camera timing/drops)
  - `run_20260301_120511.log` (control-loop timing with OAK enabled)
- Invalid/partial artifacts:
  - `S2/S3/S4` app scenarios in this run set were not valid for full-loop comparison due to module path / singleton-lock startup issues (app process did not stay alive in those captures).
- Action: re-run `S2/S3/S4` from `pi_app/` with `-m app.main` after stopping any resident app instance before each scenario set.

## Baseline table (current valid measurements)

| Metric | Value | Source |
|---|---:|---|
| Control loop mean (`loop_dt_ms`) | 23.08 ms | `logs/run_20260301_120511.log` |
| Control loop p95 | 31 ms | `logs/run_20260301_120511.log` |
| Control loop p99 | 36 ms | `logs/run_20260301_120511.log` |
| Control loop max | 73 ms | `logs/run_20260301_120511.log` |
| Loop ticks >= 30 ms | 7.69% | `logs/run_20260301_120511.log` |
| OAK probe frames (30s) | 896 | `logs/perf/20260301_115852_S1_oak_probe_720p30.csv` |
| OAK dropped frames | 0 | `logs/perf/20260301_115852_S1_oak_probe_720p30.csv` |
| OAK inter-frame mean | 33.39 ms | `logs/perf/20260301_115852_S1_oak_probe_720p30.csv` |
| OAK inter-frame p95 | 33.389 ms | `logs/perf/20260301_115852_S1_oak_probe_720p30.csv` |
| Idle temp avg / max | 50.44 C / 52.1 C | `logs/perf/20260301_115724_S0_idle_30s/thermal.log` |
| Stress temp avg / max | 62.62 C / 67.0 C | `logs/perf/20260301_120037_S3_full_stress_60s/thermal.log` |
| Thermal throttling observed | No (`0x0`) | stress thermal logs |
| Current live app CPU (single process, 15s sample) | 4.63% total-system CPU | `/proc/<pid>/stat` sampled at runtime |

## Interpretation

- OAK transport itself looks healthy at 720p30 (zero drops, steady frame cadence).
- The control loop still shows latency tail (`p95=31ms`, `p99=36ms`, `max=73ms`), matching previous concern about host-side spikes.
- Thermal headroom is acceptable in current ambient conditions (no throttle flags).

## Recommended first code change

Implement **P1 quick win** in `pi_app/hardware/oak_recorder.py`:

- Add a strict fast-path that **completely bypasses**:
  - RGB annotation/JPEG preview generation
  - Depth colorization/JPEG preview generation
  - MCAP image serialization
- Condition this on recording/viewer config flags so none of that work runs when not actively needed.

Why first:
- It is low-risk, localized, and directly removes recurring OpenCV + encoding work from the Pi CPU path.
- It can be validated quickly with before/after `loop_dt_ms` and app CPU deltas.

## Next measurement step (required)

Re-run full valid matrix:

1. Stop resident app.
2. Run `S2_full_nominal_60s` from `pi_app/` with `python3 -u -m app.main --pid-debug`.
3. Run `S3_full_stress_60s` (same command, with stress-ng active).
4. Run `S4_thermal_soak_180s` (same command).
5. Restart resident app after measurements.

Then use this report as the "before" section for Phase P1 change validation.
