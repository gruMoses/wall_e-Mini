# Performance P1 Results (2026-03-01)

## What was changed

1. `pi_app/hardware/oak_recorder.py`
   - Added preview generation gating so RGB/depth JPEG work only runs when needed:
     - active web stream clients, or
     - active MCAP image recording.
   - Added stream-client counters for RGB/depth.
   - Made preview rates configurable from `config.py`.

2. `pi_app/web/oak_viewer.py`
   - Added client connect/disconnect callbacks to recorder for stream demand tracking.
   - Switched stream and telemetry send rates to config-driven values.

3. `config.py`
   - Added:
     - `OakRecordingConfig.preview_rgb_fps` (default `6.0`)
     - `OakRecordingConfig.preview_depth_fps` (default `3.0`)
     - `OakWebViewerConfig.rgb_stream_fps` (default `6.0`)
     - `OakWebViewerConfig.depth_stream_fps` (default `3.0`)
     - `OakWebViewerConfig.telemetry_hz` (default `4.0`)

4. `pi_app/app/main.py`
   - Reworked BT shared-file reads with mtime + data cache and 50ms poll interval to reduce per-loop file I/O/parsing overhead.

5. `tools/perf/run_scenario.sh`
   - Fixed app shutdown reliability by running app command in its own process group and terminating the whole group (`INT` -> `TERM` -> `KILL`).

6. `pi_app/hardware/oak_depth.py`
   - Replaced `np.percentile` with `np.partition`-based quantiles for depth ROI stats (`p5`, `p50`) to reduce host CPU work.
   - Implemented `get_recording_queues()` return path (was hardcoded `None`) and wired queue lifecycle.
   - Fixed encoder input format by requesting `NV12` from camera for `VideoEncoder` input.

## Validation matrix executed

- `P1post2_S0_idle_30s`
- `P1post2_S1_oak_probe_720p30.csv`
- `P1post2_S2_nominal_60s`
- `P1post2_S3_stress_60s`
- `P1post2_S4_soak_120s`
- Final clean confirmation:
  - `P1final_S2_nominal_60s`
  - `P1final_S3_stress_60s`
- Post-encoder quick regression:
  - `P1final2_S2_nominal_30s`
  - `P1final2_S3_stress_30s`

## Before vs after (key KPIs)

### Control loop timing (`loop_dt_ms`)

| Scenario | Mean (ms) | p95 (ms) | p99 (ms) | Max (ms) | % >= 30ms |
|---|---:|---:|---:|---:|---:|
| Before reference (`run_20260301_120511.log`) | 23.08 | 31 | 36 | 73 | 7.69% |
| After nominal (`run_20260301_125031.log`) | 20.08 | 21 | 21 | 23 | 0.00% |
| After stress (`run_20260301_125139.log`) | 21.20 | 23 | 27 | 34 | 0.43% |

### App CPU from `pidstat`

| Scenario | Mean app `%CPU` |
|---|---:|
| After nominal (`P1final_S2_nominal_60s`) | 14.85 |
| After stress (`P1final_S3_stress_60s`) | 19.82 |

### OAK probe

| Probe | Frames | Drops | Inter-frame mean (ms) | Host latency mean (ms) |
|---|---:|---:|---:|---:|
| Before (`20260301_115852_S1_oak_probe_720p30.csv`) | 896 | 0 | 33.39 | invalid (old timebase bug) |
| After (`20260301_P1post2_S1_oak_probe_720p30.csv`) | 896 | 0 | 33.39 | 49.01 |

### Thermal (stress)

| Scenario | Avg temp (C) | Max temp (C) | Throttle flag |
|---|---:|---:|---|
| Before stress (`20260301_120037_S3_full_stress_60s`) | 62.62 | 67.0 | none |
| After stress (`20260301_125126_P1final_S3_stress_60s`) | 66.51 | 70.3 | none |

## Outcome

- P1 optimizations materially reduced loop latency tail and eliminated almost all slow ticks in nominal and stress conditions.
- OAK transport remains stable (30 FPS cadence, zero drops).
- No thermal throttling observed in the tested windows.
- Encoder warning spam was eliminated after switching encoder input to `NV12`.

## Post-encoder quick check

| Scenario | Mean (ms) | p95 (ms) | p99 (ms) | Max (ms) | % >= 30ms |
|---|---:|---:|---:|---:|---:|
| `P1final2_S2_nominal_30s` (`run_20260301_125518.log`) | 20.05 | 21 | 21 | 22 | 0.00% |
| `P1final2_S3_stress_30s` (`run_20260301_125555.log`) | 21.23 | 24 | 27 | 34 | 0.55% |

## Remaining work / risks

1. Follow-Me dynamic behavior has not yet been validated with a moving human target in this pass.
2. Full P2 offload step (device-side ROI spatial stats replacing host depth quantile path) remains open.
