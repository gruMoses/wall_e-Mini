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

## Additional follow-up findings

- Live person-in-frame FOLLOW_ME validation passed with fast reacquisition and no loop-latency tail reintroduced.
- MCAP dependency was missing at runtime and was installed (`mcap==1.3.1`), restoring `session.mcap` outputs.
- Queue-policy/drain tuning was tested and **reverted** after full 60s stress regression.
- Added `--pid-csv` flag in `app.main` so high-rate PID CSV logging is opt-in instead of always on.
- Added device-side ROI min-distance path via `SpatialLocationCalculator` and reduced host depth-telemetry computation cadence.
- Added recorder trigger gating for obstacle-triggered sessions: armed-only + configurable scale threshold (`obstacle_trigger_scale`).

## Incremental commit trail

- `651dbce`: device-side ROI spatial depth path for obstacle distance.
- `88adeff`: decimated host depth telemetry computation.
- `4d98b79`: armed/threshold gate for obstacle-triggered recording.

## Latest quick-regression snapshot

- Nominal short run (`run_20260301_133300.log`): mean `20.76ms`, p95 `25ms`, max `25ms`, `>=30ms` `0.0%`.
- Stress short run (`run_20260301_133406.log`): mean `23.69ms`, p95 `36ms`, p99 `55ms`, max `70ms`, `>=30ms` `10.47%`.
- Note: short stress windows are noisy and startup-sensitive; use 60s+ repeated stress runs for release gating.

## Repeated validation (2x 60s nominal + 2x 60s stress)

- Nominal repeats:
  - `run_20260301_140056.log`: mean `21.30ms`, p95 `25ms`, p99 `31ms`, max `37ms`, `>=30ms` `1.71%`
  - `run_20260301_140311.log`: mean `21.07ms`, p95 `25ms`, p99 `27ms`, max `35ms`, `>=30ms` `0.43%`
  - Aggregate nominal mean: `21.19ms`; aggregate p95: `25ms`
- Stress repeats:
  - `run_20260301_140205.log`: mean `23.35ms`, p95 `33ms`, p99 `43ms`, max `65ms`, `>=30ms` `8.94%`
  - `run_20260301_140422.log`: mean `23.53ms`, p95 `33ms`, p99 `44ms`, max `84ms`, `>=30ms` `9.89%`
  - Aggregate stress mean: `23.44ms`; aggregate p95: `33ms`

This confirms normal-load behavior is stable and that stress-tail latency remains the main remaining performance risk for deeper optimization work.

## Recording overhead A/B under stress (new profiling flag)

`app.main` now supports `--disable-recording` to isolate recording overhead during profiling.

- Recording enabled (stress): `run_20260301_141745.log`
  - mean `23.47ms`, p95 `32ms`, p99 `60ms`, max `91ms`, `>=30ms` `8.26%`
  - `recording_state`: mostly `RECORDING`/`LINGERING`
- Recording disabled (stress): `run_20260301_141602.log`
  - mean `20.93ms`, p95 `22ms`, p99 `24ms`, max `32ms`, `>=30ms` `0.21%`
  - `recording_state`: `None`

This isolates recording/preview/serialization work as a major contributor to stress-tail latency when active.

## MCAP telemetry write-rate tuning

Added `oak_recording.mcap_telemetry_hz` (default `10.0`) and applied telemetry write-rate limiting in recorder MCAP path.

Stress comparison (recording enabled):
- Pre (`run_20260301_141745.log`): mean `23.47ms`, p95 `32ms`, p99 `60ms`, max `91ms`, `>=30ms` `8.26%`
- Post 1 (`run_20260301_142343.log`): mean `22.99ms`, p95 `32ms`, p99 `40ms`, max `56ms`, `>=30ms` `9.01%`
- Post 2 (`run_20260301_142515.log`): mean `23.04ms`, p95 `31ms`, p99 `50ms`, max `69ms`, `>=30ms` `6.38%`

Trend: lower high-tail outliers (p99/max) with similar mean loop time.

## Lower default MCAP snapshot rates

Defaults updated:
- `mcap_image_fps`: `5.0 -> 3.0`
- `mcap_depth_fps`: `2.0 -> 1.0`
- `mcap_telemetry_hz`: `10.0 -> 5.0`

Stress comparison (recording enabled):
- Baseline (`run_20260301_142515.log`): mean `23.04ms`, p95 `31ms`, p99 `50ms`, max `69ms`, `>=30ms` `6.38%`
- Low-MCAP #1 (`run_20260301_143115.log`): mean `22.82ms`, p95 `32ms`, p99 `45ms`, max `68ms`, `>=30ms` `7.31%`
- Low-MCAP #2 (`run_20260301_143258.log`): mean `22.50ms`, p95 `31ms`, p99 `37ms`, max `59ms`, `>=30ms` `7.03%`

Net: lower mean and much better high-tail outliers (p99/max), with a slight increase in `>=30ms` frequency.

## Remaining work / risks

1. Full P2 offload step (device-side ROI spatial stats replacing host depth quantile path) remains open.
2. Recording trigger policy may still over-record in tight indoor spaces when obstacle scaling stays below `1.0` for long periods.

## MCAP image snapshots: follow/person-only gating

Added `oak_recording.mcap_images_follow_only` (default `True`) and gated MCAP RGB/depth snapshot generation so obstacle-only recordings keep video+telemetry but skip MCAP image serialization work unless:
- mode is `FOLLOW_ME`, or
- person detections are present, or
- a live web stream client is connected (preview still served for viewer clients).

Initial validation:
- Stress + recording (`run_20260301_144004.log`):
  - mean `20.66ms`, p95 `23ms`, p99 `25ms`, max `26ms`, `>=30ms` `0.00%`
  - `recording_state`: mostly `RECORDING`/`LINGERING`
- Follow-me probe (`follow_me_probe.py`, 35s window):
  - mode rows: `FOLLOW_ME=327`, `MANUAL=3`
  - tracking `30.61%` / persons-seen `30.61%` (scene-dependent)
  - loop `mean 21.07ms`, p95 `25ms`, p99 `27ms`, max `35ms`, `>=30ms 0.91%`

This is a strong improvement vs recent recording-enabled stress baselines and indicates MCAP image path load was a primary stress-tail source in obstacle-only sessions.
