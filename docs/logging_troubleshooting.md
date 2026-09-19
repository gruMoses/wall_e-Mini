# Logging and Troubleshooting (Dashboard / OAK Feed)

This doc summarizes where logs live, how to use them for the **dashboard video feed root-cause** testing, and recommendations to improve logging if tests don’t go well.

NOTE: the "Current logging layout" table below predates the 2026-09-19
logging audit. Read "Log format (2026-09-19)" first for the current line
shapes; the table is kept for the parts that are still accurate (file
paths, journalctl commands).

## Current logging layout

| What | Where | Notes |
|------|--------|--------|
| **Structured JSON (main loop)** | `logs/run_YYYYMMDD_HHMMSS.log` | One JSON object per line, ~10 Hz. Symlink: `logs/latest.log`. |
| **Camera health** | Inside each JSON line as `oak_camera_health` | `pipeline_running`, `*_age_s`, `*_stale`, `is_stale`, `last_*_error`. |
| **Health transition warnings** | stdout (journalctl) | Only when health flips healthy↔stale; includes age fields and last errors. |
| **Module loggers** (oak_depth, recorder, viewer, etc.) | stderr → journalctl | `journalctl -u wall-e.service -n 500 --no-pager` |

## Key log locations for dashboard testing

- **Structured run log:** `logs/latest.log` (or the current `run_*.log`).
- **Service stdout/stderr:** `journalctl -u wall-e.service -f` (follow) or `-n 200` (last 200 lines).
- **Dashboard:** `http://<robot-ip>:8080/` — telemetry includes `camera_health` (RGB/Depth age, Pipeline running).

## Recommendations to improve logging (if testing doesn’t go well)

1. **Structured log: add a "health changed" event** — DONE 2026-09-19.
   The main loop writes `{"type":"event","event":"oak_health","from":...,"to":...,"ts":...}`
   on every transition, in addition to the existing console print. See
   "Log format (2026-09-19)" below.

2. **Emit transition to the structured log file (not only print)** — DONE 2026-09-19,
   same change as item 1.

3. **OAK thread: log first exception with full traceback**
   - In `oak_depth.py`, when setting `_last_depth_error_msg` / `_last_pipeline_error_msg` etc., the first time an error is set in a run, call `logger.exception(...)` (or `logger.error(..., exc_info=True)`) so journalctl gets one full traceback. Subsequent same-error can stay as the short summary to avoid spam. That helps distinguish pipeline crash vs device vs host-side bugs.

4. **Optional: periodic "heartbeat" line in structured log** — DONE 2026-09-19
   (differently than proposed): a 1 Hz `{"type":"slow",...}` line now carries
   `imu_pipeline`, `oak_camera_health`, and `oak.chip_temp_c`, replacing the
   heartbeat idea with something more useful than just confirming the loop
   is alive. See below.

5. **Dashboard: surface last error strings**
   - Telemetry already exposes `camera_health`. If not already shown, add the `last_*_error` strings to the dashboard (e.g. collapsed or tooltip) so you can see the last OAK error without opening logs.

6. **Retention and disk**
   - Run logs can grow large. Keep `_cleanup_old_logs(..., days=7)` (or configurable). For a failure run, copy the relevant `run_*.log` (and a journalctl snippet) off the Pi before the next restart so the exact failure window is preserved.

## Quick checks during/after a bad run

- **When did health go stale?**  
  `grep '"is_stale": true' logs/latest.log | head -1` and check `ts_iso` / `ts`.
- **What did the pipeline report?**  
  Last line before staleness: `oak_camera_health.last_pipeline_error`, `last_depth_error`, etc., in that line.
- **Did the control loop keep ticking?**  
  `ts` and `loop_dt_ms` should keep advancing; if they stop, the main process likely hung or exited.
- **Journal around the time of stall:**  
  `journalctl -u wall-e.service --since "2026-03-13 18:25" --until "2026-03-13 18:35"` (adjust to your stall window).

## Log format (2026-09-19)

A 2026-09-19 audit found `logs/run_*.log` / `logs/arm_*.log` wrote 7,658 B
per armed line: `imu.oak_imu` alone was 2,197 B, `imu_pipeline` 1,539 B,
`oak_camera_health` 710 B. Most of that was monotonic counters that change
over seconds, logged 10 times a second. This section describes the format
after the fix. Each JSON line has a `"type"` field, except the original
per-tick line, which has none (it predates this change and stays that way
for backward compatibility with existing tools).

### Per-tick line (no `"type"` field)

One line per tick while armed (10 Hz); a low-rate heartbeat plus
always-log-on-event while disarmed (`should_log_tick`,
`pi_app/app/log_gating.py`). Built by `build_log_obj()`. Top-level blocks:

- `rc`, `bt`, `motor`, `safety`, `pid`, `obstacle`, `waypoint_nav`,
  `detections`, `heading_align`, `recording_state`, `bms`, `events`.
- `imu`: `controller.get_imu_status()`, rounded to 3 decimals. Its nested
  `oak_imu` sub-block drops the counters that duplicate `imu_pipeline`
  (`producer_*`, `queue_*`, `drain_batch_*`, `cadence_*`, `host_queue_*`,
  `max_packets_per_drain`, plus `last_batch_packets`,
  `zupt_engage_count`, `bias_updates`, `bias_g[xyz]_dps`,
  `window_*_std_*`, `last_bias_update_host_ts`,
  `stationary_tracking_enabled`, `zupt_enabled`) — confirmed key-for-key
  against `OakDepthReader.get_imu_metrics()`. It keeps
  `yaw_rate_source_*`, `yaw_axis_sign`, `integrate_status`,
  `integration_path`, `sample_age_s`, `last_dt_s`, `gx/gy/gz_body_dps`,
  `yaw_rate_world_dps`, `heading_deg`, `tracked_bias_dps`,
  `gyro_bias_dps`, `stationary`, `zupt_active`, and `count_*`.
- `imu_steering`: `steering_input`, `correction_raw`, `correction_applied`,
  `correction_blend`, `speed_gain_scale`, `saturated` (the anti-windup
  clamp flag).
- `vesc`: adds `rpm_plausible`, `gate_trips`, `l_temp_c`/`r_temp_c`,
  `l_motor_temp_c`/`r_motor_temp_c`, `l_duty`/`r_duty` (all were already
  computed by `controller.process()` and simply not logged before).
- `gps`: adds `age_s` (seconds since the current fix, monotonic clock;
  `None` when there is no fix this tick).
- `follow_me`: adds `speed_loop` — `open_loop_byte`, `target_mps`,
  `actual_mps`, `err_mps`, `p`, `i`, `d`, `corr_mps`, `corr_byte`,
  `closed` (bool: whether the closed-loop velocity PID ran this tick; `p`/
  `i`/`d` are only meaningful when `closed` is `True`).
- Top level also adds `straight_intent`.
- Dropped: `imu_pipeline`, `oak_camera_health` (moved to the slow line —
  see below), and the flat `heading_offset_deg` / `heading_offset_locked`
  / `heading_offset_frozen` / `heading_offset_refining` keys (the same
  values are in `heading_align.offset_deg` / `.locked` / `.frozen` /
  `.refining` on the same line). `corrected_heading_deg` is unchanged.

### Slow line (`"type": "slow"`)

One line per second, armed or not (`should_write_slow_line`,
`build_slow_obj()`). Carries `imu_pipeline` and `oak_camera_health` at full
float precision (the per-tick `imu` block above is rounded to 3 decimals;
this line is not), plus `oak.chip_temp_c` — the OAK module's chip
temperature, sampled once a second from
`pipeline.getDefaultDevice().getChipTemperature()` inside the pipeline
worker thread.

`tools/imu_pipeline_analyzer.py` reads `imu_pipeline` from the slow line
when present, and falls back to the old per-tick location for logs
captured before this change.

### Session header (`"type": "session_header"`)

The first line written to every new `run_*.log` and `arm_*.log`
(`_session_header()`, called right after each `_open_log_file()`).
Carries `git_sha`, `git_dirty`, `file`, and a `config` block with the
tunables that matter for interpreting a run or deriving PID gains offline:
`imu_steering` (`kp`/`ki`/`kd`/`max_correction`/`invert_output`/
`deadband_deg`/`oak_yaw_rate_source`/`oak_yaw_rate_scale`/
`oak_stationary_*`/`oak_zupt_enabled`/`oak_yaw_axis_sign_auto`),
`follow_me` (`speed_kp`/`speed_ki`/`speed_kd`/`speed_integral_limit`/
`speed_pid_max_correction_mps`/`speed_loop_mps_per_byte`/
`follow_distance_m`/`max_follow_speed_byte`), `vesc`
(`rpm_plausibility_*`), `waypoint_nav` (`min_rtk_quality`/
`align_threshold_deg`/`recovery_threshold_deg`/`pivot_yaw_cmd`),
`gps_heading_align.max_lock_yaw_rate_dps`, and `imu_source`.

### Event lines (`"type": "event"`)

Written immediately when something happens, not on a timer. Currently one
kind: `{"type":"event","event":"oak_health","from":"HEALTHY"|"STALE",
"to":"HEALTHY"|"STALE","ts":...}`, mirroring the console
"OAK camera health transition" print.

### GPS (2026-09-19, Commit D)

The per-tick `gps` block gains `utc` (ISO 8601 string, `None` before the
receiver has a time fix), `cog_deg` and `sog_mps` (course/speed over
ground; `None` when the receiver reports no course, e.g. stationary),
`nmea_mode` (the RMC/VTG mode-indicator character), and `geoid_sep_m`.
`cog_deg`/`sog_mps` are logged only -- nothing in the control path reads
them yet.

The slow line gains `gps_health`: `RtkGpsReader.get_health()` --
`reconnect_count`, `poll_error_count`, `consecutive_i2c_errors`,
`last_poll_age_s`, `rmc_errors`, `mode_register_value` (the raw value last
read from operation-mode register 93).

A fix-quality change (e.g. RTK float -> RTK fixed) is logged separately at
WARNING with UTC/sats/HDOP/diff-age/station-id context, independent of the
JSON log -- `journalctl -u wall-e.service | grep 'fix_quality'` finds it
even without a copy of the run log. See `docs/rtk_gps_module.md` for the
register map and the read protocol this is built on.
