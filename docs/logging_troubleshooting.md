# Logging and Troubleshooting (Dashboard / OAK Feed)

This doc summarizes where logs live, how to use them for the **dashboard video feed root-cause** testing, and recommendations to improve logging if tests don’t go well.

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

1. **Structured log: add a “health changed” event**
   - In the main loop, when `oak_prev_stale != is_stale`, add a dedicated field in the same JSON line, e.g. `"oak_health_transition": "healthy->stale"` or `"stale->healthy"`, plus a one-line summary of which subsystem(s) are stale. That gives a machine-readable timeline of health transitions in the same file as the 10 Hz tick.

2. **Emit transition to the structured log file (not only print)**
   - Today the healthy↔stale message is only `print()` (so journalctl). Also write one extra JSON line to `log_fh` on transition, e.g. `{"ts": ..., "event": "oak_health_transition", "state": "stale", "ages": {...}, "errors": [...]}`. Then you can `grep "oak_health_transition" logs/latest.log` and align with the tick stream.

3. **OAK thread: log first exception with full traceback**
   - In `oak_depth.py`, when setting `_last_depth_error_msg` / `_last_pipeline_error_msg` etc., the first time an error is set in a run, call `logger.exception(...)` (or `logger.error(..., exc_info=True)`) so journalctl gets one full traceback. Subsequent same-error can stay as the short summary to avoid spam. That helps distinguish pipeline crash vs device vs host-side bugs.

4. **Optional: periodic “heartbeat” line in structured log**
   - Every N seconds (e.g. 60), write a minimal JSON line with `ts`, `event: "heartbeat"`, and `oak_camera_health`. Confirms that the main loop is still writing and gives a low-rate health sample for long runs without scanning the whole file.

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
