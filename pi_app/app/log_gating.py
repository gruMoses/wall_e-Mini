"""Pure helpers for gating and pruning WALL-E's structured telemetry logs.

Kept dependency-free (stdlib only) so they're unit-testable without pulling
in the hardware stack (pyserial, depthai, etc.) that ``pi_app.app.main``
imports at module load time.
"""
from __future__ import annotations

import time
from pathlib import Path


def should_log_tick(
    is_armed: bool,
    now_ts: float,
    last_log_ts: float,
    log_interval_s: float,
    heartbeat_s: float,
    has_event: bool,
    charger_inhibit_changed: bool,
    emergency_active: bool,
    mode_changed: bool,
) -> bool:
    """Decide whether the main loop should write a telemetry line this tick.

    Armed: full-rate gate only (``log_interval_s``, historically 10 Hz) --
    unchanged from the original always-on behavior, so armed-state logs stay
    identical in content and cadence to before this change.

    Disarmed: low-rate heartbeat gate (``heartbeat_s``) so idle telemetry
    doesn't flood the disk, UNLESS this tick carries a meaningful event --
    a non-empty ``events`` list, a ``charger_inhibit`` flip, an active
    emergency, or a mode transition -- in which case it always logs
    immediately regardless of heartbeat timing.
    """
    if is_armed:
        return (now_ts - last_log_ts) >= log_interval_s
    if has_event or charger_inhibit_changed or emergency_active or mode_changed:
        return True
    return (now_ts - last_log_ts) >= heartbeat_s


# Bulk telemetry logs that are safe to age out. Small saved tuning artifacts
# (pid_*.json / pid_*.ndjson, tuned_pid.txt, bias_tune*.json, pid_latest.png)
# are intentionally NOT matched here -- those are kept indefinitely.
_PRUNABLE_LOG_GLOBS = ("run_*.log", "arm_*.log", "pid_*.csv")


def cleanup_old_logs(log_dir: Path, days: int = 7) -> None:
    """Delete bulk run/arm/pid-csv logs older than ``days``.

    Does not touch small saved tuning-result files (pid_*.json,
    pid_*.ndjson, tuned_pid.txt, bias_tune*.json, pid_latest.png) -- those
    are intentional keeps, not bulk logs.
    """
    try:
        cutoff = time.time() - days * 24 * 3600
        for pattern in _PRUNABLE_LOG_GLOBS:
            for p in log_dir.glob(pattern):
                try:
                    if p.stat().st_mtime < cutoff:
                        p.unlink()
                except Exception:
                    pass
    except Exception:
        pass
