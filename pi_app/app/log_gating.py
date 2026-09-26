"""Pure helpers for gating and pruning WALL-E's structured telemetry logs.

Kept dependency-free (stdlib only) so they're unit-testable without pulling
in the hardware stack (pyserial, depthai, etc.) that ``pi_app.app.main``
imports at module load time.
"""
from __future__ import annotations

import subprocess
import time
from datetime import datetime
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


# ---------------------------------------------------------------------------
# Value formatting helpers, shared by the console heartbeat gate and the
# structured-log builders below.
# ---------------------------------------------------------------------------

def to_int(val):
    """Recursively round numeric leaves to the nearest int (bools untouched)."""
    if isinstance(val, dict):
        return {k: to_int(v) for k, v in val.items()}
    if isinstance(val, list):
        return [to_int(v) for v in val]
    if isinstance(val, (int, float)) and not isinstance(val, bool):
        return int(round(val))
    return val


def round1(val):
    """Recursively round numeric leaves to 1 decimal place (bools untouched)."""
    if isinstance(val, dict):
        return {k: round1(v) for k, v in val.items()}
    if isinstance(val, list):
        return [round1(v) for v in val]
    if isinstance(val, (int, float)) and not isinstance(val, bool):
        return round(float(val), 1)
    return val


def round_floats(val, ndigits: int):
    """Recursively round numeric leaves to ``ndigits`` (bools untouched).

    Unlike ``round1``/``to_int`` this also rounds tuple leaves (e.g. the OAK
    IMU's ``gyro_bias_dps`` / ``accel_mean_g`` 3-tuples), which are common in
    the ``imu`` block but never appeared in the blocks ``round1`` was written
    for.
    """
    if isinstance(val, dict):
        return {k: round_floats(v, ndigits) for k, v in val.items()}
    if isinstance(val, (list, tuple)):
        rounded = [round_floats(v, ndigits) for v in val]
        return tuple(rounded) if isinstance(val, tuple) else rounded
    if isinstance(val, (int, float)) and not isinstance(val, bool):
        return round(float(val), ndigits)
    return val


# ---------------------------------------------------------------------------
# imu.oak_imu per-tick filter (2026-09-19 logging audit, Commit C)
# ---------------------------------------------------------------------------
#
# OakImuReader.get_health() re-exports OakDepthReader.get_imu_metrics()
# wholesale (the `imu_metrics` dict merged via **imu_metrics), so every key
# below is an exact duplicate of a key already in the separate `imu_pipeline`
# block -- confirmed key-for-key against pi_app/hardware/oak_depth.py's
# get_imu_metrics(). Dropping them from the per-tick imu.oak_imu dict does
# NOT touch get_health() itself (still full for live consumers: the field
# chalk-test CLI, controller.get_imu_status()) -- only what main.py copies
# into the per-tick log line.
_OAK_IMU_DROP_PREFIXES = ("producer_", "queue_", "drain_batch_", "cadence_", "host_queue_")
_OAK_IMU_DROP_KEYS = frozenset({
    "max_packets_per_drain",
    "last_batch_packets",
    "zupt_engage_count",
    "bias_updates",
    "bias_gx_dps",
    "bias_gy_dps",
    "bias_gz_dps",
    "window_gyro_std_dps",
    "window_accel_std_g",
    "last_bias_update_host_ts",
    "stationary_tracking_enabled",
    "zupt_enabled",
})


def _filter_oak_imu_for_log(oak_imu: dict) -> dict:
    """Drop the imu.oak_imu keys that duplicate the `imu_pipeline`/slow-line
    block. Keeps OakImuReader's own state: yaw_rate_source_*, yaw_axis_sign,
    integrate_status, integration_path, sample_age_s, last_dt_s,
    gx/gy/gz_body_dps, yaw_rate_world_dps, heading_deg, tracked_bias_dps,
    gyro_bias_dps, stationary, zupt_active, count_*, and the oak_* health
    flags -- none of which are re-exports of get_imu_metrics().
    """
    if not isinstance(oak_imu, dict):
        return oak_imu
    out = {}
    for k, v in oak_imu.items():
        if k in _OAK_IMU_DROP_KEYS:
            continue
        if any(k.startswith(p) for p in _OAK_IMU_DROP_PREFIXES):
            continue
        out[k] = v
    return out


def _filter_imu_status_for_log(imu_status):
    """Apply ``_filter_oak_imu_for_log`` to imu_status["oak_imu"] (if
    present) without mutating the caller's dict."""
    if not isinstance(imu_status, dict):
        return imu_status
    if "oak_imu" not in imu_status:
        return imu_status
    filtered = dict(imu_status)
    filtered["oak_imu"] = _filter_oak_imu_for_log(imu_status["oak_imu"])
    return filtered


# ---------------------------------------------------------------------------
# Console heartbeat gate
# ---------------------------------------------------------------------------

def should_print_console_line(
    is_tty: bool, now: float, last_print_t: float, min_interval_s: float = 5.0
) -> bool:
    """Decide whether to print the per-tick console heartbeat line.

    An interactive TTY gets it at loop rate (the ``\\r``-overwritten status
    line CLI users expect). Under systemd, stdout is the journal: nothing
    overwrites, ``flush=True`` makes every call its own journal entry, and a
    30 Hz loop cost ~21 MB/h of "[blob data]" lines for no operational
    benefit. A non-TTY is throttled to at most one line per
    ``min_interval_s``.
    """
    if is_tty:
        return True
    return (now - last_print_t) >= min_interval_s


# ---------------------------------------------------------------------------
# Session header (first line of every run_*.log / arm_*.log)
# ---------------------------------------------------------------------------

def _git_sha(timeout_s: float = 2.0) -> str:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, timeout=timeout_s, check=True,
        )
        sha = out.stdout.strip()
        return sha if sha else "unknown"
    except Exception:
        return "unknown"


def _git_dirty(timeout_s: float = 2.0):
    """True if tracked files have uncommitted changes, else False; None if
    the check itself failed (e.g. git missing, not a repo, timed out)."""
    try:
        out = subprocess.run(
            ["git", "status", "--porcelain", "--untracked-files=no"],
            capture_output=True, text=True, timeout=timeout_s, check=True,
        )
        return bool(out.stdout.strip())
    except Exception:
        return None


def _session_header(config, path) -> dict:
    """Build the JSON session-header object written as the first line of
    every new structured log file (run_*.log and arm_*.log).

    Captures the git commit + dirty flag and the tunable config values in
    effect for this session, so a log can be interpreted -- and PID gains
    derived offline -- without cross-referencing config.py history. A pure
    function of (config, path) so it's unit-testable by patching subprocess.
    """
    imu_steering = getattr(config, "imu_steering", None)
    follow_me = getattr(config, "follow_me", None)
    vesc = getattr(config, "vesc", None)
    waypoint_nav = getattr(config, "waypoint_nav", None)
    gps_heading_align = getattr(config, "gps_heading_align", None)
    now = time.time()
    return {
        "type": "session_header",
        "schema": 1,
        "ts": round(now, 3),
        "ts_iso": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
        "git_sha": _git_sha(),
        "git_dirty": _git_dirty(),
        "file": str(path) if path is not None else None,
        "config": {
            "imu_steering": {
                "kp": getattr(imu_steering, "kp", None),
                "ki": getattr(imu_steering, "ki", None),
                "kd": getattr(imu_steering, "kd", None),
                "max_correction": getattr(imu_steering, "max_correction", None),
                "invert_output": getattr(imu_steering, "invert_output", None),
                "deadband_deg": getattr(imu_steering, "deadband_deg", None),
                "oak_yaw_rate_source": getattr(imu_steering, "oak_yaw_rate_source", None),
                "oak_yaw_rate_scale": getattr(imu_steering, "oak_yaw_rate_scale", None),
                "oak_stationary_bias_tracking_enabled": getattr(
                    imu_steering, "oak_stationary_bias_tracking_enabled", None
                ),
                "oak_stationary_window_s": getattr(imu_steering, "oak_stationary_window_s", None),
                "oak_stationary_gyro_std_dps": getattr(
                    imu_steering, "oak_stationary_gyro_std_dps", None
                ),
                "oak_stationary_accel_std_g": getattr(
                    imu_steering, "oak_stationary_accel_std_g", None
                ),
                "oak_stationary_max_rate_dps": getattr(
                    imu_steering, "oak_stationary_max_rate_dps", None
                ),
                "oak_stationary_bias_tau_s": getattr(imu_steering, "oak_stationary_bias_tau_s", None),
                "oak_zupt_enabled": getattr(imu_steering, "oak_zupt_enabled", None),
                "oak_yaw_axis_sign_auto": getattr(imu_steering, "oak_yaw_axis_sign_auto", None),
            },
            "follow_me": {
                "speed_kp": getattr(follow_me, "speed_kp", None),
                "speed_ki": getattr(follow_me, "speed_ki", None),
                "speed_kd": getattr(follow_me, "speed_kd", None),
                "speed_integral_limit": getattr(follow_me, "speed_integral_limit", None),
                "speed_pid_max_correction_mps": getattr(
                    follow_me, "speed_pid_max_correction_mps", None
                ),
                "speed_loop_mps_per_byte": getattr(follow_me, "speed_loop_mps_per_byte", None),
                "follow_distance_m": getattr(follow_me, "follow_distance_m", None),
                "max_follow_speed_byte": getattr(follow_me, "max_follow_speed_byte", None),
                # Steering and detection tunables retuned 2026-09-19; the
                # analyzer (tools/analyze_follow_me_log.py) reads them here.
                "pid_lateral_kp": getattr(follow_me, "pid_lateral_kp", None),
                "pid_lateral_kd": getattr(follow_me, "pid_lateral_kd", None),
                "max_steer_offset_byte": getattr(follow_me, "max_steer_offset_byte", None),
                "direct_mode_max_steer_byte": getattr(follow_me, "direct_mode_max_steer_byte", None),
                "detect_min_bbox_width": getattr(follow_me, "detect_min_bbox_width", None),
                "steer_hold_grace_s": getattr(follow_me, "steer_hold_grace_s", None),
                "direct_turn_speed_knee_norm": getattr(follow_me, "direct_turn_speed_knee_norm", None),
                "direct_turn_speed_min_scale": getattr(follow_me, "direct_turn_speed_min_scale", None),
            },
            "vesc": {
                "max_erpm": getattr(vesc, "max_erpm", None),
                "rpm_plausibility_enabled": getattr(vesc, "rpm_plausibility_enabled", None),
                "rpm_plausibility_min_cmd_bytes": getattr(
                    vesc, "rpm_plausibility_min_cmd_bytes", None
                ),
                "rpm_plausibility_min_erpm": getattr(vesc, "rpm_plausibility_min_erpm", None),
                "rpm_plausibility_window_s": getattr(vesc, "rpm_plausibility_window_s", None),
                "rpm_plausibility_hold_s": getattr(vesc, "rpm_plausibility_hold_s", None),
            },
            "waypoint_nav": {
                "min_rtk_quality": getattr(waypoint_nav, "min_rtk_quality", None),
                "align_threshold_deg": getattr(waypoint_nav, "align_threshold_deg", None),
                "recovery_threshold_deg": getattr(waypoint_nav, "recovery_threshold_deg", None),
                "pivot_yaw_cmd": getattr(waypoint_nav, "pivot_yaw_cmd", None),
            },
            "gps_heading_align": {
                "max_lock_yaw_rate_dps": getattr(gps_heading_align, "max_lock_yaw_rate_dps", None),
            },
            "imu_source": getattr(config, "imu_source", None),
        },
    }


# ---------------------------------------------------------------------------
# Per-tick structured log object
# ---------------------------------------------------------------------------

def build_log_obj(
    *,
    now_ts: float,
    src: str,
    s,
    bt_override,
    bt_age,
    imu_status,
    telem: dict,
    oak_depth_stats,
    oak_persons,
    gps_reading,
    bms_state,
    bms_charging,
    recording_state,
    cmd,
    loop_dt_ms,
    imu_dt_ms,
    imu_motion_witness_still,
    events,
    oak_camera_health=None,
) -> dict:
    """Build the per-tick structured JSON log object.

    Pure/stdlib-only (duck-types every argument) so it's unit-testable
    without the hardware stack that ``pi_app.app.main`` imports at module
    load time. Callers pass already-resolved tick state: ``telem`` is
    ``controller.process()``'s telemetry dict, ``bms_state``/``bms_charging``
    are already pulled from the live ``BmsService``, ``recording_state`` is
    ``oak_recorder.recording_state`` (a string or None), and ``events`` is
    the list of ``SafetyEvent`` members fired this tick.

    ``imu_pipeline`` and ``oak_camera_health`` are NOT in this per-tick
    object (2026-09-19 logging audit, Commit C): both are slowly-changing
    diagnostics logged once/second in the separate "slow" line instead (see
    ``build_slow_obj``). The ``imu`` block here is filtered
    (``_filter_imu_status_for_log``, drops imu.oak_imu's re-exported
    imu_pipeline duplicates) and rounded to 3 decimals -- the slow line
    keeps full precision.
    """
    _oak = oak_camera_health if isinstance(oak_camera_health, dict) else {}
    _g = telem.get("gesture") if isinstance(telem.get("gesture"), dict) else {}
    _au = telem.get("arms_up") if isinstance(telem.get("arms_up"), dict) else {}
    return {
        "ts": round(now_ts, 3),
        "ts_iso": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
        "src": src,
        "mode": telem.get("mode", "MANUAL"),
        # Detection-stream gate (2026-09-25). None when this tick's
        # telemetry has no sample (gate inactive). vision_stale here is
        # the controller's per-tick sample; oak.vision_stale is the health copy.
        "vision_age_s": telem.get("vision_age_s"),
        "vision_stale": telem.get("vision_stale"),
        "follow_me_exit_reason": telem.get("follow_me_exit_reason"),
        "charger_inhibit": telem.get("charger_inhibit", False),
        "vesc_pack_low_latched": telem.get("vesc_pack_low_latched", False),
        # Armed-idle watchdog (2026-09-24 overnight-armed incident).
        "armed_idle_s": telem.get("armed_idle_s"),
        "rearm_requires_switch_cycle": telem.get("rearm_requires_switch_cycle"),
        "rc": to_int({"ch1": s.ch1_us, "ch2": s.ch2_us, "ch3": s.ch3_us, "ch4": s.ch4_us, "ch5": s.ch5_us}),
        "bt": to_int({"L": bt_override[0] if bt_override else None, "R": bt_override[1] if bt_override else None, "age_s": bt_age}),
        "imu": round_floats(_filter_imu_status_for_log(imu_status), 3) if imu_status else None,
        "imu_steering": {
            "steering_input": telem.get("steering_input"),
            "correction_raw": telem.get("imu_correction_raw"),
            "correction_applied": telem.get("imu_correction_applied"),
            "correction_blend": telem.get("correction_blend"),
            "speed_gain_scale": telem.get("speed_gain_scale"),
            "saturated": (imu_status or {}).get("saturated"),
        },
        "pid": round1({
            "error_deg": telem.get("pid_error_deg"),
            "p": telem.get("pid_p"),
            "i": telem.get("pid_i"),
            "d": telem.get("pid_d"),
            "correction": telem.get("pid_correction"),
            "integral_error": (imu_status or {}).get("integral_error"),
        }),
        "obstacle": round1({
            "distance_m": telem.get("obstacle_distance_m"),
            "throttle_scale": telem.get("obstacle_throttle_scale"),
            "depth_p5_mm": oak_depth_stats.p5_mm if oak_depth_stats else None,
            "depth_p50_mm": oak_depth_stats.p50_mm if oak_depth_stats else None,
            "depth_valid_pct": oak_depth_stats.valid_pixel_pct if oak_depth_stats else None,
            # The corridor's own valid fraction and pixel support (2026-09-19):
            # the full-frame figure above was 2-7 percent in low sun while the
            # corridor produced phantom near hits; this is what the reject path
            # and the support floor actually use (oak_depth._corridor_near_distance_mm).
            "corridor_valid_pct": getattr(oak_depth_stats, "corridor_valid_pct", None) if oak_depth_stats else None,
            "corridor_support_px": getattr(oak_depth_stats, "corridor_support_px", None) if oak_depth_stats else None,
            "corridor_near_px": getattr(oak_depth_stats, "corridor_near_px", None) if oak_depth_stats else None,
            "corridor_speckle_px": getattr(oak_depth_stats, "corridor_speckle_px", None) if oak_depth_stats else None,
        }),
        "follow_me": round1({
            "tracking": telem.get("follow_me_tracking"),
            "target_z_m": telem.get("follow_me_target_z_m"),
            "target_x_m": telem.get("follow_me_target_x_m"),
            "target_track_id": telem.get("follow_me_target_track_id"),
            "num_persons": telem.get("follow_me_num_detections"),
            "distance_error_m": telem.get("follow_me_distance_error_m"),
            "speed_offset": telem.get("follow_me_speed_offset"),
            "steer_offset": telem.get("follow_me_steer_offset"),
            "actual_speed_mps": telem.get("follow_me_actual_speed_mps"),
            "pursuit_mode": telem.get("follow_me_pursuit_mode"),
            "trail_length": telem.get("trail_length"),
            "trail_distance_m": telem.get("trail_distance_m"),
            "trail_rejected_jump_count": telem.get("trail_rejected_jump_count"),
            "trail_rejected_speed_count": telem.get("trail_rejected_speed_count"),
            "trail_lookahead_x": telem.get("trail_lookahead_x"),
            "trail_lookahead_y": telem.get("trail_lookahead_y"),
            "target_world_x": telem.get("follow_me_target_world_x"),
            "target_world_y": telem.get("follow_me_target_world_y"),
            "odom_x": telem.get("odom_x"),
            "odom_y": telem.get("odom_y"),
            "odom_theta_deg": telem.get("odom_theta_deg"),
            "odom_source": telem.get("odom_source"),
            "gps_speed_mps": telem.get("gps_speed_mps"),
            "confidence": telem.get("follow_me_target_confidence"),
            "num_detections": telem.get("follow_me_num_detections"),
            "steer_decay_factor": telem.get("follow_me_steer_decay_factor"),
            "fresh_detection": telem.get("follow_me_fresh_detection"),
            "steer_hold_active": telem.get("follow_me_steer_hold_active"),
            "depth_known": telem.get("follow_me_depth_known"),
            "depth_coast_s": telem.get("follow_me_depth_coast_s"),
            "tracker_reject": telem.get("follow_me_tracker_reject_reason"),
            "filter_rejects": telem.get("follow_me_filter_rejects"),
            "nearest_person_m": telem.get("follow_me_nearest_person_m"),
            "speed_depth_m": telem.get("follow_me_speed_depth_m"),
            "close_unknown": telem.get("follow_me_close_unknown"),
            # Speed-loop instrumentation (SpeedLayer + velocity PIDController):
            # open_loop_byte, target_mps, actual_mps, err_mps, p, i, d,
            # corr_mps, corr_byte, closed. See FollowMeController.get_status().
            "speed_loop": telem.get("speed_loop"),
            # Direct-pursuit forward scale from the fresh bbox x (2026-09-19);
            # 1.0 = no turn slowdown this tick.
            "turn_speed_scale": telem.get("turn_speed_scale"),
        }),
        "detections": [
            {"x_m": round(d.x_m, 2), "z_m": round(d.z_m, 2),
             "conf": round(d.confidence, 2),
             "bbox": [round(b, 3) for b in d.bbox],
             "track_id": getattr(d, "track_id", None),
             "depth_status": getattr(d, "depth_status", "ok"),
             "depth_valid_px": getattr(d, "depth_valid_px", -1),
             "depth_roi_px": getattr(d, "depth_roi_px", -1),
             "z_stereo_m": round(getattr(d, "z_stereo_m", 0.0), 2),
             "z_height_m": round(getattr(d, "z_height_m", 0.0), 2),
             "z_spread_m": round(getattr(d, "z_spread_m", 0.0), 2)}
            for d in oak_persons
        ] if oak_persons else None,
        "gps": {
            "lat": round(gps_reading.latitude, 8) if gps_reading else None,
            "lon": round(gps_reading.longitude, 8) if gps_reading else None,
            "alt_m": round(gps_reading.altitude_m, 1) if gps_reading else None,
            "fix": gps_reading.fix_quality if gps_reading else None,
            "sats": gps_reading.satellites_used if gps_reading else None,
            "hdop": round(gps_reading.hdop, 2) if gps_reading else None,
            "diff_age_s": round(gps_reading.diff_age_s, 1) if gps_reading else None,
            "station_id": gps_reading.station_id if gps_reading else None,
            # Sourced from controller telemetry (time.monotonic() -
            # reading.timestamp, both monotonic clocks -- computed here from
            # wall-clock now_ts would be wrong), but gated on THIS tick's
            # gps_reading like every other field above: the controller's
            # self._gps_reading can lag a tick behind (set_gps_reading() is
            # only called when gps_reader is not None), and a stale age_s
            # next to five None siblings would be misleading.
            "age_s": telem.get("gps_age_s") if gps_reading else None,
            # Commit D (2026-09-19): UTC/COG/SOG/mode/geoid separation, now
            # read from the receiver. cog_deg/sog_mps are logging only in
            # this commit -- not fed into the heading aligner or any control
            # path.
            "utc": gps_reading.utc_iso if gps_reading else None,
            "cog_deg": gps_reading.cog_deg if gps_reading else None,
            "sog_mps": gps_reading.sog_mps if gps_reading else None,
            "nmea_mode": gps_reading.nmea_mode if gps_reading else None,
            "geoid_sep_m": round(gps_reading.geoid_sep_m, 1) if gps_reading and gps_reading.geoid_sep_m is not None else None,
        },
        "waypoint_nav": round1({
            "wp_index": telem.get("wp_index"),
            "wp_total": telem.get("wp_total"),
            "wp_name": telem.get("wp_name"),
            "wp_bearing_deg": telem.get("wp_bearing_deg"),
            "wp_distance_m": telem.get("wp_distance_m"),
            "wp_heading_error_deg": telem.get("wp_heading_error_deg"),
            "wp_completed": telem.get("wp_completed"),
            "nav_state": telem.get("nav_state"),
            "wp_v_cmd": telem.get("wp_v_cmd"),
            "wp_yaw_cmd": telem.get("wp_yaw_cmd"),
            "wp_in_align": telem.get("wp_in_align"),
        }),
        "straight_intent": telem.get("straight_intent"),
        # heading_offset_deg/locked/frozen/refining dropped as flat top-level
        # keys (2026-09-19 logging audit): they duplicate heading_align's
        # offset_deg/locked/frozen/refining below exactly (same telemetry
        # values -- see controller.py's telemetry["heading_align"] block).
        "corrected_heading_deg": round1(telem.get("corrected_heading_deg")),
        "heading_align": round1(telem.get("heading_align") or {}),
        "recording_state": recording_state,
        "bms": {
            "voltage_v": bms_state.pack_voltage_v,
            "current_a": bms_state.pack_current_a,
            "soc_pct": bms_state.soc_pct,
            "cell_min_mv": bms_state.cell_min_mv,
            "cell_max_mv": bms_state.cell_max_mv,
            "cell_delta_mv": bms_state.cell_delta_mv,
            "temp_max_c": bms_state.temp_max_c,
            "charge_fet_on": bms_state.charge_fet_on,
            "discharge_fet_on": bms_state.discharge_fet_on,
            "bms_mode": getattr(bms_state, "bms_mode", None),
            "charger_connected": getattr(bms_state, "charger_connected", None),
            "load_connected": getattr(bms_state, "load_connected", None),
            "cycle_count": bms_state.cycle_count,
            "error_flags": bms_state.error_flags,
            "connected": bms_state.connected,
            "charging": bms_charging,
        } if bms_state is not None else None,
        "vesc": {
            "left_rpm": telem.get("vesc_left_rpm"),
            "right_rpm": telem.get("vesc_right_rpm"),
            "speed_mps": round(telem.get("vesc_actual_speed_mps"), 3)
                         if telem.get("vesc_actual_speed_mps") is not None else None,
            "rx_frame_count": telem.get("vesc_rx_frame_count"),
            "rx_parse_error_count": telem.get("vesc_rx_parse_error_count"),
            "rx_recv_error_count": telem.get("vesc_rx_recv_error_count"),
            "rx_reopen_count": telem.get("vesc_rx_reopen_count"),
            "rx_last_frame_age_s": (
                round(telem.get("vesc_rx_last_frame_age_s"), 3)
                if isinstance(telem.get("vesc_rx_last_frame_age_s"), (int, float))
                else None
            ),
            "rpm_plausible": telem.get("vesc_rpm_plausible"),
            "gate_trips": telem.get("vesc_rpm_gate_trips"),
            "l_temp_c": telem.get("vesc_left_temp_c"),
            "r_temp_c": telem.get("vesc_right_temp_c"),
            "l_motor_temp_c": telem.get("vesc_left_motor_temp_c"),
            "r_motor_temp_c": telem.get("vesc_right_motor_temp_c"),
            "l_duty": telem.get("vesc_left_duty"),
            "r_duty": telem.get("vesc_right_duty"),
        },
        "motor": to_int({"L": cmd.left_byte, "R": cmd.right_byte}),
        "safety": {"armed": cmd.is_armed, "emergency": cmd.emergency_active},
        "loop_dt_ms": loop_dt_ms,
        "imu_dt_ms": imu_dt_ms,
        "imu_motion_witness_still": imu_motion_witness_still,
        "events": [e.name for e in events] if events else [],
        "oak": {
            "det_fps": _oak.get("det_fps") if _oak else None,
            "depth_fps": _oak.get("depth_fps") if _oak else None,
            "det_latency_s": _oak.get("det_latency_s") if _oak else None,
            "vision_loop_hz": _oak.get("vision_loop_hz") if _oak else None,
            "vision_work_ms": _oak.get("vision_work_ms") if _oak else None,
            "hand_poll_ms": _oak.get("hand_poll_ms") if _oak else None,
            "nn_input_queue_size": _oak.get("nn_input_queue_size") if _oak else None,
            "hand_poll_enabled": _oak.get("hand_poll_enabled") if _oak else None,
            "det_fresh_age_s": _oak.get("det_fresh_age_s") if _oak else None,
            "det_seq": _oak.get("det_seq") if _oak else None,
            "det_seq_stuck_packets": _oak.get("det_seq_stuck_packets") if _oak else None,
            "vision_stale": _oak.get("vision_stale") if _oak else None,
        },
        "gesture": {
            "hand_detected": _g.get("hand_detected"),
            "hand_span_px": _g.get("hand_span_px"),
            "finger_count": _g.get("finger_count"),
            "label": _g.get("label"),
            "streak": _g.get("streak"),
            "phase": _g.get("phase"),
            "seq_idx": _g.get("seq_idx"),
            "event": _g.get("event"),
            "event_reason": _g.get("event_reason"),
            "hand_poll_enabled": _g.get("hand_poll_enabled"),
        },
        "arms_up": {
            "raw": _au.get("raw"),
            "active": _au.get("active"),
            "streak_s": _au.get("streak_s"),
            "sample_age_s": _au.get("sample_age_s"),
            "l_wrist_y": _au.get("l_wrist_y"),
            "r_wrist_y": _au.get("r_wrist_y"),
            "l_shoulder_y": _au.get("l_shoulder_y"),
            "r_shoulder_y": _au.get("r_shoulder_y"),
            "min_visibility_seen": _au.get("min_visibility_seen"),
            "twitch_active": _au.get("twitch_active"),
            "twitch_count": _au.get("twitch_count"),
            "twitch_blocked_reason": _au.get("twitch_blocked_reason"),
            "twitch_cancel_reason": _au.get("twitch_cancel_reason"),
            "test_latched": _au.get("test_latched"),
            "test_budget_left": _au.get("test_budget_left"),
            "test_expires_in_s": _au.get("test_expires_in_s"),
            "still_s": _au.get("still_s"),
            "crop_y0": _au.get("crop_y0"),
            "crop_y1": _au.get("crop_y1"),
            "pose_ms": _au.get("pose_ms"),
            "pose_hz": _au.get("pose_hz"),
            "pose_enabled": _au.get("pose_enabled"),
            "mp_pose_loaded": _au.get("mp_pose_loaded"),
        },
    }


# ---------------------------------------------------------------------------
# Slow line (1 Hz diagnostics, armed or not)
# ---------------------------------------------------------------------------

def should_write_slow_line(now: float, last_write_t: float, interval_s: float = 1.0) -> bool:
    """Gate for the 1 Hz "slow" diagnostics line -- unconditional on arm
    state (unlike should_log_tick), since imu_pipeline/oak_camera_health/
    chip temperature are worth knowing about even while disarmed."""
    return (now - last_write_t) >= interval_s


def build_slow_obj(
    *, now_ts: float, imu_pipeline, oak_camera_health, chip_temp_c, gps_health=None
) -> dict:
    """Build the 1 Hz "slow" diagnostics line.

    imu_pipeline and oak_camera_health are slowly-changing (their counters
    move over seconds, not ticks) and were previously logged at the full
    armed 10 Hz tick rate for no benefit -- imu_pipeline alone was ~1.5 KB/
    line. Both, plus the OAK chip temperature and (Commit D, 2026-09-19)
    RtkGpsReader.get_health(), now get one line per second regardless of
    arm state, at full float precision (unlike the per-tick "imu" block,
    which is rounded to 3 decimals).
    """
    _oak = oak_camera_health if isinstance(oak_camera_health, dict) else {}
    return {
        "type": "slow",
        "ts": round(now_ts, 3),
        "ts_iso": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
        "imu_pipeline": imu_pipeline,
        "oak_camera_health": oak_camera_health,
        "oak": {
            "chip_temp_c": chip_temp_c,
            "det_fps": _oak.get("det_fps"),
            "depth_fps": _oak.get("depth_fps"),
            "det_latency_s": _oak.get("det_latency_s"),
            "vision_loop_hz": _oak.get("vision_loop_hz"),
            "vision_work_ms": _oak.get("vision_work_ms"),
            "hand_poll_ms": _oak.get("hand_poll_ms"),
            "nn_input_queue_size": _oak.get("nn_input_queue_size"),
            "hand_poll_enabled": _oak.get("hand_poll_enabled"),
            "det_fresh_age_s": _oak.get("det_fresh_age_s"),
            "det_seq": _oak.get("det_seq"),
            "det_seq_stuck_packets": _oak.get("det_seq_stuck_packets"),
            "vision_stale": _oak.get("vision_stale"),
        },
        "gesture": {
            "hand_poll_enabled": _oak.get("hand_poll_enabled"),
            "hand_poll_ms": _oak.get("hand_poll_ms"),
            "mp_loaded": _oak.get("mp_loaded"),
            "hand_detect_rate": _oak.get("hand_detect_rate"),
        },
        "gps_health": gps_health,
    }
