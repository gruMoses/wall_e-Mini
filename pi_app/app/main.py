import time
import sys
import os
import json
import signal
import subprocess
from datetime import datetime, timedelta
from pathlib import Path
import fcntl
import argparse


def _sigterm_handler(signum, frame):
    raise KeyboardInterrupt

signal.signal(signal.SIGTERM, _sigterm_handler)

try:
    from pi_app.hardware.vesc import VescCanDriver
    from pi_app.hardware.arduino_modelx import ArduinoModelXDriver
    from pi_app.hardware.arduino_rc import ArduinoRCReader
    from pi_app.hardware.imu_reader import ImuReader
    from pi_app.hardware.oak_depth import OakDepthReader
    from pi_app.hardware.oak_recorder import OakRecorder, RecordingTelemetry
    from pi_app.hardware.rtk_gps import RtkGpsReader
    from pi_app.hardware.bms import BmsService
    from pi_app.web.oak_viewer import OakWebViewer
    from pi_app.control.controller import Controller, RCInputs
    from pi_app.control.safety import SafetyEvent
    from pi_app.control.imu_steering import ImuSteeringCompensator
    from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController
    from pi_app.control.follow_me import FollowMeController
    from pi_app.control.gesture_control import GestureStateMachine
    from pi_app.control.waypoint_nav import (
        WaypointNavController, WaypointNavConfig as WpNavCfg, load_waypoints,
    )
    from config import config
except ModuleNotFoundError:
    sys.path.append(str(Path(__file__).resolve().parents[2]))
    from pi_app.hardware.vesc import VescCanDriver  # type: ignore
    from pi_app.hardware.arduino_modelx import ArduinoModelXDriver  # type: ignore
    from pi_app.hardware.arduino_rc import ArduinoRCReader  # type: ignore
    from pi_app.hardware.imu_reader import ImuReader  # type: ignore
    from pi_app.hardware.oak_depth import OakDepthReader  # type: ignore
    from pi_app.hardware.oak_recorder import OakRecorder, RecordingTelemetry  # type: ignore
    from pi_app.hardware.rtk_gps import RtkGpsReader  # type: ignore
    from pi_app.hardware.bms import BmsService  # type: ignore
    from pi_app.web.oak_viewer import OakWebViewer  # type: ignore
    from pi_app.control.controller import Controller, RCInputs  # type: ignore
    from pi_app.control.safety import SafetyEvent  # type: ignore
    from pi_app.control.imu_steering import ImuSteeringCompensator  # type: ignore
    from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController  # type: ignore
    from pi_app.control.follow_me import FollowMeController  # type: ignore
    from pi_app.control.gesture_control import GestureStateMachine  # type: ignore
    from pi_app.control.waypoint_nav import (  # type: ignore
        WaypointNavController, WaypointNavConfig as WpNavCfg, load_waypoints,
    )
    from config import config  # type: ignore


def to_int(val):
    if isinstance(val, dict):
        return {k: to_int(v) for k, v in val.items()}
    if isinstance(val, list):
        return [to_int(v) for v in val]
    if isinstance(val, (int, float)) and not isinstance(val, bool):
        return int(round(val))
    return val


def round1(val):
    if isinstance(val, dict):
        return {k: round1(v) for k, v in val.items()}
    if isinstance(val, list):
        return [round1(v) for v in val]
    if isinstance(val, (int, float)) and not isinstance(val, bool):
        return round(float(val), 1)
    return val


_PID_CSV_COLS = (
    "t_ms,heading_deg,target_deg,error_deg,yaw_rate_dps,"
    "roll_deg,pitch_deg,p_term,i_term,d_term,"
    "correction_raw,correction_applied,integral_accum,"
    "steering_input,correction_blend,motor_l,motor_r,"
    "ch1_us,ch2_us,armed,straight_intent,loop_dt_ms,"
    "mode,fm_tracking,fm_target_z_m,fm_target_x_m,"
    "fm_dist_err_m,fm_speed_offset,fm_steer_offset,"
    "fm_num_det,fm_confidence,obstacle_dist_m,obstacle_scale"
)


def _open_log_file(logs_dir: Path, prefix: str):
    """Open a new structured JSON log file and update the latest.log symlink.
    Returns (file_handle, path) or (None, None) on failure."""
    dt = datetime.now()
    path = logs_dir / f"{prefix}_{dt.strftime('%Y%m%d_%H%M%S')}.log"
    fh = None
    try:
        fh = open(path, "a", encoding="utf-8", buffering=1)
        try:
            latest_link = logs_dir / "latest.log"
            if latest_link.exists() or latest_link.is_symlink():
                latest_link.unlink(missing_ok=True)
            os.symlink(path.name, latest_link)
        except Exception:
            pass
    except Exception:
        fh = None
        path = None
    return fh, path


def _open_pid_csv(logs_dir: Path):
    """Open a new PID CSV file and update the pid_latest.csv symlink.
    Returns (file_handle, path) or (None, None) on failure."""
    dt = datetime.now()
    path = logs_dir / f"pid_{dt.strftime('%Y%m%d_%H%M%S')}.csv"
    fh = None
    try:
        fh = open(path, "w", encoding="utf-8", buffering=1)
        fh.write(_PID_CSV_COLS + "\n")
        try:
            pid_latest = logs_dir / "pid_latest.csv"
            if pid_latest.exists() or pid_latest.is_symlink():
                pid_latest.unlink(missing_ok=True)
            os.symlink(path.name, pid_latest)
        except Exception:
            pass
    except Exception:
        fh = None
        path = None
    return fh, path


def _cleanup_old_logs(log_dir: Path, days: int = 7) -> None:
    try:
        cutoff = time.time() - days * 24 * 3600
        for p in log_dir.glob("run_*.log"):
            try:
                if p.stat().st_mtime < cutoff:
                    p.unlink()
            except Exception:
                pass
    except Exception:
        pass


def run() -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--pid-debug",
        action="store_true",
        help="Print PID controller debug values",
    )
    parser.add_argument(
        "--pid-csv",
        action="store_true",
        help="Write high-rate PID CSV (heavy disk I/O)",
    )
    parser.add_argument(
        "--disable-recording",
        action="store_true",
        help="Disable OAK recording pipeline for profiling/debug",
    )
    parser.add_argument(
        "--vesc-telemetry-debug",
        action="store_true",
        help="Print VESC RPM vs commanded speed every second",
    )
    args, _ = parser.parse_known_args()
    pid_debug = args.pid_debug or config.imu_steering.log_steering_corrections
    pid_csv_enabled = bool(args.pid_csv)
    disable_recording = bool(args.disable_recording)
    vesc_telem_debug = bool(args.vesc_telemetry_debug)

    rc_reader = ArduinoRCReader()
    port = rc_reader.start()
    print(f"Arduino RC detected on {port}")

    # IMU compensator will be initialized after OAK-D (oak_d source needs it running)
    imu_compensator = None

    # Initialize OAK-D Lite depth camera, derived controllers, and recorder
    oak_reader = None
    oak_recorder = None
    obstacle_ctrl = None
    follow_me_ctrl = None
    gesture_ctrl = None
    if config.obstacle_avoidance.enabled or config.follow_me.enabled:
        try:
            if OakDepthReader.detect():
                rec_cfg = (
                    config.oak_recording
                    if (config.oak_recording.enabled and not disable_recording)
                    else None
                )
                gesture_cfg = config.gesture if config.gesture.enabled else None
                oak_reader = OakDepthReader(
                    config.obstacle_avoidance, config.follow_me,
                    recording_config=rec_cfg,
                    gesture_config=gesture_cfg,
                    imu_poll_hz=float(getattr(config.imu_steering, "oak_imu_poll_hz", 60.0)),
                    imu_packet_mode=str(getattr(config.imu_steering, "oak_imu_packet_mode", "latest")),
                    imu_max_packets_per_poll=int(getattr(config.imu_steering, "oak_imu_max_packets_per_poll", 4)),
                    detection_config=getattr(config, "oak_detection", None),
                )
                oak_reader.start()
                print("OAK-D Lite detected and started")
                if config.obstacle_avoidance.enabled:
                    obstacle_ctrl = ObstacleAvoidanceController(config.obstacle_avoidance)
                    print("  Obstacle avoidance enabled")
                if config.follow_me.enabled:
                    follow_me_ctrl = FollowMeController(config.follow_me)
                    print("  Follow Me mode available (Ch4 switch to activate)")
                if gesture_cfg is not None:
                    gesture_ctrl = GestureStateMachine(
                        activation_sequence=gesture_cfg.activation_sequence,
                        stop_gesture=gesture_cfg.stop_gesture,
                        hold_frames=gesture_cfg.hold_frames,
                        sequence_timeout_s=gesture_cfg.sequence_timeout_s,
                        cooldown_s=gesture_cfg.cooldown_s,
                    )
                    print("  Hand-gesture Follow Me activation enabled (3-4-3 / palm stop)")
                if rec_cfg is not None:
                    try:
                        oak_recorder = OakRecorder(
                            config.oak_recording,
                            roi_vertical_offset_pct=getattr(config.obstacle_avoidance, "roi_vertical_offset_pct", 0.0),
                            obstacle_config=config.obstacle_avoidance,
                        )
                        oak_recorder.start(oak_reader)
                        print("  Activity-triggered recording enabled")
                    except Exception as e:
                        print(f"  Recording init failed: {e} — continuing without recording")
                        oak_recorder = None
            else:
                print("OAK-D Lite not detected — obstacle avoidance / Follow Me disabled")
        except Exception as e:
            print(f"OAK-D Lite initialization failed: {e}")
            print("  Continuing without depth camera")

    # Initialize IMU and steering compensator (after OAK-D so oak_d fallback is available).
    # Priority: external I2C IMU (best quality) > OAK-D onboard IMU > none.
    imu = None
    imu_source_cfg = getattr(config, "imu_source", "auto")
    if config.imu_steering.enabled and imu_source_cfg != "none":
        imu = None
        imu_source_used = None

        # Try external I2C IMU first (unless config explicitly says oak_d only)
        if imu_source_cfg in ("auto", "external"):
            try:
                print("Probing external I2C IMU...")
                imu = ImuReader(
                    calibration_path=config.imu_calibration_path,
                    mag_axis_map=getattr(config, "imu_mag_axis_map", None),
                    heading_cw_positive=getattr(config, "imu_heading_cw_positive", True),
                    use_magnetometer=config.imu_use_magnetometer,
                )
                imu_source_used = "external"
                print("  External I2C IMU detected")
            except Exception:
                if imu_source_cfg == "external":
                    print("  External IMU not found")
                else:
                    print("  External IMU not found, will try OAK-D IMU")

        # Fall back to OAK-D onboard IMU if external is not available
        if imu is None and imu_source_cfg in ("auto", "oak_d") and oak_reader is not None:
            try:
                from pi_app.hardware.oak_imu import OakImuReader
                print("Initializing OAK-D onboard IMU...")
                time.sleep(0.5)  # let the pipeline push a few IMU frames
                imu = OakImuReader(
                    oak_reader,
                    nmni_enabled=bool(getattr(config.imu_steering, "oak_nmni_enabled", False)),
                    nmni_threshold_dps=float(getattr(config.imu_steering, "oak_nmni_threshold_dps", 0.3)),
                    bias_adapt_enabled=bool(getattr(config.imu_steering, "oak_bias_adapt_enabled", False)),
                    bias_adapt_alpha=float(getattr(config.imu_steering, "oak_bias_adapt_alpha", 0.001)),
                    use_gravity_projected_yaw_rate=bool(
                        getattr(config.imu_steering, "oak_use_gravity_projected_yaw_rate", False)
                    ),
                )
                imu_source_used = "oak_d"
            except Exception as e:
                print(f"  OAK-D IMU initialization failed: {e}")

        if imu is not None:
            try:
                imu_compensator = ImuSteeringCompensator(config.imu_steering, imu)
                print(f"  IMU steering compensation enabled (source: {imu_source_used})")
            except Exception as e:
                print(f"  IMU steering init failed: {e}")
                if not config.imu_steering.fallback_on_error:
                    raise
                print("   Falling back to RC-only control")
        else:
            print("  No IMU available — steering compensation disabled")

    # Initialize RTK GPS and waypoint navigation
    gps_reader = None
    waypoint_nav_ctrl = None
    if config.gps.enabled:
        try:
            if RtkGpsReader.detect(config.gps.i2c_bus, config.gps.i2c_addr):
                gps_reader = RtkGpsReader(config.gps)
                gps_reader.start()
                print("RTK GPS detected and started")
                if config.waypoint_nav.enabled:
                    wp_cfg = WpNavCfg(
                        arrival_radius_m=config.waypoint_nav.arrival_radius_m,
                        cruise_speed_byte=config.waypoint_nav.cruise_speed_byte,
                        approach_speed_byte=config.waypoint_nav.approach_speed_byte,
                        slow_radius_m=config.waypoint_nav.slow_radius_m,
                        min_rtk_quality=config.waypoint_nav.min_rtk_quality,
                        stale_timeout_s=config.gps.stale_timeout_s,
                    )
                    wp_file = Path(__file__).resolve().parents[2] / config.waypoint_nav.waypoint_file
                    wps = []
                    if wp_file.exists():
                        try:
                            wps = load_waypoints(wp_file)
                            print(f"  Loaded {len(wps)} waypoints from {wp_file.name}")
                        except Exception as e:
                            print(f"  Failed to load waypoints: {e}")
                    else:
                        print(f"  No waypoint file ({wp_file.name}); nav available but empty")
                    waypoint_nav_ctrl = WaypointNavController(wp_cfg, wps)
                    print("  Waypoint navigation available")
            else:
                print("RTK GPS not detected — waypoint navigation disabled")
        except Exception as e:
            print(f"RTK GPS initialization failed: {e}")
            print("  Continuing without GPS")

    # Initialize Daly BMS Bluetooth service
    bms_service = None
    bms_cfg = getattr(config, "bms", None)
    if bms_cfg is not None and bms_cfg.enabled:
        try:
            bms_service = BmsService(bms_cfg)
            bms_service.start()
            print(f"Daly BMS service started (MAC: {bms_cfg.bms_mac_address})")
        except Exception as e:
            print(f"BMS service init failed: {e} — continuing without BMS")
            bms_service = None
    else:
        print("Daly BMS disabled (set config.bms.enabled=True and bms_mac_address to enable)")

    motor_driver = None
    if VescCanDriver.detect():
        print("VESC over CAN detected; using VESC driver")
        vesc_cfg = config.vesc
        motor_driver = VescCanDriver(
            left_id=vesc_cfg.left_can_id,
            right_id=vesc_cfg.right_can_id,
            max_rpm=vesc_cfg.max_erpm,
            vesc_cfg=vesc_cfg,
        )
        motor_driver.start()
        print(f"  VESC CAN telemetry RX started (voltage shutdown at {vesc_cfg.voltage_shutdown_threshold_v}V)")
    else:
        print("VESC not detected; using Arduino Model X motor driver (stub)")
        motor_driver = ArduinoModelXDriver(rc_reader=rc_reader)

    controller = Controller(
        motor_driver=motor_driver,
        imu_compensator=imu_compensator,
        obstacle_avoidance=obstacle_ctrl,
        follow_me=follow_me_ctrl,
        waypoint_nav=waypoint_nav_ctrl,
        gesture_controller=gesture_ctrl,
    )
    bt_server = None  # Set to None to indicate external SPP service is used

    # Start web viewer (needs recorder + controller)
    oak_web_viewer = None
    if config.oak_web_viewer.enabled and oak_recorder is not None:
        try:
            oak_web_viewer = OakWebViewer(
                config.oak_web_viewer, oak_recorder,
                controller=controller,
                oak_reader=oak_reader,
                motor_driver=motor_driver,
                imu_reader=imu,
            )
            oak_web_viewer.start()
            print(f"Web viewer at http://0.0.0.0:{config.oak_web_viewer.port}/")
        except Exception as e:
            print(f"Web viewer failed to start: {e}")

    # Debug trackers removed to simplify CLI view

    # Prepare structured logging directory and cleanup
    logs_dir = Path(__file__).resolve().parents[2] / "logs"
    try:
        logs_dir.mkdir(parents=True, exist_ok=True)
    except Exception:
        pass
    _cleanup_old_logs(logs_dir, days=7)

    # Open a per-run structured log file (e.g., run_20250821_132230.log)
    log_fh, log_path = _open_log_file(logs_dir, "run")

    # High-rate PID tuning CSV (optional; every loop tick, full precision)
    pid_csv_fh = None
    if pid_csv_enabled:
        pid_csv_fh, _ = _open_pid_csv(logs_dir)
    pid_csv_t0 = None

    try:
        last_log_ts = 0.0
        log_interval = 0.1  # 10 Hz logging
        prev_loop_ts = time.monotonic()
        _vesc_debug_last_t = 0.0
        prev_imu_ts = getattr(controller, "_last_imu_update", None)
        bt_shared_path = Path("/tmp/wall_e_bt_latest.json")
        bt_cached_data = None
        bt_cached_mtime_ns = None
        bt_next_poll_t = 0.0
        bt_poll_interval_s = 0.05
        imu_status = None
        oak_imu_metrics_getter = None
        oak_health_getter = None
        oak_prev_stale = None
        if oak_reader is not None:
            try:
                maybe_getter = getattr(oak_reader, "get_imu_metrics", None)
                if callable(maybe_getter):
                    oak_imu_metrics_getter = maybe_getter
            except Exception:
                oak_imu_metrics_getter = None
            try:
                maybe_health = getattr(oak_reader, "get_health", None)
                if callable(maybe_health):
                    oak_health_getter = maybe_health
            except Exception:
                oak_health_getter = None
        while True:
            loop_now = time.monotonic()
            loop_dt_ms = int(round((loop_now - prev_loop_ts) * 1000))
            prev_loop_ts = loop_now
            s = rc_reader.get_state()
            rc = RCInputs(
                ch1_us=s.ch1_us,
                ch2_us=s.ch2_us,
                ch3_us=s.ch3_us,
                ch4_us=s.ch4_us,
                ch5_us=s.ch5_us,
                last_update_epoch_s=s.last_update_epoch_s,
            )
            # Prefer fresh BT over RC using timestamp freshness
            # Read from shared file written by standalone SPP server
            bt_override = None
            bt_age = None
            try:
                if loop_now >= bt_next_poll_t:
                    bt_next_poll_t = loop_now + bt_poll_interval_s
                    st = bt_shared_path.stat()
                    mtime_ns = st.st_mtime_ns
                    if mtime_ns != bt_cached_mtime_ns:
                        with bt_shared_path.open("r", encoding="utf-8") as sf:
                            bt_cached_data = json.load(sf)
                        bt_cached_mtime_ns = mtime_ns
                if bt_cached_data is not None:
                    bt_age = time.time() - bt_cached_data["last_update_epoch_s"]
                    if bt_age <= 0.6:  # Fresh BT command within 600ms
                        bt_override = (bt_cached_data["left_byte"], bt_cached_data["right_byte"])
            except Exception:
                pass  # No BT data available, use RC instead
            # Feed OAK-D Lite data to controller
            oak_depth_stats = None
            oak_persons = []
            oak_camera_health = None
            if oak_reader is not None:
                dist_m, dist_age = oak_reader.get_min_distance()
                controller.set_obstacle_data(dist_m, dist_age)
                oak_depth_stats = oak_reader.get_depth_stats()
                if follow_me_ctrl is not None:
                    oak_persons = oak_reader.get_person_detections()
                    controller.set_person_detections(oak_persons)
                if gesture_ctrl is not None:
                    controller.set_hand_data(oak_reader.get_hand_data())
                if oak_health_getter is not None:
                    try:
                        oak_camera_health = oak_health_getter()
                    except Exception:
                        oak_camera_health = None
            # Feed GPS data to controller
            gps_reading = None
            if gps_reader is not None:
                gps_reading = gps_reader.get_reading()
                controller.set_gps_reading(gps_reading)
            # Update charger inhibit from BMS (fail-open: False when BMS unreachable)
            if bms_service is not None:
                controller.set_charger_inhibit(bms_service.is_charging())
            cmd, events, telem = controller.process(rc, bt_override_bytes=bt_override)

            # Start a new log file on each arm event for per-session analysis
            if any(e is SafetyEvent.ARMED for e in events):
                try:
                    if log_fh is not None:
                        log_fh.flush()
                        log_fh.close()
                except Exception:
                    pass
                log_fh, log_path = _open_log_file(logs_dir, "arm")
                if pid_csv_enabled and pid_csv_fh is not None:
                    try:
                        pid_csv_fh.flush()
                        pid_csv_fh.close()
                    except Exception:
                        pass
                    pid_csv_fh, _ = _open_pid_csv(logs_dir)
                    pid_csv_t0 = None  # Reset elapsed-time offset for new segment

            # Feed recorder (activity-triggered)
            if oak_recorder is not None:
                try:
                    need_rgb = oak_recorder.wants_rgb_preview()
                    need_depth = oak_recorder.wants_depth_preview()
                    if oak_reader is not None:
                        oak_reader.set_rgb_poll_enabled(need_rgb)
                    rec_telem = RecordingTelemetry(
                        timestamp=time.time(),
                        mode=telem.get("mode", "MANUAL"),
                        throttle_scale=telem.get("obstacle_throttle_scale", 1.0),
                        obstacle_distance_m=telem.get("obstacle_distance_m"),
                        motor_left=cmd.left_byte,
                        motor_right=cmd.right_byte,
                        is_armed=cmd.is_armed,
                        depth_stats=oak_depth_stats,
                        person_detections=oak_persons,
                        follow_tracking=telem.get("follow_me_tracking", False),
                        follow_target_x_m=telem.get("follow_me_target_x_m"),
                        follow_target_z_m=telem.get("follow_me_target_z_m"),
                        heading_deg=(imu_status or {}).get("heading_deg"),
                        yaw_rate_dps=(imu_status or {}).get("yaw_rate_dps"),
                        pursuit_mode=telem.get("follow_me_pursuit_mode"),
                        trail_length=telem.get("trail_length"),
                        trail_distance_m=telem.get("trail_distance_m"),
                        trail_rejected_jump_count=telem.get("trail_rejected_jump_count"),
                        trail_rejected_speed_count=telem.get("trail_rejected_speed_count"),
                        follow_target_world_x=telem.get("follow_me_target_world_x"),
                        follow_target_world_y=telem.get("follow_me_target_world_y"),
                        odom_x=telem.get("odom_x"),
                        odom_y=telem.get("odom_y"),
                        odom_theta_deg=telem.get("odom_theta_deg"),
                        speed_offset=telem.get("follow_me_speed_offset"),
                        steer_offset=telem.get("follow_me_steer_offset"),
                        distance_error_m=telem.get("follow_me_distance_error_m"),
                        gps_lat=gps_reading.latitude if gps_reading else None,
                        gps_lon=gps_reading.longitude if gps_reading else None,
                        gps_alt_m=gps_reading.altitude_m if gps_reading else None,
                        gps_fix=gps_reading.fix_quality if gps_reading else None,
                        gps_sats=gps_reading.satellites_used if gps_reading else None,
                        gps_hdop=gps_reading.hdop if gps_reading else None,
                        gps_diff_age_s=gps_reading.diff_age_s if gps_reading else None,
                        gps_station_id=gps_reading.station_id if gps_reading else None,
                    )
                    depth_frame = oak_reader.get_latest_depth_frame() if (oak_reader and need_depth) else None
                    rgb_frame = oak_reader.get_latest_rgb_frame() if (oak_reader and need_rgb) else None
                    oak_recorder.update(rec_telem, depth_frame=depth_frame, rgb_frame=rgb_frame)
                except Exception:
                    pass

            imu_dt_ms = None
            imu_update_ts = getattr(controller, "_last_imu_update", None)
            if (
                imu_update_ts is not None
                and prev_imu_ts is not None
                and imu_update_ts != prev_imu_ts
            ):
                imu_dt_ms = int(round((imu_update_ts - prev_imu_ts) * 1000))
            prev_imu_ts = imu_update_ts
            # Suppress BT extrema debug output to keep CLI clean
            # Get IMU status for display
            imu_status = controller.get_imu_status()
            # Populate IMU heading in recorder telemetry (for map overlay)
            if imu_status and imu_status.get('is_available'):
                rec_telem.imu_heading_deg = imu_status.get('heading_deg')
            # Prepare concise IMU info (heading → target)
            if imu_status and imu_status.get('is_available'):
                imu_info = f"IMU {imu_status['heading_deg']:.0f}°→{imu_status['target_heading_deg']:.0f}°"
            elif imu_status:
                imu_info = "IMU OFFLINE"
            else:
                imu_info = "IMU DISABLED"

            # Minimal console heartbeat with incoming RC info
            src = "BT" if bt_override is not None else "RC"
            # Show only RC/BT values, concise IMU, and IMU corrections
            corr_raw = telem.get("imu_correction_raw")
            corr_applied = telem.get("imu_correction_applied")
            corr_raw_str = f"{corr_raw:.0f}" if isinstance(corr_raw, (int, float)) else "-"
            corr_app_str = f"{corr_applied:.0f}" if isinstance(corr_applied, (int, float)) else "-"
            # Show BT values if available
            bt_display = "BT(external SPP)"
            if bt_override is not None:
                bt_display = f"BT(L={bt_override[0]:3d} R={bt_override[1]:3d})"

            # Obstacle / Follow Me info
            mode_str = telem.get("mode", "MANUAL")
            oa_scale = telem.get("obstacle_throttle_scale", 1.0)
            oa_dist = telem.get("obstacle_distance_m")
            oak_info = ""
            if oak_reader is not None:
                dist_str = f"{oa_dist:.2f}m" if oa_dist is not None else "?"
                oak_info = f"  OAK({dist_str} scl={oa_scale:.1f})"
                if isinstance(oak_camera_health, dict):
                    is_stale = bool(oak_camera_health.get("is_stale", False))
                    if oak_prev_stale is None:
                        oak_prev_stale = is_stale
                    elif oak_prev_stale != is_stale:
                        state = "STALE" if is_stale else "HEALTHY"
                        age_parts = []
                        for key in ("loop_age_s", "depth_age_s", "detections_age_s", "rgb_age_s"):
                            val = oak_camera_health.get(key)
                            if isinstance(val, (int, float)):
                                age_parts.append(f"{key}={val:.2f}")
                        print(f"\nOAK camera health transition -> {state} ({', '.join(age_parts)})")
                        if is_stale:
                            err_keys = (
                                "last_pipeline_error",
                                "last_depth_error",
                                "last_detection_error",
                                "last_rgb_error",
                                "last_imu_error",
                            )
                            errs = [oak_camera_health.get(k) for k in err_keys if oak_camera_health.get(k)]
                            if errs:
                                print(f"OAK last errors: {' | '.join(str(e) for e in errs)}")
                        oak_prev_stale = is_stale
            mode_info = f"  [{mode_str}]" if mode_str != "MANUAL" else ""

            gps_info = ""
            if gps_reader is not None:
                if gps_reading is not None:
                    gps_info = f"  GPS(q{gps_reading.fix_quality} sat={gps_reading.satellites_used} dif={gps_reading.diff_age_s:.1f}s)"
                else:
                    gps_info = "  GPS(no fix)"

            line_cli = (
                f"{src} RC(ch1={s.ch1_us:4d} ch2={s.ch2_us:4d} ch3={s.ch3_us:4d} ch4={s.ch4_us:4d} ch5={s.ch5_us:4d}) "
                f"{bt_display}  "
                f"{imu_info}  corr_raw={corr_raw_str} corr_applied={corr_app_str}  "
                f"armed={'Y' if cmd.is_armed else 'N'} ev={len(events)}"
                f"{oak_info}{gps_info}{mode_info}   "
            )
            if pid_debug:
                print(line_cli)
                pid_line = (
                    f"PID err={telem.get('pid_error_deg', 0.0):.1f} "
                    f"P={telem.get('pid_p', 0.0):.1f} "
                    f"I={telem.get('pid_i', 0.0):.1f} "
                    f"D={telem.get('pid_d', 0.0):.1f} "
                    f"yaw={(imu_status or {}).get('yaw_rate_dps', 0.0):.1f} "
                    f"int={(imu_status or {}).get('integral_error', 0.0):.1f}"
                )
                print(pid_line)
            else:
                print(line_cli, end="\r", flush=True)

            if vesc_telem_debug and (loop_now - _vesc_debug_last_t) >= 1.0:
                _vesc_debug_last_t = loop_now
                _vt = controller.get_vesc_telemetry()
                _l_rpm = _vt.get("left_rpm")
                _r_rpm = _vt.get("right_rpm")
                _spd = _vt.get("actual_speed_mps")
                _spd_str = f"{_spd:.3f} m/s" if _spd is not None else "n/a"
                _slip = telem.get("follow_me_slip_active", False)
                print(
                    f"\n[VESC] L={_l_rpm} RPM  R={_r_rpm} RPM  "
                    f"speed={_spd_str}  "
                    f"cmd_L={telem.get('motor_left_byte')} cmd_R={telem.get('motor_right_byte')}"
                    f"{'  SLIP!' if _slip else ''}"
                )

            # Structured JSON log for analysis (one line per tick)
            try:
                now_ts = time.time()
                if now_ts - last_log_ts >= log_interval:
                    last_log_ts = now_ts
                    imu_pipeline = None
                    if oak_reader is not None:
                        imu_pipeline = {"metrics_available": False}
                        if oak_imu_metrics_getter is not None:
                            try:
                                imu_metrics = oak_imu_metrics_getter()
                                if isinstance(imu_metrics, dict):
                                    imu_pipeline["metrics_available"] = True
                                    # Keep IMU pipeline timing metrics at full precision.
                                    imu_pipeline.update(imu_metrics)
                                elif hasattr(imu_metrics, "__dict__"):
                                    imu_pipeline["metrics_available"] = True
                                    # Keep IMU pipeline timing metrics at full precision.
                                    imu_pipeline.update(vars(imu_metrics))
                            except Exception:
                                pass
                    log_obj = {
                        "ts": round(now_ts, 3),
                        "ts_iso": datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                        "src": src,
                        "mode": telem.get("mode", "MANUAL"),
                        "rc": to_int({"ch1": s.ch1_us, "ch2": s.ch2_us, "ch3": s.ch3_us, "ch4": s.ch4_us, "ch5": s.ch5_us}),
                        "bt": to_int({"L": bt_override[0] if bt_override else None, "R": bt_override[1] if bt_override else None, "age_s": bt_age}),
                        "imu": imu_status if imu_status else None,
                        "imu_steering": {
                            "steering_input": telem.get("steering_input"),
                            "correction_raw": telem.get("imu_correction_raw"),
                            "correction_applied": telem.get("imu_correction_applied"),
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
                        }),
                        "detections": [
                            {"x_m": round(d.x_m, 2), "z_m": round(d.z_m, 2),
                             "conf": round(d.confidence, 2),
                             "bbox": [round(b, 3) for b in d.bbox]}
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
                        },
                        "waypoint_nav": round1({
                            "wp_index": telem.get("wp_index"),
                            "wp_total": telem.get("wp_total"),
                            "wp_name": telem.get("wp_name"),
                            "wp_bearing_deg": telem.get("wp_bearing_deg"),
                            "wp_distance_m": telem.get("wp_distance_m"),
                            "wp_completed": telem.get("wp_completed"),
                        }),
                        "recording_state": oak_recorder.recording_state if oak_recorder is not None else None,
                        "bms": (lambda s: {
                            "voltage_v": s.pack_voltage_v,
                            "current_a": s.pack_current_a,
                            "soc_pct": s.soc_pct,
                            "cell_min_mv": s.cell_min_mv,
                            "cell_max_mv": s.cell_max_mv,
                            "cell_delta_mv": s.cell_delta_mv,
                            "temp_max_c": s.temp_max_c,
                            "charge_fet_on": s.charge_fet_on,
                            "discharge_fet_on": s.discharge_fet_on,
                            "cycle_count": s.cycle_count,
                            "error_flags": s.error_flags,
                            "connected": s.connected,
                            "charging": bms_service.is_charging(),
                        })(bms_service.get_state()) if bms_service is not None else None,
                        "motor": to_int({"L": cmd.left_byte, "R": cmd.right_byte}),
                        "safety": {"armed": cmd.is_armed, "emergency": cmd.emergency_active},
                        "loop_dt_ms": loop_dt_ms,
                        "imu_dt_ms": imu_dt_ms,
                        "imu_pipeline": imu_pipeline,
                        "oak_camera_health": oak_camera_health,
                        "events": [e.name for e in events] if events else [],
                    }
                    line = json.dumps(log_obj)
                    # Do not print structured JSON to console; keep file logging only
                    if log_fh is not None:
                        try:
                            log_fh.write(line + "\n")
                        except Exception as e:
                            # Print the error once to avoid flooding
                            try:
                                if '_log_write_error_reported' not in locals() or not _log_write_error_reported:
                                    print(f"Log write error: {e}")
                                    _log_write_error_reported = True
                            except Exception:
                                pass
            except Exception:
                pass

            # High-rate PID CSV row (every tick, no throttling)
            try:
                if pid_csv_fh is not None:
                    if pid_csv_t0 is None:
                        pid_csv_t0 = loop_now
                    t_ms = int(round((loop_now - pid_csv_t0) * 1000))
                    _imu = imu_status or {}
                    pid_csv_fh.write(
                        f"{t_ms},"
                        f"{_imu.get('heading_deg', 0.0):.2f},"
                        f"{_imu.get('target_heading_deg', 0.0):.2f},"
                        f"{telem.get('pid_error_deg', 0.0):.3f},"
                        f"{_imu.get('yaw_rate_dps', 0.0):.2f},"
                        f"{_imu.get('roll_deg', 0.0):.2f},"
                        f"{_imu.get('pitch_deg', 0.0):.2f},"
                        f"{telem.get('pid_p', 0.0):.3f},"
                        f"{telem.get('pid_i', 0.0):.3f},"
                        f"{telem.get('pid_d', 0.0):.3f},"
                        f"{telem.get('imu_correction_raw', '')!s},"
                        f"{telem.get('imu_correction_applied', '')!s},"
                        f"{_imu.get('integral_error', 0.0):.3f},"
                        f"{telem.get('steering_input', 0.0):.4f},"
                        f"{telem.get('correction_blend', '')!s},"
                        f"{cmd.left_byte},{cmd.right_byte},"
                        f"{s.ch1_us},{s.ch2_us},"
                        f"{1 if cmd.is_armed else 0},"
                        f"{1 if telem.get('straight_intent') else 0},"
                        f"{loop_dt_ms},"
                        f"{telem.get('mode', 'MANUAL')},"
                        f"{1 if telem.get('follow_me_tracking') else 0},"
                        f"{telem.get('follow_me_target_z_m', '')!s},"
                        f"{telem.get('follow_me_target_x_m', '')!s},"
                        f"{telem.get('follow_me_distance_error_m', '')!s},"
                        f"{telem.get('follow_me_speed_offset', '')!s},"
                        f"{telem.get('follow_me_steer_offset', '')!s},"
                        f"{telem.get('follow_me_num_detections', 0)},"
                        f"{telem.get('follow_me_target_confidence', '')!s},"
                        f"{telem.get('obstacle_distance_m', '')!s},"
                        f"{telem.get('obstacle_throttle_scale', '')!s}\n"
                    )
            except Exception:
                pass

            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        try:
            if motor_driver is not None:
                motor_driver.stop()
        except Exception:
            pass
        try:
            if oak_recorder is not None:
                oak_recorder.stop()
        except Exception:
            pass
        try:
            if oak_reader is not None:
                oak_reader.stop()
        except Exception:
            pass
        try:
            if gps_reader is not None:
                gps_reader.stop()
        except Exception:
            pass
        try:
            if bms_service is not None:
                bms_service.stop()
        except Exception:
            pass
        try:
            if bt_server is not None:  # Only stop if bt_server was created
                bt_server.stop()
        except Exception:
            pass
        rc_reader.stop()
        try:
            if pid_csv_fh is not None:
                pid_csv_fh.flush()
                pid_csv_fh.close()
        except Exception:
            pass
        try:
            if log_fh is not None:
                log_fh.flush()
                log_fh.close()
        except Exception:
            pass
        # On exit, run compact PID analyzer if debugging was enabled
        try:
            if 'pid_debug' in locals() and pid_debug:
                tools_analyzer = Path(__file__).resolve().parents[2] / "tools" / "pid_log_analyzer.py"
                if tools_analyzer.exists():
                    # Prefer the exact run log we just wrote
                    target_log = log_path if 'log_path' in locals() else (Path(__file__).resolve().parents[2] / "logs" / "latest.log")
                    if target_log and target_log.exists():
                        subprocess.run(
                            [sys.executable, str(tools_analyzer), "--log", str(target_log)],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                            check=False,
                            timeout=15,
                        )
        except Exception:
            # Never let analysis interfere with shutdown
            pass


if __name__ == "__main__":
    # Singleton lock to prevent multiple instances
    lock_file = Path("/tmp/wall_e_mini_main.lock")
    _lock_fh = None
    try:
        _lock_fh = open(lock_file, "w")
        fcntl.flock(_lock_fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            _lock_fh.truncate(0)
            _lock_fh.write(str(os.getpid()))
            _lock_fh.flush()
        except Exception:
            pass
    except BlockingIOError:
        print("Another WALL-E Mini control app instance is already running. Exiting.")
        sys.exit(1)
    except Exception:
        # If locking fails for unexpected reasons, proceed without blocking to avoid false negatives
        _lock_fh = None

    try:
        run()
    finally:
        try:
            if _lock_fh is not None:
                fcntl.flock(_lock_fh, fcntl.LOCK_UN)
                _lock_fh.close()
        except Exception:
            pass
