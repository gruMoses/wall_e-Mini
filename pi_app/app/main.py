import time
import sys
import os
import json
import subprocess
from datetime import datetime, timedelta
from pathlib import Path
import fcntl
import argparse

try:
    from pi_app.hardware.vesc import VescCanDriver
    from pi_app.hardware.arduino_modelx import ArduinoModelXDriver
    from pi_app.hardware.arduino_rc import ArduinoRCReader
    from pi_app.hardware.imu_reader import ImuReader
    from pi_app.control.controller import Controller, RCInputs
    from pi_app.control.imu_steering import ImuSteeringCompensator
    from pi_app.io.bt_input import BtCommandServer
    from config import config
except ModuleNotFoundError:
    # Allow running as a script: python3 /home/pi/pi_app/app/main.py
    sys.path.append(str(Path(__file__).resolve().parents[2]))
    from pi_app.hardware.vesc import VescCanDriver  # type: ignore
    from pi_app.hardware.arduino_modelx import ArduinoModelXDriver  # type: ignore
    from pi_app.hardware.arduino_rc import ArduinoRCReader  # type: ignore
    from pi_app.hardware.imu_reader import ImuReader  # type: ignore
    from pi_app.control.controller import Controller, RCInputs  # type: ignore
    from pi_app.control.imu_steering import ImuSteeringCompensator  # type: ignore
    from pi_app.io.bt_input import BtCommandServer  # type: ignore
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
    args, _ = parser.parse_known_args()
    pid_debug = args.pid_debug or config.log_steering_corrections

    rc_reader = ArduinoRCReader()
    port = rc_reader.start()
    print(f"Arduino RC detected on {port}")

    # Initialize IMU and steering compensator
    imu_compensator = None
    if config.imu_steering.enabled:
        try:
            print("Initializing IMU...")
            imu = ImuReader(calibration_path=config.imu_calibration_path,
                            mag_axis_map=getattr(config, 'imu_mag_axis_map', None),
                            heading_cw_positive=getattr(config, 'imu_heading_cw_positive', True))
            imu_compensator = ImuSteeringCompensator(config.imu_steering, imu)
            print("✓ IMU steering compensation enabled")
        except Exception as e:
            print(f"⚠️  IMU initialization failed: {e}")
            if config.imu_steering.fallback_on_error:
                print("   Falling back to RC-only control")
            else:
                raise

    motor_driver = None
    if VescCanDriver.detect():
        print("VESC over CAN detected; using VESC driver")
        motor_driver = VescCanDriver()
    else:
        print("VESC not detected; using Arduino Model X motor driver (stub)")
        motor_driver = ArduinoModelXDriver(rc_reader=rc_reader)

    controller = Controller(motor_driver=motor_driver, imu_compensator=imu_compensator)
    bt_server = BtCommandServer()
    bt_server.start()

    # Debug trackers removed to simplify CLI view

    # Prepare structured logging directory and cleanup
    logs_dir = Path(__file__).resolve().parents[2] / "logs"
    try:
        logs_dir.mkdir(parents=True, exist_ok=True)
    except Exception:
        pass
    _cleanup_old_logs(logs_dir, days=7)

    # Open a per-run structured log file (e.g., run_20250821_132230.log)
    run_dt = datetime.now()
    log_filename = f"run_{run_dt.strftime('%Y%m%d_%H%M%S')}.log"
    log_path = logs_dir / log_filename
    log_fh = None
    try:
        log_fh = open(log_path, "a", encoding="utf-8", buffering=1)
        # Update handy latest.log symlink
        try:
            latest_link = logs_dir / "latest.log"
            if latest_link.exists() or latest_link.is_symlink():
                latest_link.unlink(missing_ok=True)
            os.symlink(log_path.name, latest_link)
        except Exception:
            pass
    except Exception:
        # If file cannot be opened, continue without file logging
        log_fh = None

    try:
        last_log_ts = 0.0
        log_interval = 0.1  # 10 Hz logging
        prev_loop_ts = time.monotonic()
        prev_imu_ts = getattr(controller, "_last_imu_update", None)
        while True:
            loop_now = time.monotonic()
            loop_dt_ms = int(round((loop_now - prev_loop_ts) * 1000))
            prev_loop_ts = loop_now
            s = rc_reader.get_state()
            rc = RCInputs(
                ch1_us=s.ch1_us,
                ch2_us=s.ch2_us,
                ch3_us=s.ch3_us,
                ch5_us=s.ch5_us,
                last_update_epoch_s=s.last_update_epoch_s,
            )
            # Prefer fresh BT over RC using timestamp freshness
            bt = bt_server.get_latest_bytes()
            bt_age = time.time() - bt.last_update_epoch_s
            bt_override = (bt.left_byte, bt.right_byte) if bt_age <= 0.6 else None
            cmd, events, telem = controller.process(rc, bt_override_bytes=bt_override)
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
            line_cli = (
                f"{src} RC(ch1={s.ch1_us:4d} ch2={s.ch2_us:4d} ch3={s.ch3_us:4d} ch5={s.ch5_us:4d}) "
                f"BT(L={bt.left_byte:3d} R={bt.right_byte:3d})  "
                f"{imu_info}  corr_raw={corr_raw_str} corr_applied={corr_app_str}   "
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

            # Structured JSON log for analysis (one line per tick) - only when armed
            try:
                now_ts = time.time()
                if now_ts - last_log_ts >= log_interval and cmd.is_armed:
                    last_log_ts = now_ts
                    log_obj = {
                        "ts": to_int(now_ts),
                        "ts_iso": datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                        "src": src,
                        "rc": to_int({"ch1": s.ch1_us, "ch2": s.ch2_us, "ch3": s.ch3_us, "ch5": s.ch5_us}),
                        "bt": to_int({"L": bt.left_byte, "R": bt.right_byte, "age_s": bt_age}),
                        "imu": to_int(imu_status) if imu_status else None,
                        "imu_steering": to_int({
                            "steering_input": telem.get("steering_input"),
                            "correction_raw": telem.get("imu_correction_raw"),
                            "correction_applied": telem.get("imu_correction_applied"),
                        }),
                        "pid": round1({
                            "error_deg": telem.get("pid_error_deg"),
                            "p": telem.get("pid_p"),
                            "i": telem.get("pid_i"),
                            "d": telem.get("pid_d"),
                            "correction": telem.get("pid_correction"),
                            "integral_error": (imu_status or {}).get("integral_error"),
                        }),
                        "motor": to_int({"L": cmd.left_byte, "R": cmd.right_byte}),
                        "safety": {"armed": cmd.is_armed, "emergency": cmd.emergency_active},
                        "loop_dt_ms": loop_dt_ms,
                        "imu_dt_ms": imu_dt_ms,
                        "events": [e.name for e in events] if events else [],
                    }
                    line = json.dumps(log_obj)
                    # Do not print structured JSON to console; keep file logging only
                    if log_fh is not None:
                        try:
                            log_fh.write(line + "\n")
                        except Exception:
                            pass
            except Exception:
                pass
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        try:
            bt_server.stop()
        except Exception:
            pass
        rc_reader.stop()
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
