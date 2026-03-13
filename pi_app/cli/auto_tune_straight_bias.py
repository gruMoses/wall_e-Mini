import argparse
import json
import time
from pathlib import Path
from typing import Optional, Tuple, List

try:
    # Normal imports when running inside project
    from pi_app.hardware.imu_reader import ImuReader
    from pi_app.hardware.arduino_rc import ArduinoRCReader
    from pi_app.hardware.arduino_modelx import ArduinoModelXDriver
    from pi_app.hardware.vesc import VescCanDriver
    from config import config
except Exception:
    import sys
    from pathlib import Path as _Path
    sys.path.append(str(_Path(__file__).resolve().parents[2]))
    from pi_app.hardware.imu_reader import ImuReader  # type: ignore
    from pi_app.hardware.arduino_rc import ArduinoRCReader  # type: ignore
    from pi_app.hardware.arduino_modelx import ArduinoModelXDriver  # type: ignore
    from pi_app.hardware.vesc import VescCanDriver  # type: ignore
    from config import config  # type: ignore


from pi_app.control.mapping import CENTER_OUTPUT_VALUE as CENTER_BYTE


def wait_for_arm(rc: ArduinoRCReader, high_threshold_us: int, debounce_s: float) -> None:
    print(f"Waiting for ARM (ch3 >= {high_threshold_us} µs for {debounce_s:.1f}s)...")
    high_since: Optional[float] = None
    while True:
        s = rc.get_state()
        now = time.monotonic()
        if s.ch3_us >= high_threshold_us:
            if high_since is None:
                high_since = now
            if (now - high_since) >= debounce_s:
                print("ARM detected.")
                return
        else:
            high_since = None
        time.sleep(0.02)


def collect_yaw_rate(imu: ImuReader, rc: ArduinoRCReader, duration_s: float, arm_required: bool) -> float:
    """Collect average yaw rate (deg/s) over armed time within duration window.

    If disarmed periods occur, they are skipped (not counted).
    Returns the average over armed samples; if no armed samples, returns 0.0.
    """
    end_t = time.monotonic() + duration_s
    samples: List[float] = []
    while time.monotonic() < end_t:
        s = rc.get_state()
        if not arm_required or s.ch3_us >= 1800:
            data = imu.read()
            samples.append(float(data['gz_dps']))
        time.sleep(0.02)
    if not samples:
        return 0.0
    return sum(samples) / len(samples)


def run_bin(
    imu: ImuReader,
    rc: ArduinoRCReader,
    motor,
    base_offset: int,
    test_delta: int,
    seg_duration_s: float,
    max_bias_abs: int,
) -> Tuple[int, float, float, float, int]:
    """Run two segments at a given throttle bin and compute suggested bias.

    Returns (base_byte, yaw1, yaw2, suggested_delta, suggested_delta_clamped)
    where suggested_delta is the left-minus-right half difference in bytes to apply during straight intent.
    """
    base = CENTER_BYTE + int(base_offset)
    base = max(0, min(254, base))

    # Segment 1: equal tracks
    left = base
    right = base
    motor.set_tracks(left, right)
    yaw1 = collect_yaw_rate(imu, rc, seg_duration_s, arm_required=True)

    # Segment 2: introduce known delta
    left = max(0, min(254, base + test_delta))
    right = max(0, min(254, base - test_delta))
    motor.set_tracks(left, right)
    yaw2 = collect_yaw_rate(imu, rc, seg_duration_s, arm_required=True)

    # Stop motion between bins
    motor.set_tracks(CENTER_BYTE, CENTER_BYTE)

    # Compute slope: dyaw/d(delta_effective)
    d_eff = 2.0 * float(test_delta)
    slope = (yaw2 - yaw1) / d_eff if d_eff != 0 else 0.0
    # delta_needed such that yaw ~ 0 => yaw1 + slope * (2*delta_needed) = 0
    delta_needed = 0.0
    if abs(slope) > 1e-6:
        delta_needed = -yaw1 / (2.0 * slope)
    # Clamp
    delta_needed_clamped = int(max(-max_bias_abs, min(max_bias_abs, round(delta_needed))))
    return base, yaw1, yaw2, float(delta_needed), int(delta_needed_clamped)


def apply_bias_to_config(config_path: Path, bias_left: int, bias_right: int) -> bool:
    """In-place edit of config.py straight_bias values. Returns True on success."""
    try:
        text = config_path.read_text()
    except Exception:
        return False

    import re
    # Replace values while preserving whitespace/comments on the lines
    def repl_line(pattern: str, new_value: int, s: str) -> str:
        return re.sub(
            pattern,
            lambda m: f"{m.group(1)}{new_value}{m.group(3)}",
            s,
            flags=re.MULTILINE,
        )

    left_pat = r"^(\s*straight_bias_left_byte:\s*int\s*=\s*)(-?\d+)(\s*)$"
    right_pat = r"^(\s*straight_bias_right_byte:\s*int\s*=\s*)(-?\d+)(\s*)$"
    new_text = repl_line(left_pat, bias_left, text)
    new_text = repl_line(right_pat, bias_right, new_text)
    if new_text == text:
        return False
    try:
        config_path.write_text(new_text)
        return True
    except Exception:
        return False


def main() -> None:
    ap = argparse.ArgumentParser(description="Guided auto-tune of straight bias (left/right bytes)")
    ap.add_argument("--bins", default="20,40,60", help="Throttle offsets from center (bytes), comma-separated")
    ap.add_argument("--seg", type=float, default=3.0, help="Seconds per segment (equal and test)")
    ap.add_argument("--test-delta", type=int, default=8, help="Test delta bytes applied in second segment")
    ap.add_argument("--max-bias", type=int, default=15, help="Clamp for suggested bias magnitude (bytes)")
    ap.add_argument("--apply", action="store_true", help="Apply the aggregated bias to config.py")
    args = ap.parse_args()

    bins = [int(x.strip()) for x in args.bins.split(",") if x.strip()]
    logs_dir = Path(__file__).resolve().parents[2] / "logs"
    try:
        logs_dir.mkdir(parents=True, exist_ok=True)
    except Exception:
        pass

    imu = ImuReader(calibration_path=config.imu_calibration_path,
                    use_magnetometer=config.imu_use_magnetometer)
    rc = ArduinoRCReader()
    try:
        port = rc.start()
        print(f"Arduino RC detected on {port}")
    except Exception as e:
        print(f"Failed to start Arduino RC reader: {e}")
        return

    if VescCanDriver.detect():
        print("VESC over CAN detected; using VESC driver")
        motor = VescCanDriver(
            left_id=2,
            right_id=1,
            max_rpm=getattr(config, "vesc", None).max_erpm if hasattr(config, "vesc") else 15000,
        )
    else:
        print("VESC not detected; using Arduino Model X motor driver (via Arduino)")
        motor = ArduinoModelXDriver(rc_reader=rc)

    # Ensure neutral at start
    motor.set_tracks(CENTER_BYTE, CENTER_BYTE)

    # Wait for ARM
    wait_for_arm(rc, high_threshold_us=1800, debounce_s=0.3)

    results = []
    for off in bins:
        print(f"\n[Bin offset +{off}] Running equal and test-delta segments...")
        base, y1, y2, delta, delta_clamped = run_bin(
            imu=imu,
            rc=rc,
            motor=motor,
            base_offset=off,
            test_delta=int(args.test_delta),
            seg_duration_s=float(args.seg),
            max_bias_abs=int(args.max_bias),
        )
        print(
            f"  base={base}  yaw_equal={y1:.3f} dps  yaw_test={y2:.3f} dps  suggested_delta={delta:.2f} -> clamp {delta_clamped}"
        )
        results.append({
            "offset": int(off),
            "base": int(base),
            "yaw_equal_dps": float(y1),
            "yaw_test_dps": float(y2),
            "delta_suggest": float(delta),
            "delta_suggest_clamped": int(delta_clamped),
        })

    # Neutral at end
    motor.set_tracks(CENTER_BYTE, CENTER_BYTE)
    try:
        motor.stop()
    except Exception:
        pass

    # Aggregate suggestion: median of clamped deltas across bins
    deltas = [int(r["delta_suggest_clamped"]) for r in results if isinstance(r.get("delta_suggest_clamped"), int)]
    if not deltas:
        print("No valid suggestions computed.")
        return
    deltas_sorted = sorted(deltas)
    agg_delta = deltas_sorted[len(deltas_sorted) // 2]
    bias_left = int(agg_delta)
    bias_right = int(-agg_delta)

    print("\nSuggested straight bias (apply to config.py):")
    print(f"  straight_bias_left_byte  = {bias_left}")
    print(f"  straight_bias_right_byte = {bias_right}")

    # Save run details
    out_json = logs_dir / f"bias_tune_{int(time.time())}.json"
    try:
        out_json.write_text(json.dumps({
            "bins": bins,
            "results": results,
            "aggregate": {"delta": agg_delta, "bias_left": bias_left, "bias_right": bias_right},
        }, indent=2))
        print(f"Wrote details to {out_json}")
    except Exception:
        pass

    if args.apply:
        cfg_path = Path(__file__).resolve().parents[2] / "config.py"
        ok = apply_bias_to_config(cfg_path, bias_left=bias_left, bias_right=bias_right)
        if ok:
            print(f"Applied to {cfg_path}")
        else:
            print("Failed to update config.py (pattern not found or write error)")


if __name__ == "__main__":
    main()


