#!/usr/bin/env python3
"""One-shot forward calibration run with hard safety guards.

Behavior:
- Waits for a *new* arm event (disarmed -> armed transition) after script starts.
- On arm, commands equal forward motor bytes for a fixed duration.
- If disarm is detected at any point, immediately commands neutral and aborts.
- Always commands neutral on exit (normal, error, or Ctrl+C).

Intended use: controlled calibration runs only.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from config import config
from pi_app.control.mapping import CENTER_OUTPUT_VALUE, MAX_OUTPUT
from pi_app.hardware.arduino_modelx import ArduinoModelXDriver
from pi_app.hardware.arduino_rc import ArduinoRCReader
from pi_app.hardware.vesc import VescCanDriver
from pi_app.control.safety import SafetyParams


def clamp_byte(v: int) -> int:
    return max(CENTER_OUTPUT_VALUE, min(MAX_OUTPUT, int(v)))


def is_armed(ch3_us: int, params: SafetyParams) -> bool:
    return ch3_us >= params.arm_high_threshold_us


def is_disarmed(ch3_us: int, params: SafetyParams) -> bool:
    return ch3_us <= params.arm_low_threshold_us


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Wait for next arm event, run forward for N seconds, "
            "abort immediately on disarm."
        )
    )
    parser.add_argument(
        "--forward-byte",
        type=int,
        default=146,
        help="Equal motor byte command during run (126..254, default 146).",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=10.0,
        help="Run duration in seconds once armed (default 10.0).",
    )
    parser.add_argument(
        "--tick-s",
        type=float,
        default=0.02,
        help="Control loop period in seconds (default 0.02).",
    )
    parser.add_argument(
        "--driver",
        choices=("arduino", "vesc", "auto"),
        default="vesc",
        help="Motor driver selection (default vesc).",
    )
    parser.add_argument(
        "--disarm-confirm-s",
        type=float,
        default=0.2,
        help="Require disarm state for this long before aborting (default 0.2).",
    )
    parser.add_argument(
        "--min-valid-run-s",
        type=float,
        default=1.0,
        help=(
            "If disarm abort happens before this run time, treat as false start and "
            "return to waiting-for-arm (default 1.0)."
        ),
    )
    args = parser.parse_args()

    if args.duration_s <= 0:
        print("duration-s must be > 0")
        return 2

    forward = clamp_byte(args.forward_byte)
    if forward <= CENTER_OUTPUT_VALUE:
        print(f"forward-byte must be > {CENTER_OUTPUT_VALUE}")
        return 2

    params = SafetyParams()
    stop_flag = {"value": False}

    def _on_sigint(_sig, _frame):
        stop_flag["value"] = True

    signal.signal(signal.SIGINT, _on_sigint)
    signal.signal(signal.SIGTERM, _on_sigint)

    rc = ArduinoRCReader()
    motor = None
    try:
        port = rc.start()
        print(f"RC reader started on {port}")

        if args.driver == "vesc":
            if not VescCanDriver.detect():
                print("ERROR: VESC CAN interface not detected (expected can0).")
                print("If needed for legacy hardware only, run with --driver arduino.")
                return 2
            print("Using VESC CAN driver (forced)")
            motor = VescCanDriver(
                left_id=2,
                right_id=1,
                max_rpm=getattr(config, "vesc", None).max_erpm if hasattr(config, "vesc") else 15000,
            )
        elif args.driver == "auto" and VescCanDriver.detect():
            print("Using VESC CAN driver (auto)")
            motor = VescCanDriver(
                left_id=2,
                right_id=1,
                max_rpm=getattr(config, "vesc", None).max_erpm if hasattr(config, "vesc") else 15000,
            )
        else:
            print("Using Arduino Model X motor driver")
            motor = ArduinoModelXDriver(rc_reader=rc)

        # Always start neutral.
        motor.stop()

        print(
            "Safety: Script waits for a NEW arm event.\n"
            "If you are already armed, disarm first, then arm again to start."
        )

        while not stop_flag["value"]:
            # Wait for a new disarm->arm transition.
            saw_disarmed = False
            while not stop_flag["value"]:
                s = rc.get_state()
                if is_disarmed(s.ch3_us, params):
                    saw_disarmed = True
                if saw_disarmed and is_armed(s.ch3_us, params):
                    break
                time.sleep(args.tick_s)

            if stop_flag["value"]:
                print("Cancelled before start.")
                return 130

            print(
                f"START: forward={forward} duration={args.duration_s:.1f}s "
                "(disarm at any time to abort)"
            )
            t0 = time.monotonic()
            disarm_since = None
            aborted = False
            while not stop_flag["value"]:
                s = rc.get_state()
                # RC freshness guard.
                age_s = time.time() - s.last_update_epoch_s if s.last_update_epoch_s > 0 else 999.0
                if age_s > 1.0:
                    print(f"ABORT: RC stale ({age_s:.2f}s).")
                    aborted = True
                    break
                # Abort on sustained disarm (debounced).
                if is_disarmed(s.ch3_us, params):
                    if disarm_since is None:
                        disarm_since = time.monotonic()
                    elif (time.monotonic() - disarm_since) >= args.disarm_confirm_s:
                        print(f"ABORT: disarm detected (ch3={s.ch3_us}).")
                        aborted = True
                        break
                else:
                    disarm_since = None
                if (time.monotonic() - t0) >= args.duration_s:
                    print("DONE: duration reached.")
                    return 0
                motor.set_tracks(forward, forward)
                time.sleep(args.tick_s)

            if stop_flag["value"]:
                return 130

            if aborted and (time.monotonic() - t0) < args.min_valid_run_s:
                print(
                    "False start: disarmed too quickly; waiting for next arm event."
                )
                continue
            # For a normal abort (after a meaningful run), exit cleanly.
            return 0

        return 130
    finally:
        try:
            if motor is not None:
                motor.stop()
        except Exception:
            pass
        try:
            rc.stop()
        except Exception:
            pass
        print("Neutral command sent. Exiting.")


if __name__ == "__main__":
    raise SystemExit(main())
