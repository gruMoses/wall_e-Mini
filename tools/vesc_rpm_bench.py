"""
vesc_rpm_bench.py — VESC RPM truth-check bench script.

Commands a fixed low RPM for --duration seconds, samples reported ERPM and
RX counters at ~10 Hz, prints a per-sample table, and emits a final verdict
(per-type counter delta, whether ERPM was ever nonzero, commanded vs reported).

NOTE: VESC firmware command timeout — motors stop if RPM commands are not
refreshed within ~1 s; this is VESC firmware behavior, observed 2026-06-11
on the real robot.

SAFETY GATES — this script CAN spin motors:
  1. --i-confirm-wheels-are-off-the-ground  (literal flag, required)
  2. Refuses to run if the wall-e systemd service is active unless
     --service-already-stopped is also passed (two processes on one CAN bus
     corrupts arbitration).
  3. Default ERPM is low (1500) with a hard cap of 5000.
  4. --dry-run prints exactly what would be sent without opening the bus.

Usage examples:
  # Dry-run (safe, no bus):
  python tools/vesc_rpm_bench.py --dry-run

  # Real run (wheels must be off the ground, service must be stopped):
  python tools/vesc_rpm_bench.py \\
      --i-confirm-wheels-are-off-the-ground \\
      --service-already-stopped \\
      --rpm 1500 --duration 3.0

  # With explicit channel/IDs:
  python tools/vesc_rpm_bench.py \\
      --i-confirm-wheels-are-off-the-ground \\
      --service-already-stopped \\
      --channel can0 --left-id 2 --right-id 1 --rpm 1500 --duration 3.0
"""

from __future__ import annotations

import argparse
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

# Allow running as `python3 tools/vesc_rpm_bench.py` from anywhere: put the
# repo root on sys.path so `pi_app.hardware.vesc` resolves.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

# Hard cap: never command more than this ERPM regardless of --rpm value.
_ERPM_HARD_CAP = 5_000
_ERPM_DEFAULT = 1_500


# ---------------------------------------------------------------------------
# Safety helpers
# ---------------------------------------------------------------------------

def _is_service_active(service_name: str) -> Optional[bool]:
    """Return True/False if systemctl is available; None if not (non-Linux)."""
    if shutil.which("systemctl") is None:
        return None
    try:
        result = subprocess.run(
            ["systemctl", "is-active", service_name],
            capture_output=True,
            text=True,
            timeout=3,
        )
        return result.stdout.strip() == "active"
    except (OSError, subprocess.TimeoutExpired):
        return None


def _check_safety_gates(args: argparse.Namespace) -> None:
    """Enforce safety gates. Raises SystemExit on any violation."""
    if not args.i_confirm_wheels_are_off_the_ground:
        print(
            "ERROR: Safety gate not cleared.\n"
            "  This script WILL spin motors. You must pass:\n"
            "    --i-confirm-wheels-are-off-the-ground\n"
            "  Only run this script with the robot elevated so the wheels "
            "cannot contact the ground.",
            file=sys.stderr,
        )
        sys.exit(2)

    if not args.dry_run:
        active = _is_service_active("wall-e")
        if active is True and not args.service_already_stopped:
            print(
                "ERROR: The wall-e systemd service is currently active.\n"
                "  Two processes sharing one CAN bus corrupts arbitration.\n"
                "  Stop the service first:\n"
                "    sudo systemctl stop wall-e\n"
                "  Then re-run with --service-already-stopped.\n"
                "  OR pass --service-already-stopped if you are certain the\n"
                "  service is not driving the bus (e.g. it is in a failed state).",
                file=sys.stderr,
            )
            sys.exit(3)

    if args.rpm > _ERPM_HARD_CAP:
        print(
            f"ERROR: --rpm {args.rpm} exceeds the hard cap of {_ERPM_HARD_CAP} ERPM.\n"
            f"  Lower the value or edit _ERPM_HARD_CAP in this script if you are\n"
            f"  certain the mechanical setup can handle it.",
            file=sys.stderr,
        )
        sys.exit(4)


# ---------------------------------------------------------------------------
# Table formatting
# ---------------------------------------------------------------------------

_HEADER = (
    f"{'t(s)':>6}  "
    f"{'cmd_tx':>6}  "
    f"{'L-ERPM':>8}  {'R-ERPM':>8}  "
    f"{'rx_tot':>6}  {'rx_s9':>5}  {'rx_s16':>6}  {'rx_s27':>6}  "
    f"{'parse_err':>9}  {'recv_err':>8}"
)
_SEP = "-" * len(_HEADER)


def _fmt_row(
    elapsed: float,
    left_rpm: Optional[int],
    right_rpm: Optional[int],
    health: dict,
    cmd_tx: int = 0,
) -> str:
    lrpm = f"{left_rpm:+d}" if left_rpm is not None else "None"
    rrpm = f"{right_rpm:+d}" if right_rpm is not None else "None"
    return (
        f"{elapsed:>6.2f}  "
        f"{cmd_tx:>6}  "
        f"{lrpm:>8}  {rrpm:>8}  "
        f"{health['rx_frame_count']:>6}  "
        f"{health['rx_status_count']:>5}  "
        f"{health['rx_status4_count']:>6}  "
        f"{health['rx_status5_count']:>6}  "
        f"{health['rx_parse_error_count']:>9}  "
        f"{health['rx_recv_error_count']:>8}"
    )


# ---------------------------------------------------------------------------
# Core bench logic
# ---------------------------------------------------------------------------

def run_bench(args: argparse.Namespace) -> int:
    """Execute the bench run. Returns exit code (0=pass, 1=fail/no-erpm)."""
    from pi_app.hardware.vesc import VescCanDriver  # noqa: PLC0415 (intentional late import)

    rpm = min(args.rpm, _ERPM_HARD_CAP)
    duration = min(args.duration, 10.0)

    driver = VescCanDriver(
        channel=args.channel,
        left_id=args.left_id,
        right_id=args.right_id,
        max_rpm=_ERPM_HARD_CAP,
    )

    print(f"\nVESC RPM Bench")
    print(f"  Channel : {args.channel}")
    print(f"  Left ID : {args.left_id}   Right ID: {args.right_id}")
    print(f"  Command : {rpm:+d} ERPM (both motors)")
    print(f"  Duration: {duration:.1f}s  (max 10s)")
    print(f"  Sample  : ~10 Hz")
    print()
    print(_HEADER)
    print(_SEP)

    driver.start()

    # samples stores (elapsed, left_rpm, right_rpm, health, cmd_tx)
    samples: list[tuple[float, Optional[int], Optional[int], dict, int]] = []
    erpm_ever_nonzero = False
    cmd_tx = 0  # running count of RPM command sends

    # Snapshot health at start so we can compute deltas
    health0 = driver.get_rx_health()

    def _stop_motors() -> None:
        driver.stop()

    # Register signal handlers for clean shutdown
    def _sig_handler(signum, frame):  # noqa: ARG001
        _stop_motors()
        driver.shutdown()
        sys.exit(130)

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    # Command target RPM and sample in a try/finally so stop is ALWAYS sent
    try:
        # Initial send before entering the loop
        # We use _send_rpm directly so the value is the exact commanded ERPM.
        driver._send_rpm(args.left_id, rpm)
        driver._send_rpm(args.right_id, rpm)
        cmd_tx += 1

        t_start = time.monotonic()
        next_sample = t_start
        SAMPLE_INTERVAL = 0.1  # 10 Hz

        while True:
            now = time.monotonic()
            elapsed = now - t_start
            if elapsed >= duration:
                break

            if now >= next_sample:
                # Re-send RPM on every sample tick to prevent VESC ~1 s command
                # timeout from stopping the motors during the run.
                driver._send_rpm(args.left_id, rpm)
                driver._send_rpm(args.right_id, rpm)
                cmd_tx += 1

                left_rpm = driver.get_rpm("left")
                right_rpm = driver.get_rpm("right")
                health = driver.get_rx_health()

                samples.append((elapsed, left_rpm, right_rpm, health, cmd_tx))
                print(_fmt_row(elapsed, left_rpm, right_rpm, health, cmd_tx))

                if (left_rpm is not None and left_rpm != 0) or \
                   (right_rpm is not None and right_rpm != 0):
                    erpm_ever_nonzero = True

                next_sample += SAMPLE_INTERVAL

            time.sleep(0.005)

    finally:
        # ALWAYS command stop — runs on normal exit, exception, and signal
        driver.stop()
        # Small wait so the stop frame clears before we close
        time.sleep(0.05)
        driver.shutdown()

    # ---- Final verdict ----
    health_final = driver.get_rx_health()
    # Build deltas vs start snapshot
    delta = {k: health_final[k] - health0[k]
             for k in ("rx_frame_count", "rx_status_count",
                       "rx_status4_count", "rx_status5_count",
                       "rx_parse_error_count", "rx_recv_error_count",
                       "rx_reopen_count")}

    # Steady-state window: samples with elapsed > 0.5 s
    steady = [(s[1], s[2]) for s in samples if s[0] > 0.5]
    steady_l = [v for v, _ in steady if v is not None]
    steady_r = [v for _, v in steady if v is not None]

    mean_l = (sum(steady_l) / len(steady_l)) if steady_l else None
    mean_r = (sum(steady_r) / len(steady_r)) if steady_r else None

    def _pct_err(mean: Optional[float], cmd: int) -> str:
        if mean is None or cmd == 0:
            return "N/A"
        return f"{((mean - cmd) / cmd) * 100:+.1f}%"

    print()
    print(_SEP)
    print("VERDICT")
    print(_SEP)
    print(f"  Commanded ERPM              : {rpm:+d}")
    print(f"  RPM commands sent (cmd_tx)  : {cmd_tx}  (initial + {cmd_tx - 1} loop refreshes)")
    print()
    print("  Steady-state window (t > 0.5 s):")
    if steady_l or steady_r:
        print(f"    Left  — mean reported ERPM : {mean_l:+.1f}  "
              f"({len(steady_l)} samples, error vs commanded: {_pct_err(mean_l, rpm)})")
        print(f"    Right — mean reported ERPM : {mean_r:+.1f}  "
              f"({len(steady_r)} samples, error vs commanded: {_pct_err(mean_r, rpm)})")
    else:
        print("    No steady-state samples (run too short or ERPM never reported).")
    print()
    print(f"  ERPM ever nonzero           : {'YES' if erpm_ever_nonzero else 'NO  <-- STATUS(9) packets may not be flowing'}")
    print()
    print("  RX counter deltas over run:")
    print(f"    rx_frame_count    (total)   : {delta['rx_frame_count']}")
    print(f"    rx_status_count   (pkt 9)   : {delta['rx_status_count']}  <- ERPM lives here")
    print(f"    rx_status4_count  (pkt 16)  : {delta['rx_status4_count']}")
    print(f"    rx_status5_count  (pkt 27)  : {delta['rx_status5_count']}")
    print(f"    rx_parse_error_count        : {delta['rx_parse_error_count']}")
    print(f"    rx_recv_error_count         : {delta['rx_recv_error_count']}")
    print(f"    rx_reopen_count             : {delta['rx_reopen_count']}")
    print()

    if delta["rx_status_count"] == 0:
        print("  DIAGNOSIS: No STATUS(9) frames received.")
        print("    -> VESC may not be broadcasting RPM status (check VESC Tool → App → General → Status Message Mode).")
        print("    -> Or the commanded ERPM is below the VESC's minimum speed threshold.")
        if delta["rx_frame_count"] > 0:
            print("    -> Other frame types did arrive; CAN bus connectivity is OK.")
    elif not erpm_ever_nonzero:
        print("  DIAGNOSIS: STATUS(9) frames arrived but ERPM was always 0.")
        print("    -> VESC is running but speed PID / duty is not producing rotation.")
        print("    -> Check velocity PID enable in VESC Tool.")
    else:
        print("  DIAGNOSIS: ERPM was nonzero — RPM telemetry is flowing correctly.")

    return 0 if erpm_ever_nonzero else 1


# ---------------------------------------------------------------------------
# Dry-run mode
# ---------------------------------------------------------------------------

def run_dry(args: argparse.Namespace) -> int:
    """Print exactly what would be sent without touching the CAN bus."""
    import struct  # noqa: PLC0415

    rpm = min(args.rpm, _ERPM_HARD_CAP)
    duration = min(args.duration, 10.0)

    print(f"\n[DRY-RUN] No CAN bus will be opened.\n")
    print(f"  Channel : {args.channel}")
    print(f"  Left ID : {args.left_id}   Right ID: {args.right_id}")
    print(f"  Command : {rpm:+d} ERPM (both motors)")
    print(f"  Duration: {duration:.1f}s  (max 10s)")
    print()

    for can_id, label in ((args.left_id, "left"), (args.right_id, "right")):
        arb_id = 0x300 + can_id
        data = struct.pack(">i", int(rpm))
        print(f"  WOULD SEND  RPM cmd  → arb_id=0x{arb_id:03X}  "
              f"data={data.hex()}  ({label}, ERPM={rpm:+d})")

    for can_id, label in ((args.left_id, "left"), (args.right_id, "right")):
        arb_id = 0x300 + can_id
        data = struct.pack(">i", 0)
        print(f"  WOULD SEND  STOP cmd → arb_id=0x{arb_id:03X}  "
              f"data={data.hex()}  ({label}, ERPM=0)")

    print(f"\n  WOULD SAMPLE ~{int(duration / 0.1)} times at 10 Hz over {duration:.1f}s.")
    print()
    return 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="vesc_rpm_bench.py",
        description=(
            "VESC RPM truth-check bench script. "
            "Commands a fixed RPM for --duration seconds, samples ERPM + RX health "
            "counters at 10 Hz, then always stops motors and prints a verdict."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    # Safety flags
    safety = p.add_argument_group("SAFETY GATES (read carefully)")
    safety.add_argument(
        "--i-confirm-wheels-are-off-the-ground",
        dest="i_confirm_wheels_are_off_the_ground",
        action="store_true",
        default=False,
        help="Required: confirms the robot is elevated and wheels cannot contact the ground.",
    )
    safety.add_argument(
        "--service-already-stopped",
        dest="service_already_stopped",
        action="store_true",
        default=False,
        help="Confirms the wall-e systemd service is not running (required when it is active).",
    )
    safety.add_argument(
        "--dry-run",
        dest="dry_run",
        action="store_true",
        default=False,
        help="Print what would be sent without opening the CAN bus. No confirmation flags needed.",
    )

    # CAN parameters
    hw = p.add_argument_group("CAN hardware")
    hw.add_argument("--channel", default="can0", help="SocketCAN channel (default: can0)")
    hw.add_argument("--left-id", dest="left_id", type=int, default=2,
                    help="VESC CAN ID for left motor (default: 2)")
    hw.add_argument("--right-id", dest="right_id", type=int, default=1,
                    help="VESC CAN ID for right motor (default: 1)")

    # Bench parameters
    bench = p.add_argument_group("bench parameters")
    bench.add_argument(
        "--rpm", type=int, default=_ERPM_DEFAULT,
        help=f"Target ERPM to command (default: {_ERPM_DEFAULT}, hard cap: {_ERPM_HARD_CAP}).",
    )
    bench.add_argument(
        "--duration", type=float, default=3.0,
        help="Duration to hold RPM command in seconds (default: 3.0, max: 10.0).",
    )

    return p


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    # --dry-run bypasses all safety gates (nothing physical happens)
    if args.dry_run:
        return run_dry(args)

    _check_safety_gates(args)
    return run_bench(args)


if __name__ == "__main__":
    sys.exit(main())
