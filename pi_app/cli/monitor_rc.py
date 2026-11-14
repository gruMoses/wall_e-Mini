import argparse
import signal
import sys
import time

from typing import Optional

try:
    from pi_app.hardware.arduino_rc import ArduinoRCReader
    from pi_app.control.rc_signal import rc_signal_ok
except Exception:
    # Fallback for running as a script without an installed package
    from pathlib import Path
    sys.path.append(str(Path(__file__).resolve().parents[2]))
    from pi_app.hardware.arduino_rc import ArduinoRCReader  # type: ignore
    from pi_app.control.rc_signal import rc_signal_ok  # type: ignore


def _fmt_channel(us: int) -> str:
    """A width as 4 digits; the no-signal sentinel (or any invalid value) as NOSIG."""
    return f"{int(us):4d}" if rc_signal_ok(us) else "NOSIG"


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Monitor Arduino RC channels (CSV: ch1..ch5, or ch1..ch6; 0 = no signal)"
    )
    parser.add_argument("--port", help="Explicit serial port (e.g., /dev/ttyUSB0)", default=None)
    parser.add_argument("--hz", type=float, default=10.0, help="Print rate in Hz (default 10)")
    args = parser.parse_args(argv)

    reader = ArduinoRCReader(port=args.port)
    try:
        port = reader.start()
    except RuntimeError as e:
        print(str(e), file=sys.stderr)
        return 1

    print(f"Arduino RC detected on {port}")

    should_run = True

    def handle_sigint(_signum, _frame):
        nonlocal should_run
        should_run = False

    signal.signal(signal.SIGINT, handle_sigint)

    period_s = 1.0 / max(0.1, args.hz)
    try:
        while should_run:
            state = reader.get_state()
            age_s = time.monotonic() - state.last_update_mono_s if state.last_update_mono_s else float("inf")
            stale = age_s > 1.0
            status = "STALE" if stale else "OK   "
            values = [state.ch1_us, state.ch2_us, state.ch3_us, state.ch4_us, state.ch5_us]
            ch6 = getattr(state, "ch6_us", None)
            if ch6 is not None:
                values.append(ch6)
            fields = "  ".join(
                f"ch{i}={_fmt_channel(v)}" for i, v in enumerate(values, start=1)
            )
            print(
                f"{status} {fields}  age={age_s:4.1f}s",
                end="\r",
                flush=True,
            )
            time.sleep(period_s)
    finally:
        reader.stop()
        print()  # newline after the carriage-return loop
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


