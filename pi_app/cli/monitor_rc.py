import argparse
import signal
import sys
import time

from typing import Optional

try:
    from pi_app.hardware.arduino_rc import ArduinoRCReader
except Exception:
    # Fallback for running as a script without an installed package
    from pathlib import Path
    sys.path.append(str(Path(__file__).resolve().parents[2]))
    from pi_app.hardware.arduino_rc import ArduinoRCReader  # type: ignore


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Monitor Arduino RC channels (CSV: ch1,ch2,ch3,ch4,ch5)")
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
            age_s = time.time() - state.last_update_epoch_s if state.last_update_epoch_s else float("inf")
            stale = age_s > 1.0
            status = "STALE" if stale else "OK   "
            print(
                f"{status} ch1={state.ch1_us:4d}  ch2={state.ch2_us:4d}  ch3={state.ch3_us:4d}  ch4={state.ch4_us:4d}  ch5={state.ch5_us:4d}  age={age_s:4.1f}s",
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


