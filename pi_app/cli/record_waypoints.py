#!/usr/bin/env python3
"""
Interactive waypoint recorder.

Reads live GPS, lets the user press Enter to capture the current position,
and saves the result to a JSON file compatible with waypoint_nav.

Usage:
    python3 -m pi_app.cli.record_waypoints [--output waypoints.json]
"""

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from config import config
from pi_app.hardware.rtk_gps import RtkGpsReader


def main() -> None:
    parser = argparse.ArgumentParser(description="Record GPS waypoints interactively")
    parser.add_argument(
        "--output", "-o",
        default=str(Path(__file__).resolve().parents[2] / "waypoints.json"),
        help="Output JSON file (default: waypoints.json in project root)",
    )
    args = parser.parse_args()

    bus = config.gps.i2c_bus
    addr = config.gps.i2c_addr

    if not RtkGpsReader.detect(bus, addr):
        print("RTK GPS not detected. Check wiring.")
        sys.exit(1)

    reader = RtkGpsReader(config.gps)
    reader.start()

    print("Waiting for GPS fix ...")
    for _ in range(30):
        if reader.get_reading() is not None:
            break
        time.sleep(1.0)
    else:
        print("No fix after 30 s — continuing anyway.\n")

    waypoints: list[dict] = []
    print("Press ENTER to record current position, 'q' to save and quit.\n")

    try:
        while True:
            r = reader.get_reading()
            if r is not None:
                age = time.monotonic() - r.timestamp
                sys.stdout.write(
                    f"\r  lat={r.latitude:.8f}  lon={r.longitude:.8f}  "
                    f"fix={r.fix_quality}  sats={r.satellites_used}  "
                    f"age={age:.1f}s   "
                )
                sys.stdout.flush()
            else:
                sys.stdout.write("\r  (waiting for fix)          ")
                sys.stdout.flush()

            # Non-blocking stdin check
            import select
            ready, _, _ = select.select([sys.stdin], [], [], 0.5)
            if not ready:
                continue

            line = sys.stdin.readline().strip()
            if line.lower() == "q":
                break

            if r is None:
                print("\n  No GPS fix — cannot record. Wait for a fix.\n")
                continue

            name = f"WP{len(waypoints) + 1}"
            wp = {"lat": r.latitude, "lon": r.longitude, "name": name}
            waypoints.append(wp)
            print(f"\n  Recorded {name}: ({r.latitude:.8f}, {r.longitude:.8f})\n")

    except KeyboardInterrupt:
        print()
    finally:
        reader.stop()

    if not waypoints:
        print("No waypoints recorded.")
        return

    out_path = Path(args.output)
    out_path.write_text(json.dumps(waypoints, indent=2) + "\n")
    print(f"Saved {len(waypoints)} waypoints to {out_path}")


if __name__ == "__main__":
    main()
