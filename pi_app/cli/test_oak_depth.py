#!/usr/bin/env python3
"""
Standalone OAK-D Lite verification tool.

Runs headlessly (no OpenCV display) so it works over SSH on a Pi.
Tests depth readings and person detection in separate phases.

Usage:
    python3 pi_app/cli/test_oak_depth.py [--duration SECONDS]
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from config import config
from pi_app.hardware.oak_depth import OakDepthReader


def main() -> None:
    parser = argparse.ArgumentParser(description="OAK-D Lite hardware verification")
    parser.add_argument(
        "--duration", type=float, default=10.0,
        help="Seconds to run each test phase (default: 10)",
    )
    args = parser.parse_args()

    # Phase 0: detection
    print("=== Phase 0: Device Detection ===")
    if not OakDepthReader.detect():
        print("ERROR: No OAK-D Lite found on USB.")
        print("Troubleshooting:")
        print("  - Check USB 3.0 cable and port")
        print("  - Run: lsusb | grep 03e7")
        print("  - Install udev rules if not done:")
        print("    echo 'SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"03e7\", MODE=\"0666\"' "
              "| sudo tee /etc/udev/rules.d/80-movidius.rules")
        print("    sudo udevadm control --reload-rules && sudo udevadm trigger")
        sys.exit(1)
    print("OAK-D Lite detected on USB.\n")

    # Phase 1: depth + detection pipeline
    print("=== Phase 1: Depth + Person Detection ===")
    print(f"Running for {args.duration:.0f}s — point camera at objects / people...\n")
    reader = OakDepthReader(config.obstacle_avoidance, config.follow_me)
    reader.start()

    # Allow pipeline to initialise
    time.sleep(2.0)

    end_time = time.monotonic() + args.duration
    tick = 0
    while time.monotonic() < end_time:
        dist_m, age_s = reader.get_min_distance()
        persons = reader.get_person_detections()
        tick += 1

        if dist_m == float("inf"):
            dist_str = "  no depth"
        else:
            dist_str = f"  min_dist={dist_m:.3f}m  age={age_s:.2f}s"

        person_strs = []
        for p in persons:
            person_strs.append(
                f"    person z={p.z_m:.2f}m x={p.x_m:.2f}m conf={p.confidence:.0%}"
            )

        print(f"[tick {tick:4d}]{dist_str}  persons={len(persons)}")
        for ps in person_strs:
            print(ps)

        time.sleep(0.2)

    reader.stop()
    print("\nDone. Camera shut down cleanly.")


if __name__ == "__main__":
    main()
