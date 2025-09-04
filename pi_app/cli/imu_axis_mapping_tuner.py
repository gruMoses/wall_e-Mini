#!/usr/bin/env python3
"""
Interactive magnetometer axis mapping tuner for WALL-E Mini.

- Cycles through common mag axis mappings and streams heading/roll/pitch.
- Helps you pick a mapping that minimizes heading change when pitching/rolling.

Usage:
  python -m pi_app.cli.imu_axis_mapping_tuner [--duration 8] [--rate 10]

Tips:
- Keep the robot facing a fixed direction while tilting to spot coupling.
- Once you find a good mapping, set it in config.py as `imu_mag_axis_map` and
  re-run `calibrate_imu_quick.py` to compute offsets/scales for that mapping.
"""

import sys
import time
import argparse
from pathlib import Path

try:
    from pi_app.hardware.imu_reader import ImuReader
    from config import config
except ModuleNotFoundError:
    sys.path.append(str(Path(__file__).resolve().parents[2]))
    from pi_app.hardware.imu_reader import ImuReader  # type: ignore
    from config import config  # type: ignore


DEFAULT_MAPPINGS: list[tuple[str, str, str]] = [
    ('x', 'y', '-z'),  # ISM+MMC default (Z inverted)
    ('y', 'x', '-z'),  # swap X/Y
    ('-x', 'y', '-z'), # invert X
    ('x', '-y', '-z'), # invert Y
]


def stream_for(imu: ImuReader, seconds: float, rate_hz: float) -> None:
    end_t = time.monotonic() + seconds
    dt = 1.0 / max(1.0, rate_hz)
    while time.monotonic() < end_t:
        d = imu.read()
        print(
            f"  hdg={d['heading_deg']:+07.2f}°  roll={d['roll_deg']:+06.2f}°  "
            f"pitch={d['pitch_deg']:+06.2f}°  yawRate={d['gz_dps']:+06.2f} dps"
        )
        time.sleep(dt)


def main() -> int:
    ap = argparse.ArgumentParser(description="IMU mag axis mapping tuner")
    ap.add_argument("--duration", type=float, default=8.0, help="Seconds per mapping")
    ap.add_argument("--rate", type=float, default=10.0, help="Print rate (Hz)")
    args = ap.parse_args()

    print("\nWALL-E Mini IMU Mag Axis Mapping Tuner")
    print("-------------------------------------")
    print("Instructions:")
    print("- Keep the robot pointing roughly one direction.")
    print("- For each mapping, tilt/roll the robot and watch heading stability.")
    print("- Press Ctrl-C to skip early; press Enter to go to next mapping.")

    mappings = DEFAULT_MAPPINGS

    for i, m in enumerate(mappings, start=1):
        print(f"\n[{i}/{len(mappings)}] Testing mapping: {m}")
        imu = ImuReader(
            calibration_path=config.imu_calibration_path,
            mag_axis_map=m,
            use_magnetometer=config.imu_use_magnetometer,
        )
        # Short settle
        time.sleep(0.2)
        try:
            stream_for(imu, args.duration, args.rate)
        except KeyboardInterrupt:
            print("  (skipped)\n")
        input("Press Enter for next mapping...")

    print("\nDone. Set your chosen mapping in config.py as `imu_mag_axis_map` and")
    print("re-run `python -m pi_app.cli.calibrate_imu_quick` to save offsets/scales.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

