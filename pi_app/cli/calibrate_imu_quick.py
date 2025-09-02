#!/usr/bin/env python3
"""
Quick IMU calibration (gyro + magnetometer) for WALL-E Mini.

- Gyro bias: averages stationary gyro for a short duration.
- Magnetometer: computes hard-iron offsets and soft-iron scales.

Notes on axes:
- ISM330DHCX (accel/gyro) Z-axis is defined top-to-bottom.
- MMC5983MA (mag) Z-axis is defined bottom-to-top.
  The reader accounts for this by inverting mag Z before fusion and calibration.

Usage:
    python -m pi_app.cli.calibrate_imu_quick [--gyro-seconds 5] [--mag-seconds 12]
"""

import sys
import time
import argparse
from pathlib import Path

try:
    # Running as module
    from pi_app.hardware.imu_reader import ImuReader
    from config import config
except ModuleNotFoundError:
    # Allow running as script
    sys.path.append(str(Path(__file__).resolve().parents[2]))
    from pi_app.hardware.imu_reader import ImuReader  # type: ignore
    from config import config  # type: ignore


def main() -> int:
    parser = argparse.ArgumentParser(description="Quick gyro + mag calibration")
    parser.add_argument("--gyro-seconds", type=float, default=5.0, help="Duration for gyro bias calibration")
    parser.add_argument("--mag-seconds", type=float, default=12.0, help="Duration for magnetometer calibration")
    args = parser.parse_args()

    print("\nWALL-E Mini IMU Quick Calibration")
    print("---------------------------------")
    print("This will calibrate:")
    print("  - Gyro bias (keep robot completely stationary)")
    print("  - Magnetometer offsets and scales (rotate slowly through all orientations)")
    print("\nAxis note: ISM330DHCX Z and MMC5983MA Z are opposite directions; reader handles this internally.")

    imu = ImuReader(calibration_path=config.imu_calibration_path,
                    mag_axis_map=getattr(config, 'imu_mag_axis_map', None),
                    heading_cw_positive=getattr(config, 'imu_heading_cw_positive', True))
    eff_map = imu._get_effective_mag_axis_map()
    print(f"\nMag axis mapping in use: {eff_map}")

    # Gyro bias
    print(f"\nStep 1/2: Gyro bias for {args.gyro_seconds:.1f}s")
    print("  Place robot on a steady surface and do not move it.")
    time.sleep(1.0)
    bias = imu.calibrate_gyro(duration_s=args.gyro_seconds)
    print(f"  Gyro bias (dps): x={bias[0]:.3f}, y={bias[1]:.3f}, z={bias[2]:.3f}")

    # Magnetometer offsets and scales
    print(f"\nStep 2/2: Magnetometer calibration for {args.mag_seconds:.1f}s")
    print("  Slowly rotate robot in all directions, covering full 3D space.")
    time.sleep(1.0)
    off, scl = imu.calibrate_mag(duration_s=args.mag_seconds)
    print(f"  Mag offsets (G): x={off[0]:.3f}, y={off[1]:.3f}, z={off[2]:.3f}")
    print(f"  Mag scales:      x={scl[0]:.3f}, y={scl[1]:.3f}, z={scl[2]:.3f}")

    # Save
    imu.save_calibration()
    print(f"\n✓ Calibration saved to {config.imu_calibration_path}")

    # Quick sanity sample
    print("\nSanity check (heading samples):")
    for _ in range(5):
        d = imu.read()
        print(f"  Heading={d['heading_deg']:.1f}°, YawRate={d['gz_dps']:.2f} dps, Mag=({d['mx_g']:.3f},{d['my_g']:.3f},{d['mz_g']:.3f}) G")
        time.sleep(0.2)

    print("\nDone.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
