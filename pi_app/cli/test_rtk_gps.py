#!/usr/bin/env python3
"""
Quick-check tool for the DFRobot GNSS-RTK rover module.

Usage:
    python3 -m pi_app.cli.test_rtk_gps
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from config import config
from pi_app.hardware.rtk_gps import RtkGpsReader


def main() -> None:
    bus = config.gps.i2c_bus
    addr = config.gps.i2c_addr
    print(f"Probing I2C bus {bus} addr 0x{addr:02X} ...")

    if not RtkGpsReader.detect(bus, addr):
        print("RTK GPS not detected. Check wiring and address.")
        sys.exit(1)

    print("Device detected. Starting reader ...\n")
    reader = RtkGpsReader(config.gps)
    reader.start()

    try:
        while True:
            r = reader.get_reading()
            if r is None:
                print("  (waiting for fix)")
            else:
                age = time.monotonic() - r.timestamp
                print(
                    f"  lat={r.latitude:.8f}  lon={r.longitude:.8f}  "
                    f"alt={r.altitude_m:.1f}m  "
                    f"fix={r.fix_quality}  sats={r.satellites_used}  "
                    f"hdop={r.hdop:.1f}  age={age:.1f}s"
                )
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopping ...")
    finally:
        reader.stop()


if __name__ == "__main__":
    main()
