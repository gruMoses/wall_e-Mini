#!/usr/bin/env python3
"""
depthai_upgrade_check.py — READ-ONLY bench check for a depthai upgrade.

Run it once on depthai 3.3.0 (the baseline) and once after the upgrade, with
the robot at rest on a level floor. It prints:
  * the depthai version, the device, the USB link speed and
    DEPTHAI_AUTOCALIBRATION;
  * the CAM_A factory intrinsics at 640x400 against the values the corridor
    was validated on (fx 456.89, cx 334.95);
  * for each IMU report family this depthai has (RAW, UNCALIBRATED,
    CALIBRATED): the mean accelerometer vector in g and the mean gyro in deg/s
    over a few seconds at rest, and which axis carries gravity;
  * which family the robot itself will request (oak_depth.imu_report_names)
    and whether that family shows gravity on -Y (about -1 g), the RDF frame
    the yaw channel (gyro_y, +Y down) is built on.

Why: depthai 3.6.1 changed the RAW IMU reports from the RDF frame to the
sensor's own frame (UNCALIBRATED is the old RAW), and turned AutoCalibration
on by default. Either can change the robot's geometry without an error.

This tool moves nothing and writes nothing to the device. The OAK allows one
process at a time, so stop wall-e.service first (robot disarmed). The gravity
check fixes the vertical axis only; the yaw SIGN still needs the chalk test
(docs/heading_tuning.md) after an upgrade.

Usage (on the robot, wall-e stopped):
  python3 tools/depthai_upgrade_check.py [--seconds 3]
Exit code 0 when every check passes, 1 when one fails, 2 when the device
cannot be opened.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

# The same default the robot uses (oak_depth sets it before importing depthai).
os.environ.setdefault("DEPTHAI_AUTOCALIBRATION", "OFF")

from pi_app.hardware.oak_depth import imu_report_names  # noqa: E402

EXPECTED_FX = 456.89
EXPECTED_CX = 334.95
INTRINSICS_TOL_PX = 0.5
GRAVITY_MIN_G = 0.9          # |a_y| must exceed this on the gravity axis
G_MSS = 9.80665

# (accelerometer, gyroscope) enum names per family, in preference order.
FAMILIES = {
    "RAW": (("ACCELEROMETER_RAW",), ("GYROSCOPE_RAW",)),
    "UNCALIBRATED": (("ACCELEROMETER_UNCALIBRATED",), ("GYROSCOPE_UNCALIBRATED",)),
    "CALIBRATED": (("ACCELEROMETER_CALIBRATED", "ACCELEROMETER"), ("GYROSCOPE_CALIBRATED",)),
}


def _first_member(enum, names):
    for name in names:
        if hasattr(enum, name):
            return name
    return None


def _sample_imu(dai, accel_name: str, gyro_name: str, seconds: float):
    """Mean accel (g) and gyro (deg/s) vectors over `seconds` at rest."""
    acc = [0.0, 0.0, 0.0]
    gyr = [0.0, 0.0, 0.0]
    n_acc = n_gyr = 0
    with dai.Pipeline() as pipeline:
        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor(getattr(dai.IMUSensor, accel_name), 100)
        imu.enableIMUSensor(getattr(dai.IMUSensor, gyro_name), 100)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        q = imu.out.createOutputQueue(maxSize=64, blocking=False)
        pipeline.start()
        # Skip the first half second: the first packets after start can be
        # from before the sensor settled.
        t_skip = time.monotonic() + 0.5
        t_end = t_skip + seconds
        while time.monotonic() < t_end and pipeline.isRunning():
            msg = q.tryGet()
            if msg is None:
                time.sleep(0.005)
                continue
            if time.monotonic() < t_skip:
                continue
            for pkt in msg.packets:
                a = pkt.acceleroMeter
                g = pkt.gyroscope
                acc[0] += a.x; acc[1] += a.y; acc[2] += a.z
                n_acc += 1
                gyr[0] += g.x; gyr[1] += g.y; gyr[2] += g.z
                n_gyr += 1
        pipeline.stop()
    if n_acc == 0 or n_gyr == 0:
        return None
    acc_g = [v / n_acc / G_MSS for v in acc]
    gyr_dps = [math.degrees(v / n_gyr) for v in gyr]
    return acc_g, gyr_dps, n_acc


def _gravity_axis(acc_g) -> str:
    i = max(range(3), key=lambda k: abs(acc_g[k]))
    return f"{'+' if acc_g[i] > 0 else '-'}{'XYZ'[i]}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--seconds", type=float, default=3.0,
                    help="seconds of IMU data per report family (default 3)")
    args = ap.parse_args()

    try:
        import depthai as dai
    except ImportError:
        print("depthai is not installed for this interpreter")
        return 2

    version = getattr(dai, "__version__", "?")
    print(f"depthai {version}   DEPTHAI_AUTOCALIBRATION="
          f"{os.environ.get('DEPTHAI_AUTOCALIBRATION', '(unset)')}")
    ok = True

    try:
        with dai.Device() as device:
            print(f"device {device.getDeviceName()}   USB {device.getUsbSpeed().name}")
            calib = device.readCalibration()
            intr = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, 640, 400)
            fx, cx = float(intr[0][0]), float(intr[0][2])
    except Exception as exc:  # no device, or held by wall-e
        print(f"cannot open the OAK: {exc}")
        print("stop wall-e.service first (robot disarmed); the OAK allows one process")
        return 2

    intr_ok = (abs(fx - EXPECTED_FX) <= INTRINSICS_TOL_PX
               and abs(cx - EXPECTED_CX) <= INTRINSICS_TOL_PX)
    ok &= intr_ok
    print(f"CAM_A 640x400 fx={fx:.2f} cx={cx:.2f} (expect {EXPECTED_FX} / {EXPECTED_CX})"
          f"   {'PASS' if intr_ok else 'FAIL'}")

    robot_accel, robot_gyro = imu_report_names(version, dai.IMUSensor)
    robot_family = next(
        (fam for fam, (a, g) in FAMILIES.items() if robot_accel in a and robot_gyro in g),
        None,
    )
    print(f"the robot requests {robot_accel} + {robot_gyro} ({robot_family})")

    robot_checked = False
    for family, (accel_names, gyro_names) in FAMILIES.items():
        accel = _first_member(dai.IMUSensor, accel_names)
        gyro = _first_member(dai.IMUSensor, gyro_names)
        if accel is None or gyro is None:
            print(f"  {family:13s} not in this depthai")
            continue
        try:
            result = _sample_imu(dai, accel, gyro, args.seconds)
        except Exception as exc:
            print(f"  {family:13s} failed: {exc}")
            if family == robot_family:
                ok = False
                robot_checked = True
            continue
        if result is None:
            print(f"  {family:13s} no packets")
            if family == robot_family:
                ok = False
                robot_checked = True
            continue
        acc_g, gyr_dps, n = result
        line = (f"  {family:13s} accel g ({acc_g[0]:+.3f}, {acc_g[1]:+.3f}, {acc_g[2]:+.3f})"
                f"  gravity {_gravity_axis(acc_g)}"
                f"  gyro dps ({gyr_dps[0]:+.3f}, {gyr_dps[1]:+.3f}, {gyr_dps[2]:+.3f})  n={n}")
        if family == robot_family:
            robot_checked = True
            rdf_ok = acc_g[1] <= -GRAVITY_MIN_G
            ok &= rdf_ok
            line += f"   robot's family: gravity on -Y {'PASS' if rdf_ok else 'FAIL'}"
        print(line)

    if not robot_checked:
        print("the robot's IMU family was not sampled   FAIL")
        ok = False
    print("RESULT", "PASS" if ok else "FAIL",
          "(the yaw SIGN still needs the chalk test after an upgrade)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
