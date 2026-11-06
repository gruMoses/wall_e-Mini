#!/usr/bin/env python3
"""
vio_probe.py — LOG-ONLY stereo-inertial odometry probe (depthai BasaltVIO).

Runs depthai's BasaltVIO host node on the OAK-D Lite's two mono cameras and
its IMU, and writes the pose it reports to a CSV. Nothing here reaches the
motors or the control loop: this is an evaluation tool, to find out whether
VIO heading and distance beat gyro-only yaw on this robot before any of it is
wired into control.

What to measure with it (robot pushed or driven by hand, VIO only logging):
  * a 90 degree turn on the chalk marks (docs/heading_tuning.md): VIO yaw
    change against 90, and its sign (right turn = clockwise);
  * a parked minute: yaw and position drift at rest;
  * a taped straight line: distance against the tape.

Facts and limits:
  * BasaltVIO is a ThreadedHostNode: it runs on the Pi, not on the camera.
    Watch the CPU figure this tool prints; wall-e also needs the CPU.
  * Luxonis's example runs 640x400 stereo at 60 fps with the IMU at 200 Hz.
    The robot's own pipeline is capped at 15 fps for latency, so start at
    30 fps here and try 60.
  * VIO yaw is relative to the start pose, like the gyro. It does not replace
    the GPS course-over-ground lock to true north.
  * Netting over the lens or a featureless view (grass, sky) can blind it.
  * The OAK allows one process at a time: stop wall-e.service first (robot
    disarmed).
  * IMU layout (2026-11-06, depthai 3.6.1): BasaltVIO needs the RAW reports,
    the sensor's own frame that the EEPROM IMU extrinsics describe. With the
    UNCALIBRATED reports wall-e uses, a parked minute looked fine but a 90
    degree hand turn diverged (6 m of travel, heading back from -105 to -46).
    With RAW the same turn read -90.3 and held -90.6 for a minute, and a 7 m
    push read 6.79 m. So the default here is raw, not wall-e's choice.
  * USB: capped at USB 2 by default like wall-e (the USB 3 link jams the RTK
    GPS). 30 fps fits USB 2; at 60 fps USB 2 delivered about 42 poses/s.

Usage (on the robot, wall-e stopped):
  python3 tools/vio_probe.py --seconds 120 [--fps 30] [--imu-report raw]
                             [--max-usb-speed HIGH]
CSV: logs/vio_probe_YYYYmmdd_HHMMSS.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

# The same default the robot uses (oak_depth sets it before importing depthai).
os.environ.setdefault("DEPTHAI_AUTOCALIBRATION", "OFF")

from pi_app.hardware.oak_depth import imu_report_names, open_pipeline  # noqa: E402

REPORTS = {
    "raw": ("ACCELEROMETER_RAW", "GYROSCOPE_RAW"),
    "uncalibrated": ("ACCELEROMETER_UNCALIBRATED", "GYROSCOPE_UNCALIBRATED"),
}


def yaw_deg_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """Heading about the world Z axis in degrees, in (-180, 180]."""
    return math.degrees(math.atan2(2.0 * (qw * qz + qx * qy),
                                   1.0 - 2.0 * (qy * qy + qz * qz)))


def unwrap_deg(prev_unwrapped: float, prev_wrapped: float, wrapped: float) -> float:
    """Continue an unwrapped angle across the +/-180 seam."""
    step = (wrapped - prev_wrapped + 180.0) % 360.0 - 180.0
    return prev_unwrapped + step


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--seconds", type=float, default=120.0, help="run time (default 120)")
    ap.add_argument("--fps", type=float, default=30.0, help="mono camera fps (default 30)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=400)
    ap.add_argument("--imu-rate", type=int, default=200, help="IMU rate in Hz (default 200)")
    ap.add_argument("--imu-report", choices=("auto", "raw", "uncalibrated"), default="raw",
                    help="IMU report family (default raw, what BasaltVIO needs on "
                         "depthai 3.6.1); auto = the one wall-e uses")
    ap.add_argument("--max-usb-speed", default="HIGH",
                    help="depthai UsbSpeed cap: HIGH = USB 2 (default), SUPER = USB 3, "
                         "'' = depthai default")
    ap.add_argument("--out", type=Path, default=None, help="CSV path")
    args = ap.parse_args()

    try:
        import depthai as dai
    except ImportError:
        print("depthai is not installed for this interpreter")
        return 2
    if not hasattr(dai.node, "BasaltVIO"):
        print(f"depthai {dai.__version__} has no BasaltVIO node")
        return 2

    if args.imu_report == "auto":
        accel_name, gyro_name = imu_report_names(dai.__version__, dai.IMUSensor)
    else:
        accel_name, gyro_name = REPORTS[args.imu_report]

    out = args.out or (REPO / "logs" / time.strftime("vio_probe_%Y%m%d_%H%M%S.csv"))
    out.parent.mkdir(parents=True, exist_ok=True)
    print(f"depthai {dai.__version__}  {args.width}x{args.height} @ {args.fps:g} fps  "
          f"IMU {accel_name} + {gyro_name} @ {args.imu_rate} Hz  -> {out}")

    t0_cpu = time.process_time()
    t0 = time.monotonic()
    n = 0
    path_m = 0.0
    last_xyz = None
    yaw0 = None
    yaw_prev_wrapped = 0.0
    yaw_unwrapped = 0.0
    next_print = t0 + 1.0

    with open(out, "w", newline="") as fh, open_pipeline(dai, args.max_usb_speed) as pipeline:
        writer = csv.writer(fh)
        writer.writerow(["t_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw",
                         "yaw_deg", "yaw_from_start_deg", "path_m"])
        left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B,
                                                      sensorFps=args.fps)
        right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C,
                                                       sensorFps=args.fps)
        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor(getattr(dai.IMUSensor, accel_name), args.imu_rate)
        imu.enableIMUSensor(getattr(dai.IMUSensor, gyro_name), args.imu_rate)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        vio = pipeline.create(dai.node.BasaltVIO)
        left.requestOutput((args.width, args.height)).link(vio.left)
        right.requestOutput((args.width, args.height)).link(vio.right)
        imu.out.link(vio.imu)
        pose_q = vio.transform.createOutputQueue(maxSize=8, blocking=False)
        pipeline.start()
        try:
            usb = pipeline.getDefaultDevice().getUsbSpeed().name
        except Exception:
            usb = "?"
        print(f"USB {usb}; running for {args.seconds:g} s (Ctrl-C stops early)")

        try:
            while pipeline.isRunning() and time.monotonic() - t0 < args.seconds:
                msg = pose_q.tryGet()
                now = time.monotonic()
                if msg is None:
                    time.sleep(0.002)
                else:
                    p = msg.getTranslation()
                    q = msg.getQuaternion()
                    yaw = yaw_deg_from_quaternion(q.qx, q.qy, q.qz, q.qw)
                    if yaw0 is None:
                        yaw0 = yaw
                        yaw_prev_wrapped = yaw
                        yaw_unwrapped = yaw
                    else:
                        yaw_unwrapped = unwrap_deg(yaw_unwrapped, yaw_prev_wrapped, yaw)
                        yaw_prev_wrapped = yaw
                    xyz = (p.x, p.y, p.z)
                    if last_xyz is not None:
                        path_m += math.dist(xyz, last_xyz)
                    last_xyz = xyz
                    n += 1
                    writer.writerow([f"{now - t0:.3f}", f"{p.x:.4f}", f"{p.y:.4f}", f"{p.z:.4f}",
                                     f"{q.qx:.6f}", f"{q.qy:.6f}", f"{q.qz:.6f}", f"{q.qw:.6f}",
                                     f"{yaw:.2f}", f"{yaw_unwrapped - yaw0:.2f}", f"{path_m:.3f}"])
                if now >= next_print:
                    next_print = now + 1.0
                    if last_xyz is None:
                        print(f"{now - t0:6.1f} s  no pose yet")
                    else:
                        print(f"{now - t0:6.1f} s  poses {n}  yaw from start "
                              f"{yaw_unwrapped - yaw0:+7.1f} deg  xyz ({last_xyz[0]:+.2f}, "
                              f"{last_xyz[1]:+.2f}, {last_xyz[2]:+.2f}) m  path {path_m:.2f} m")
        except KeyboardInterrupt:
            pass
        pipeline.stop()

    wall = max(1e-6, time.monotonic() - t0)
    cpu = time.process_time() - t0_cpu
    print(f"done: {n} poses in {wall:.1f} s ({n / wall:.1f} Hz), process CPU "
          f"{100.0 * cpu / wall:.0f}% of one core, CSV {out}")
    return 0 if n else 1


if __name__ == "__main__":
    sys.exit(main())
