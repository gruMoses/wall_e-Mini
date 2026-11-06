#!/usr/bin/env python3
"""
vio_wall_e_bench.py — A/B bench for log-only VIO inside wall-e's camera reader.

Builds OakDepthReader the way pi_app/app/main.py does (same configs), with
OakDetectionConfig.vio_enabled forced on or off, runs it for a while with the
robot at rest, and prints what decides whether VIO can stay on:

  * depth fps, detection fps and oak.det_latency_s (camera_fps is load-bearing
    for latency; VIO must not slow the person stop);
  * IMU packets/s into the heading producer, and the boot gravity sign (ay
    about -9.7 m/s^2 in the RDF frame), so the RAW + y/z flip path is checked;
  * VIO status, pose rate, heading, roll/pitch and frame_ok;
  * process CPU.

Nothing here commands the motors. The OAK allows one process at a time:
stop wall-e.service first (robot disarmed), and start it again after.

Usage (on the robot, wall-e stopped):
  python3 tools/vio_wall_e_bench.py --vio off --seconds 45
  python3 tools/vio_wall_e_bench.py --vio on  --seconds 45
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from dataclasses import replace
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from config import Config  # noqa: E402
from pi_app.hardware.oak_depth import OakDepthReader  # noqa: E402


def build_reader(vio: bool) -> OakDepthReader:
    config = Config()
    gesture_cfg = config.gesture if config.gesture.enabled else None
    det = replace(config.oak_detection, vio_enabled=vio)
    return OakDepthReader(
        config.obstacle_avoidance, config.follow_me,
        recording_config=None,
        gesture_config=gesture_cfg,
        imu_poll_hz=float(getattr(config.imu_steering, "oak_imu_poll_hz", 60.0)),
        imu_packet_mode=str(getattr(config.imu_steering, "oak_imu_packet_mode", "latest")),
        imu_max_packets_per_poll=int(getattr(config.imu_steering, "oak_imu_max_packets_per_poll", 4)),
        detection_config=det,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--vio", choices=("on", "off"), required=True)
    ap.add_argument("--seconds", type=float, default=45.0)
    ap.add_argument("--warmup", type=float, default=10.0, help="seconds not counted (default 10)")
    args = ap.parse_args()

    reader = build_reader(args.vio == "on")
    cpu0 = time.process_time()
    t0 = time.monotonic()
    reader.start()
    lat, depth_fps, det_fps, vio_hz = [], [], [], []
    pk0 = pk_t0 = None
    ay = []
    last_print = 0.0
    try:
        while time.monotonic() - t0 < args.seconds:
            time.sleep(0.5)
            now = time.monotonic()
            h = reader.get_health()
            imu, _age = reader.get_imu_data()
            m = reader.get_imu_metrics()
            if now - t0 >= args.warmup:
                if h.get("det_latency_s") is not None:
                    lat.append(float(h["det_latency_s"]))
                depth_fps.append(float(h.get("depth_fps") or 0.0))
                det_fps.append(float(h.get("det_fps") or 0.0))
                vio_hz.append(float(h.get("vio_rate_hz") or 0.0))
                ay.append(float(imu.ay_mss))
                pk = int(m.get("packets_integrated") or 0)
                if pk0 is None:
                    pk0, pk_t0 = pk, now
            if now - last_print >= 5.0:
                last_print = now
                print(f"{now - t0:5.1f} s  depth {h.get('depth_fps')} fps  det {h.get('det_fps')} fps  "
                      f"lat {h.get('det_latency_s')} s  ay {imu.ay_mss:+.2f}  "
                      f"VIO {h.get('vio_status')} {h.get('vio_rate_hz')} Hz  "
                      f"hdg {h.get('vio_heading_deg')}  roll {h.get('vio_roll_deg')}  "
                      f"pitch {h.get('vio_pitch_deg')}  frame_ok {h.get('vio_frame_ok')}  "
                      f"flip {h.get('imu_flip_yz')}", flush=True)
        m = reader.get_imu_metrics()
        pk1, pk_t1 = int(m.get("packets_integrated") or 0), time.monotonic()
    finally:
        reader.stop()
    wall = max(1e-6, time.monotonic() - t0)
    cpu = time.process_time() - cpu0

    def q(vals, p):
        if not vals:
            return None
        vals = sorted(vals)
        return round(vals[min(len(vals) - 1, int(p * len(vals)))], 3)

    imu_rate = (pk1 - pk0) / max(1e-6, pk_t1 - pk_t0) if pk0 is not None else None
    print(f"SUMMARY vio={args.vio}: det_latency p50 {q(lat, 0.5)} p90 {q(lat, 0.9)} s; "
          f"depth fps median {statistics.median(depth_fps) if depth_fps else None}; "
          f"det fps median {statistics.median(det_fps) if det_fps else None}; "
          f"IMU packets/s {imu_rate and round(imu_rate, 1)}; "
          f"ay median {statistics.median(ay) if ay else None} m/s^2; "
          f"VIO Hz median {statistics.median(vio_hz) if vio_hz else None}; "
          f"process CPU {100.0 * cpu / wall:.0f}% of one core")
    return 0


if __name__ == "__main__":
    sys.exit(main())
