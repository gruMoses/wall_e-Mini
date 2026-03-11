#!/usr/bin/env python3
"""Replay recorded Follow Me sessions through the trail-following controller.

Reads JSON logs or MCAP telemetry, reconstructs person detections and IMU data,
and runs them through FollowMeController with trail following enabled. Outputs
a comparison of actual vs simulated steering plus a text-based trail map.

Usage:
    python -m pi_app.cli.trail_replay                     # auto-find richest FM log
    python -m pi_app.cli.trail_replay logs/run_xyz.log    # specific JSON log
    python -m pi_app.cli.trail_replay logs/oak/SESSION/    # MCAP session dir
"""

from __future__ import annotations

import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from config import FollowMeConfig
from pi_app.control.follow_me import FollowMeController, PersonDetection


@dataclass
class ReplayFrame:
    timestamp: float
    heading_deg: float
    motor_l: int
    motor_r: int
    person_x_m: float | None
    person_z_m: float | None
    person_conf: float
    person_bbox: tuple[float, float, float, float]
    tracking: bool
    actual_steer: float  # actual motor differential / 2


def load_json_log(path: str) -> list[ReplayFrame]:
    frames = []
    with open(path) as f:
        for line in f:
            try:
                d = json.loads(line)
            except (json.JSONDecodeError, ValueError):
                continue
            if d.get("mode") != "FOLLOW_ME":
                continue
            imu = d.get("imu", {})
            motor = d.get("motor", {})
            fm = d.get("follow_me", {})

            ml = int(motor.get("L", 126))
            mr = int(motor.get("R", 126))

            x = fm.get("target_x_m")
            z = fm.get("target_z_m")
            tracking = bool(fm.get("tracking"))

            frames.append(ReplayFrame(
                timestamp=float(d.get("ts", 0)),
                heading_deg=float(imu.get("heading_deg", 0)),
                motor_l=ml,
                motor_r=mr,
                person_x_m=x,
                person_z_m=z,
                person_conf=0.9 if tracking and x is not None else 0.0,
                person_bbox=(0.3, 0.1, 0.7, 0.9),
                tracking=tracking,
                actual_steer=(ml - mr) / 2.0,
            ))
    return frames


def load_mcap_session(session_dir: str) -> list[ReplayFrame]:
    mcap_path = os.path.join(session_dir, "session.mcap")
    if not os.path.exists(mcap_path):
        raise FileNotFoundError(f"No session.mcap in {session_dir}")

    from mcap.reader import make_reader

    frames = []
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for _schema, _channel, message in reader.iter_messages(topics=["/oak/telemetry"]):
            d = json.loads(message.data)
            if d.get("mode") != "FOLLOW_ME":
                continue

            ml = int(d.get("motor_left", 126))
            mr = int(d.get("motor_right", 126))
            dets = d.get("detections", [])

            x, z, conf, bbox = None, None, 0.0, (0.3, 0.1, 0.7, 0.9)
            if dets:
                det = dets[0]
                x = det.get("x_m")
                z = det.get("z_m")
                conf = det.get("conf", 0.9)
                bbox = tuple(det.get("bbox", [0.3, 0.1, 0.7, 0.9]))

            frames.append(ReplayFrame(
                timestamp=float(d.get("timestamp", 0)),
                heading_deg=0.0,  # MCAP telemetry lacks IMU heading
                motor_l=ml,
                motor_r=mr,
                person_x_m=x,
                person_z_m=z,
                person_conf=conf,
                person_bbox=bbox,
                tracking=x is not None,
                actual_steer=(ml - mr) / 2.0,
            ))
    return frames


def merge_heading_from_json(frames: list[ReplayFrame], json_log: str) -> None:
    """Backfill heading_deg from a JSON log file by nearest timestamp."""
    headings: list[tuple[float, float]] = []
    with open(json_log) as f:
        for line in f:
            try:
                d = json.loads(line)
                ts = float(d.get("ts", 0))
                h = float(d.get("imu", {}).get("heading_deg", 0))
                headings.append((ts, h))
            except (json.JSONDecodeError, ValueError, TypeError):
                continue

    if not headings:
        return

    for frame in frames:
        best_h = 0.0
        best_dt = float("inf")
        for ts, h in headings:
            dt = abs(ts - frame.timestamp)
            if dt < best_dt:
                best_dt = dt
                best_h = h
        frame.heading_deg = best_h


def find_richest_fm_log(logs_dir: str) -> str | None:
    """Find the JSON log with the most Follow Me lines."""
    import glob
    best_path = None
    best_count = 0
    for lf in glob.glob(os.path.join(logs_dir, "run_*.log")):
        count = 0
        with open(lf) as f:
            for line in f:
                if '"FOLLOW_ME"' in line:
                    count += 1
        if count > best_count:
            best_count = count
            best_path = lf
    return best_path


def replay(frames: list[ReplayFrame], config: FollowMeConfig | None = None
           ) -> list[dict]:
    if config is None:
        config = FollowMeConfig()

    ctrl = FollowMeController(config)
    results = []

    for i, frame in enumerate(frames):
        ctrl.update_pose(frame.heading_deg, frame.motor_l, frame.motor_r,
                         frame.timestamp)

        detections = []
        if frame.person_x_m is not None and frame.person_z_m is not None:
            detections.append(PersonDetection(
                x_m=frame.person_x_m,
                z_m=frame.person_z_m,
                confidence=frame.person_conf,
                bbox=frame.person_bbox,
            ))

        sim_left, sim_right = ctrl.compute(detections)
        status = ctrl.get_status()

        sim_steer = (sim_left - sim_right) / 2.0
        results.append({
            "t": frame.timestamp,
            "x_m": frame.person_x_m,
            "z_m": frame.person_z_m,
            "heading": frame.heading_deg,
            "actual_steer": frame.actual_steer,
            "sim_steer": sim_steer,
            "sim_left": sim_left,
            "sim_right": sim_right,
            "pursuit_mode": status.get("follow_me_pursuit_mode", "?"),
            "trail_length": status.get("trail_length", 0),
            "odom_x": status.get("odom_x", 0),
            "odom_y": status.get("odom_y", 0),
            "odom_theta": status.get("odom_theta_deg", 0),
            "lookahead_x": status.get("trail_lookahead_x", 0),
            "lookahead_y": status.get("trail_lookahead_y", 0),
        })

    return results


def print_report(frames: list[ReplayFrame], results: list[dict]) -> None:
    if not results:
        print("No data to report.")
        return

    t0 = results[0]["t"]
    duration = results[-1]["t"] - t0
    trail_frames = sum(1 for r in results if r["pursuit_mode"] == "trail")
    direct_frames = sum(1 for r in results if r["pursuit_mode"] == "direct")

    print("=" * 72)
    print("TRAIL FOLLOW REPLAY REPORT")
    print("=" * 72)
    print(f"Frames: {len(results)}  Duration: {duration:.1f}s")
    print(f"Trail pursuit: {trail_frames} frames ({trail_frames/len(results)*100:.0f}%)")
    print(f"Direct pursuit: {direct_frames} frames ({direct_frames/len(results)*100:.0f}%)")
    print()

    print(f"{'t':>5s} {'x_m':>5s} {'z_m':>5s} {'hdg':>5s} {'act':>5s} "
          f"{'sim':>5s} {'mode':>6s} {'trail':>5s} {'odom':>12s}")
    print("-" * 72)

    step = max(1, len(results) // 40)
    for i in range(0, len(results), step):
        r = results[i]
        t = r["t"] - t0
        x_s = f"{r['x_m']:5.2f}" if r["x_m"] is not None else "  ---"
        z_s = f"{r['z_m']:5.2f}" if r["z_m"] is not None else "  ---"
        odom_s = f"({r['odom_x']:5.2f},{r['odom_y']:5.2f})"
        print(f"{t:5.1f} {x_s} {z_s} {r['heading']:5.0f} "
              f"{r['actual_steer']:+5.1f} {r['sim_steer']:+5.1f} "
              f"{r['pursuit_mode']:>6s} {r['trail_length']:5d} {odom_s}")

    steer_diffs = [abs(r["sim_steer"] - r["actual_steer"]) for r in results]
    import statistics
    print()
    print(f"Steering difference (sim vs actual):")
    print(f"  Mean: {statistics.mean(steer_diffs):.1f} bytes")
    print(f"  P90:  {sorted(steer_diffs)[int(len(steer_diffs)*0.9)]:.1f} bytes")
    print(f"  Max:  {max(steer_diffs):.1f} bytes")


def print_trail_map(results: list[dict]) -> None:
    """Text-based bird's-eye view of the trail and robot path."""
    odom_pts = [(r["odom_x"], r["odom_y"]) for r in results]
    person_pts = []
    for r in results:
        if r["x_m"] is not None and r["z_m"] is not None:
            heading_rad = math.radians(r["heading"])
            wx = r["odom_x"] + r["z_m"] * math.cos(heading_rad) - r["x_m"] * math.sin(heading_rad)
            wy = r["odom_y"] + r["z_m"] * math.sin(heading_rad) + r["x_m"] * math.cos(heading_rad)
            person_pts.append((wx, wy))

    if not odom_pts and not person_pts:
        return

    all_pts = odom_pts + person_pts
    min_x = min(p[0] for p in all_pts)
    max_x = max(p[0] for p in all_pts)
    min_y = min(p[1] for p in all_pts)
    max_y = max(p[1] for p in all_pts)

    margin = 0.5
    min_x -= margin
    max_x += margin
    min_y -= margin
    max_y += margin

    W, H = 60, 30
    range_x = max(max_x - min_x, 0.1)
    range_y = max(max_y - min_y, 0.1)

    grid = [[" " for _ in range(W)] for _ in range(H)]

    def plot(x: float, y: float, ch: str) -> None:
        col = int((x - min_x) / range_x * (W - 1))
        row = int((1 - (y - min_y) / range_y) * (H - 1))
        col = max(0, min(W - 1, col))
        row = max(0, min(H - 1, row))
        grid[row][col] = ch

    for px, py in person_pts:
        plot(px, py, "·")
    for ox, oy in odom_pts:
        plot(ox, oy, "o")
    if odom_pts:
        plot(odom_pts[0][0], odom_pts[0][1], "S")
        plot(odom_pts[-1][0], odom_pts[-1][1], "E")
    if person_pts:
        plot(person_pts[-1][0], person_pts[-1][1], "P")

    print()
    print("=" * 72)
    print("TRAIL MAP  (o=robot  ·=person trail  S=start  E=end  P=last person)")
    print(f"  X: [{min_x:.1f}, {max_x:.1f}]m   Y: [{min_y:.1f}, {max_y:.1f}]m")
    print("=" * 72)
    for row in grid:
        print("|" + "".join(row) + "|")
    print("=" * 72)


def main() -> None:
    logs_dir = os.path.join(os.path.dirname(__file__), "..", "..", "logs")
    logs_dir = os.path.normpath(logs_dir)

    source = None
    if len(sys.argv) > 1:
        source = sys.argv[1]

    frames: list[ReplayFrame] = []

    if source and os.path.isdir(source):
        print(f"Loading MCAP session: {source}")
        frames = load_mcap_session(source)
        json_log = find_richest_fm_log(logs_dir)
        if json_log and any(f.heading_deg == 0 for f in frames):
            print(f"Backfilling heading from: {json_log}")
            merge_heading_from_json(frames, json_log)
    elif source and os.path.isfile(source):
        print(f"Loading JSON log: {source}")
        frames = load_json_log(source)
    else:
        print("Scanning for richest Follow Me log...")
        best = find_richest_fm_log(logs_dir)
        if best:
            print(f"Found: {best}")
            frames = load_json_log(best)
        else:
            print("No Follow Me logs found.")
            return

    if not frames:
        print("No Follow Me frames found in source.")
        return

    print(f"Loaded {len(frames)} Follow Me frames")
    tracking = sum(1 for f in frames if f.tracking)
    print(f"  Tracking: {tracking}/{len(frames)}")

    print("\nRunning replay with trail following...")
    results = replay(frames)

    print_report(frames, results)
    print_trail_map(results)


if __name__ == "__main__":
    main()
