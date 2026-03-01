#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import statistics
import time

import depthai as dai


def percentile(values: list[float], p: float) -> float | None:
    if not values:
        return None
    s = sorted(values)
    if len(s) == 1:
        return s[0]
    k = (len(s) - 1) * (p / 100.0)
    i = int(k)
    j = min(i + 1, len(s) - 1)
    if i == j:
        return s[i]
    return s[i] + (s[j] - s[i]) * (k - i)


def build_pipeline(width: int, height: int, fps: float) -> tuple[dai.Pipeline, object]:
    pipeline = dai.Pipeline()
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    _ = fps  # Camera FPS control is API-version-specific; use default sensor FPS.
    rgb_out = cam.requestOutput((int(width), int(height)))
    return pipeline, rgb_out


def main() -> None:
    parser = argparse.ArgumentParser(description="Measure OAK frame timing and latency")
    parser.add_argument("--seconds", type=int, default=120)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--out", required=True, help="Output CSV path")
    args = parser.parse_args()

    pipeline, rgb_out = build_pipeline(args.width, args.height, args.fps)

    rows: list[tuple[float, float, int, float | None, int | None, float]] = []
    queue = rgb_out.createOutputQueue(maxSize=8, blocking=False)
    pipeline.start()
    try:
        end_t = time.monotonic() + args.seconds
        prev_dev_ts: float | None = None
        prev_seq: int | None = None

        while time.monotonic() < end_t:
            msg = queue.tryGet()
            if msg is None:
                time.sleep(0.001)
                continue

            host_ts = time.monotonic()
            dev_ts = msg.getTimestampDevice().total_seconds()
            seq = msg.getSequenceNum()

            inter_ms = None if prev_dev_ts is None else (dev_ts - prev_dev_ts) * 1000.0
            dropped = None if prev_seq is None else max(0, seq - prev_seq - 1)
            # Use host-synced timestamp for latency in a comparable time base.
            synced_ts = msg.getTimestamp().total_seconds()
            host_latency_ms = (host_ts - synced_ts) * 1000.0

            rows.append((host_ts, dev_ts, seq, inter_ms, dropped, host_latency_ms))
            prev_dev_ts = dev_ts
            prev_seq = seq
    finally:
        try:
            pipeline.stop()
        except Exception:
            pass

    with open(args.out, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(
            ["host_ts", "device_ts", "seq", "inter_ms", "dropped_frames", "host_latency_ms"]
        )
        writer.writerows(rows)

    inter_vals = [r[3] for r in rows if r[3] is not None]
    latency_vals = [r[5] for r in rows]
    drops = sum((r[4] or 0) for r in rows)

    if len(rows) > 1:
        duration = rows[-1][1] - rows[0][1]
        fps_measured = (len(rows) - 1) / duration if duration > 0 else 0.0
    else:
        fps_measured = 0.0

    print(f"frames={len(rows)} fps={fps_measured:.2f} drops={drops}")
    if inter_vals:
        print(
            "inter_ms "
            f"mean={statistics.mean(inter_vals):.2f} "
            f"p50={percentile(inter_vals, 50):.2f} "
            f"p95={percentile(inter_vals, 95):.2f} "
            f"p99={percentile(inter_vals, 99):.2f}"
        )
    if latency_vals:
        print(
            "host_latency_ms "
            f"mean={statistics.mean(latency_vals):.2f} "
            f"p50={percentile(latency_vals, 50):.2f} "
            f"p95={percentile(latency_vals, 95):.2f} "
            f"p99={percentile(latency_vals, 99):.2f}"
        )


if __name__ == "__main__":
    main()
