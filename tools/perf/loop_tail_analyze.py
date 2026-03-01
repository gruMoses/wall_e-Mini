#!/usr/bin/env python3
"""
Analyze loop tail-latency conditions from run_*.log JSON lines.
"""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path


def _bucket_scale(scale: float | None) -> str:
    if scale is None:
        return "none"
    if scale >= 0.95:
        return "0.95-1.00"
    if scale >= 0.75:
        return "0.75-0.95"
    if scale >= 0.50:
        return "0.50-0.75"
    if scale >= 0.25:
        return "0.25-0.50"
    return "0.00-0.25"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", type=Path, required=True, help="Path to run_*.log")
    ap.add_argument("--threshold-ms", type=float, default=30.0)
    args = ap.parse_args()

    if not args.log.exists():
        print(f"error: log not found: {args.log}")
        return 2

    total = 0
    tail = 0
    mode_ctr: Counter[str] = Counter()
    mode_tail_ctr: Counter[str] = Counter()
    armed_tail_ctr: Counter[str] = Counter()
    scale_tail_ctr: Counter[str] = Counter()
    persons_tail_ctr: Counter[str] = Counter()
    event_tail_ctr: Counter[str] = Counter()
    worst: list[tuple[float, str]] = []

    for ln in args.log.read_text(errors="ignore").splitlines():
        try:
            obj = json.loads(ln)
        except Exception:
            continue
        if not isinstance(obj, dict):
            continue
        dt = obj.get("loop_dt_ms")
        if not isinstance(dt, (int, float)):
            continue
        total += 1
        mode = str(obj.get("mode", "UNKNOWN"))
        mode_ctr[mode] += 1
        is_tail = float(dt) >= args.threshold_ms
        if not is_tail:
            continue

        tail += 1
        mode_tail_ctr[mode] += 1

        safety = obj.get("safety")
        armed = bool(safety.get("armed")) if isinstance(safety, dict) else False
        armed_tail_ctr["armed" if armed else "disarmed"] += 1

        obstacle = obj.get("obstacle")
        scale = obstacle.get("throttle_scale") if isinstance(obstacle, dict) else None
        if isinstance(scale, (int, float)):
            scale = float(scale)
        else:
            scale = None
        scale_tail_ctr[_bucket_scale(scale)] += 1

        follow = obj.get("follow_me")
        num_persons = 0
        if isinstance(follow, dict) and isinstance(follow.get("num_persons"), (int, float)):
            num_persons = int(follow["num_persons"])
        persons_tail_ctr["0" if num_persons == 0 else ">=1"] += 1

        events = obj.get("events")
        if isinstance(events, list):
            if not events:
                event_tail_ctr["none"] += 1
            else:
                for ev in events:
                    event_tail_ctr[str(ev)] += 1
        else:
            event_tail_ctr["none"] += 1

        ts = str(obj.get("ts_iso", "?"))
        worst.append((float(dt), ts))

    print(f"log={args.log}")
    print(f"samples={total}")
    print(f"tail_threshold_ms={args.threshold_ms}")
    print(f"tail_samples={tail}")
    tail_pct = (tail / total * 100.0) if total else 0.0
    print(f"tail_pct={round(tail_pct, 2)}")
    print(f"mode_dist={dict(mode_ctr)}")
    print(f"mode_tail_dist={dict(mode_tail_ctr)}")
    print(f"armed_tail_dist={dict(armed_tail_ctr)}")
    print(f"scale_tail_dist={dict(scale_tail_ctr)}")
    print(f"persons_tail_dist={dict(persons_tail_ctr)}")
    print(f"events_tail_top={dict(event_tail_ctr.most_common(8))}")

    worst_sorted = sorted(worst, key=lambda x: x[0], reverse=True)[:5]
    if worst_sorted:
        print("worst_loops_top5=")
        for dt, ts in worst_sorted:
            print(f"  {dt:.1f}ms @ {ts}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
