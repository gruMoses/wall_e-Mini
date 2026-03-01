#!/usr/bin/env python3
"""
Analyze FOLLOW_ME behavior from structured run logs.

Supports:
  - explicit epoch window: --start-epoch/--end-epoch
  - relative window: --last-seconds
  - optional mode filter: FOLLOW_ME-only samples
"""

from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path


def _pct(values: list[float], p: float) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    i = max(0, min(len(s) - 1, int(len(s) * p / 100.0) - 1))
    return float(s[i])


def _load_rows(path: Path) -> list[dict]:
    rows: list[dict] = []
    for ln in path.read_text(errors="ignore").splitlines():
        try:
            obj = json.loads(ln)
        except Exception:
            continue
        if isinstance(obj, dict):
            rows.append(obj)
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", type=Path, required=True, help="Path to run_*.log")
    ap.add_argument("--start-epoch", type=float, default=None)
    ap.add_argument("--end-epoch", type=float, default=None)
    ap.add_argument("--last-seconds", type=float, default=None)
    ap.add_argument(
        "--follow-only",
        action="store_true",
        help="Only include rows where mode == FOLLOW_ME",
    )
    args = ap.parse_args()

    if not args.log.exists():
        print(f"error: log not found: {args.log}")
        return 2

    rows = _load_rows(args.log)
    if not rows:
        print("error: no JSON rows found")
        return 2

    start_epoch = args.start_epoch
    end_epoch = args.end_epoch
    if args.last_seconds is not None:
        end_epoch = time.time()
        start_epoch = end_epoch - float(args.last_seconds)

    windowed: list[dict] = []
    for r in rows:
        ts = r.get("ts")
        if not isinstance(ts, (int, float)):
            continue
        if start_epoch is not None and float(ts) < float(start_epoch):
            continue
        if end_epoch is not None and float(ts) > float(end_epoch):
            continue
        if args.follow_only and r.get("mode") != "FOLLOW_ME":
            continue
        windowed.append(r)

    if not windowed:
        print("error: no rows in selected window")
        return 2

    loops: list[float] = []
    mode_counts: dict[str, int] = {}
    num_persons: list[int] = []
    tracking: list[bool] = []
    z_vals: list[float] = []
    x_vals: list[float] = []

    for r in windowed:
        mode = str(r.get("mode", "UNKNOWN"))
        mode_counts[mode] = mode_counts.get(mode, 0) + 1

        loop_dt = r.get("loop_dt_ms")
        if isinstance(loop_dt, (int, float)):
            loops.append(float(loop_dt))

        fm = r.get("follow_me")
        if not isinstance(fm, dict):
            fm = {}
        n = fm.get("num_persons")
        num_persons.append(int(n) if isinstance(n, (int, float)) else 0)
        tracking.append(bool(fm.get("tracking")))
        if isinstance(fm.get("target_z_m"), (int, float)):
            z_vals.append(float(fm["target_z_m"]))
        if isinstance(fm.get("target_x_m"), (int, float)):
            x_vals.append(float(fm["target_x_m"]))

    loss_events = 0
    reacq_events = 0
    for a, b in zip(tracking, tracking[1:]):
        if a and not b:
            loss_events += 1
        if (not a) and b:
            reacq_events += 1

    runs: list[tuple[bool, int]] = []
    if tracking:
        cur = tracking[0]
        ln = 1
        for t in tracking[1:]:
            if t == cur:
                ln += 1
            else:
                runs.append((cur, ln))
                cur, ln = t, 1
        runs.append((cur, ln))

    print(f"log={args.log}")
    print(f"rows={len(windowed)}")
    print(f"mode_counts={mode_counts}")
    print(f"persons_seen_pct={round(sum(v > 0 for v in num_persons) / len(num_persons) * 100, 2)}")
    print(f"tracking_pct={round(sum(tracking) / len(tracking) * 100, 2)}")
    print(f"max_persons={max(num_persons) if num_persons else 0}")
    print(f"loss_events={loss_events}")
    print(f"reacq_events={reacq_events}")
    print(f"longest_tracking_run={max([ln for st, ln in runs if st], default=0)}")
    print(f"longest_not_tracking_run={max([ln for st, ln in runs if not st], default=0)}")

    if z_vals:
        print(
            "target_z_m="
            f"mean:{round(statistics.mean(z_vals), 2)} "
            f"min:{round(min(z_vals), 2)} max:{round(max(z_vals), 2)}"
        )
    if x_vals:
        print(
            "target_x_m="
            f"mean:{round(statistics.mean(x_vals), 2)} "
            f"min:{round(min(x_vals), 2)} max:{round(max(x_vals), 2)}"
        )
    if loops:
        print(
            "loop_dt_ms="
            f"mean:{round(statistics.mean(loops), 2)} "
            f"p95:{_pct(loops, 95)} p99:{_pct(loops, 99)} max:{max(loops)} "
            f"ge30_pct:{round(sum(v >= 30 for v in loops) / len(loops) * 100, 2)}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
